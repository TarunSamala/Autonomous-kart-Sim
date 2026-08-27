#!/usr/bin/env python3
"""Train and export a compact FSDS RGB or RGB-D behavior-cloning model."""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler


@dataclass(frozen=True)
class Sample:
    run: Path
    rgb: Path
    depth: Path
    steering: float
    throttle: float
    brake: float
    speed_mps: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--modality", choices=("rgb", "rgbd"), default="rgbd")
    parser.add_argument("--width", type=int, default=160)
    parser.add_argument("--height", type=int, default=96)
    parser.add_argument("--minimum-depth", type=float, default=0.2)
    parser.add_argument("--maximum-depth", type=float, default=4.0)
    parser.add_argument("--rgb-fov", type=float, default=80.9)
    parser.add_argument("--depth-fov", type=float, default=73.8)
    parser.add_argument("--maximum-speed", type=float, default=1.5)
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=1.0e-3)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def discover_runs(root: Path) -> list[Path]:
    if (root / "samples.csv").is_file():
        return [root]
    return sorted(path.parent for path in root.glob("*/samples.csv"))


def load_run(run: Path) -> list[Sample]:
    result: list[Sample] = []
    cone_counts: set[int] = set()
    with (run / "samples.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            cone_counts.add(int(row["cone_count"]))
            sample = Sample(
                run=run,
                rgb=run / row["rgb_file"],
                depth=run / row["depth_file"],
                steering=float(row["steering"]),
                throttle=float(row["throttle"]),
                brake=float(row["brake"]),
                speed_mps=float(row["speed_mps"]),
            )
            values = (sample.steering, sample.throttle, sample.brake, sample.speed_mps)
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite label in {run / 'samples.csv'}")
            if not sample.rgb.is_file() or not sample.depth.is_file():
                raise FileNotFoundError(f"missing image for sample in {run}")
            if not -1.0 <= sample.steering <= 1.0:
                raise ValueError(f"steering outside [-1, 1] in {run}")
            if not 0.0 <= sample.throttle <= 1.0 or not 0.0 <= sample.brake <= 1.0:
                raise ValueError(f"longitudinal control outside [0, 1] in {run}")
            result.append(sample)
    if len(cone_counts) > 1:
        raise ValueError(f"cone counter changed during {run}; exclude the contaminated run")
    if len(result) < 100:
        raise ValueError(f"{run} has only {len(result)} samples; collect at least 100")
    return result


def align_depth_to_rgb(
    depth_m: np.ndarray,
    rgb_shape: tuple[int, int],
    rgb_fov_deg: float,
    depth_fov_deg: float,
    missing_value: float,
) -> np.ndarray:
    rgb_height, rgb_width = rgb_shape
    scale = math.tan(math.radians(depth_fov_deg) / 2.0) / math.tan(
        math.radians(rgb_fov_deg) / 2.0
    )
    projected_width = int(np.clip(round(rgb_width * scale), 1, rgb_width))
    projected_height = int(
        np.clip(round(projected_width * depth_m.shape[0] / depth_m.shape[1]), 1, rgb_height)
    )
    resized = cv2.resize(
        depth_m, (projected_width, projected_height), interpolation=cv2.INTER_NEAREST
    )
    aligned = np.full((rgb_height, rgb_width), missing_value, dtype=np.float32)
    x = (rgb_width - projected_width) // 2
    y = (rgb_height - projected_height) // 2
    aligned[y : y + projected_height, x : x + projected_width] = resized
    return aligned


class BehaviorDataset(Dataset):
    def __init__(
        self,
        samples: list[Sample],
        modality: str,
        width: int,
        height: int,
        minimum_depth: float,
        maximum_depth: float,
        rgb_fov: float,
        depth_fov: float,
        maximum_speed: float,
        augment: bool,
    ) -> None:
        self.samples = samples
        self.modality = modality
        self.width = width
        self.height = height
        self.minimum_depth = minimum_depth
        self.maximum_depth = maximum_depth
        self.rgb_fov = rgb_fov
        self.depth_fov = depth_fov
        self.maximum_speed = maximum_speed
        self.augment = augment

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        sample = self.samples[index]
        bgr = cv2.imread(str(sample.rgb), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"cannot decode {sample.rgb}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_AREA)
        rgb = rgb.astype(np.float32) / 255.0
        if self.augment:
            gain = random.uniform(0.75, 1.25)
            bias = random.uniform(-0.04, 0.04)
            rgb = np.clip(rgb * gain + bias, 0.0, 1.0)
            if random.random() < 0.25:
                rgb = np.clip(
                    rgb + np.random.normal(0.0, 0.01, rgb.shape).astype(np.float32),
                    0.0,
                    1.0,
                )
        channels = [np.transpose(rgb, (2, 0, 1))]

        if self.modality == "rgbd":
            depth_mm = cv2.imread(str(sample.depth), cv2.IMREAD_UNCHANGED)
            if depth_mm is None or depth_mm.ndim != 2:
                raise RuntimeError(f"cannot decode metric depth {sample.depth}")
            depth_m = depth_mm.astype(np.float32) * 0.001
            invalid = (
                ~np.isfinite(depth_m)
                | (depth_m < self.minimum_depth)
                | (depth_m > self.maximum_depth)
            )
            depth_m[invalid] = self.maximum_depth
            aligned = align_depth_to_rgb(
                depth_m,
                (bgr.shape[0], bgr.shape[1]),
                self.rgb_fov,
                self.depth_fov,
                self.maximum_depth,
            )
            aligned = cv2.resize(
                aligned, (self.width, self.height), interpolation=cv2.INTER_NEAREST
            )
            normalized = np.clip(
                (aligned - self.minimum_depth) / (self.maximum_depth - self.minimum_depth),
                0.0,
                1.0,
            )
            channels.append(normalized[np.newaxis, ...])

        image = np.concatenate(channels, axis=0).astype(np.float32)
        state = np.array(
            [np.clip(sample.speed_mps / self.maximum_speed, 0.0, 2.0)], dtype=np.float32
        )
        controls = np.array(
            [sample.steering, sample.throttle, sample.brake], dtype=np.float32
        )
        return torch.from_numpy(image), torch.from_numpy(state), torch.from_numpy(controls)


class BehaviorNet(nn.Module):
    def __init__(self, channels: int, width: int, height: int) -> None:
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(channels, 24, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(48, 64, kernel_size=3, stride=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(inplace=True),
        )
        with torch.no_grad():
            feature_count = int(
                self.features(torch.zeros(1, channels, height, width)).numel()
            )
        self.visual_head = nn.Sequential(
            nn.Flatten(),
            nn.Linear(feature_count, 128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.15),
        )
        self.state = nn.Sequential(nn.Linear(1, 16), nn.ReLU(inplace=True))
        self.head = nn.Sequential(
            nn.Linear(144, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 3),
        )

    def forward(self, image: torch.Tensor, state: torch.Tensor) -> torch.Tensor:
        visual = self.visual_head(self.features(image))
        raw = self.head(torch.cat((visual, self.state(state)), dim=1))
        steering = torch.tanh(raw[:, 0:1])
        longitudinal = torch.sigmoid(raw[:, 1:3])
        return torch.cat((steering, longitudinal), dim=1)


def flatten(groups: Iterable[list[Sample]]) -> list[Sample]:
    return [sample for group in groups for sample in group]


def evaluate(
    model: nn.Module, loader: DataLoader, device: torch.device
) -> tuple[float, np.ndarray]:
    model.eval()
    total_loss = 0.0
    absolute_error = np.zeros(3, dtype=np.float64)
    count = 0
    weights = torch.tensor([2.0, 1.0, 2.0], device=device)
    with torch.no_grad():
        for image, state, target in loader:
            image, state, target = image.to(device), state.to(device), target.to(device)
            prediction = model(image, state)
            loss = ((prediction - target).square() * weights).mean()
            total_loss += loss.item() * image.shape[0]
            absolute_error += (
                torch.abs(prediction - target).sum(dim=0).cpu().numpy().astype(np.float64)
            )
            count += image.shape[0]
    return total_loss / count, absolute_error / count


def main() -> None:
    args = parse_args()
    if (
        args.width <= 0
        or args.height <= 0
        or args.maximum_depth <= args.minimum_depth
        or args.maximum_speed <= 0.0
    ):
        raise ValueError("invalid image/depth dimensions")
    if not 0.05 <= args.validation_fraction <= 0.5:
        raise ValueError("validation fraction must be in [0.05, 0.5]")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    runs = discover_runs(args.dataset)
    if len(runs) < 2:
        raise ValueError("collect at least two independent runs for run-level validation")
    run_samples = [(run, load_run(run)) for run in runs]
    random.Random(args.seed).shuffle(run_samples)
    validation_count = max(1, round(len(run_samples) * args.validation_fraction))
    validation_groups = run_samples[:validation_count]
    training_groups = run_samples[validation_count:]
    if not training_groups:
        raise ValueError("run-level split left no training run")
    training_samples = flatten(group for _, group in training_groups)
    validation_samples = flatten(group for _, group in validation_groups)

    dataset_arguments = dict(
        modality=args.modality,
        width=args.width,
        height=args.height,
        minimum_depth=args.minimum_depth,
        maximum_depth=args.maximum_depth,
        rgb_fov=args.rgb_fov,
        depth_fov=args.depth_fov,
        maximum_speed=args.maximum_speed,
    )
    training_dataset = BehaviorDataset(training_samples, augment=True, **dataset_arguments)
    validation_dataset = BehaviorDataset(validation_samples, augment=False, **dataset_arguments)
    sampling_weights = [
        1.0 + 4.0 * abs(sample.steering) + 2.0 * sample.brake
        for sample in training_samples
    ]
    sampler = WeightedRandomSampler(sampling_weights, len(sampling_weights), replacement=True)
    training_loader = DataLoader(
        training_dataset,
        batch_size=args.batch_size,
        sampler=sampler,
        num_workers=args.workers,
        pin_memory=torch.cuda.is_available(),
    )
    validation_loader = DataLoader(
        validation_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=torch.cuda.is_available(),
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = BehaviorNet(
        4 if args.modality == "rgbd" else 3, args.width, args.height
    ).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate, weight_decay=1.0e-4)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", patience=3, factor=0.5
    )
    weights = torch.tensor([2.0, 1.0, 2.0], device=device)
    args.output.mkdir(parents=True, exist_ok=True)
    best_loss = math.inf

    print(
        f"device={device} modality={args.modality} "
        f"train={len(training_samples)} validation={len(validation_samples)}"
    )
    for epoch in range(1, args.epochs + 1):
        model.train()
        running = 0.0
        seen = 0
        for image, state, target in training_loader:
            image, state, target = image.to(device), state.to(device), target.to(device)
            optimizer.zero_grad(set_to_none=True)
            prediction = model(image, state)
            loss = ((prediction - target).square() * weights).mean()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            running += loss.item() * image.shape[0]
            seen += image.shape[0]
        validation_loss, mae = evaluate(model, validation_loader, device)
        scheduler.step(validation_loss)
        print(
            f"epoch={epoch:03d} train={running / seen:.6f} val={validation_loss:.6f} "
            f"mae_steering={mae[0]:.4f} mae_throttle={mae[1]:.4f} mae_brake={mae[2]:.4f}"
        )
        if validation_loss < best_loss:
            best_loss = validation_loss
            torch.save(model.state_dict(), args.output / "model.pt")

    model.load_state_dict(torch.load(args.output / "model.pt", map_location=device))
    model.eval()
    final_loss, final_mae = evaluate(model, validation_loader, device)
    channels = 4 if args.modality == "rgbd" else 3
    image_example = torch.zeros(1, channels, args.height, args.width, device=device)
    state_example = torch.zeros(1, 1, device=device)
    onnx_path = args.output / "model.onnx"
    torch.onnx.export(
        model,
        (image_example, state_example),
        onnx_path,
        input_names=["image", "state"],
        output_names=["controls"],
        opset_version=13,
        dynamo=False,
    )

    metadata = {
        "schema_version": 1,
        "modality": args.modality,
        "input_width": args.width,
        "input_height": args.height,
        "minimum_depth_m": args.minimum_depth,
        "maximum_depth_m": args.maximum_depth,
        "rgb_horizontal_fov_deg": args.rgb_fov,
        "depth_horizontal_fov_deg": args.depth_fov,
        "maximum_speed_mps": args.maximum_speed,
        "output_order": ["steering", "throttle", "brake"],
        "training_runs": [str(run) for run, _ in training_groups],
        "validation_runs": [str(run) for run, _ in validation_groups],
        "training_samples": len(training_samples),
        "validation_samples": len(validation_samples),
        "validation_loss": final_loss,
        "validation_mae": {
            "steering": float(final_mae[0]),
            "throttle": float(final_mae[1]),
            "brake": float(final_mae[2]),
        },
        "seed": args.seed,
    }
    (args.output / "model.json").write_text(json.dumps(metadata, indent=2) + "\n")

    # Deployment compatibility check against the same OpenCV DNN API used by C++.
    network = cv2.dnn.readNetFromONNX(str(onnx_path))
    network.setInput(np.zeros((1, channels, args.height, args.width), np.float32), "image")
    network.setInput(np.zeros((1, 1), np.float32), "state")
    output = network.forward()
    if output.shape != (1, 3) or not np.isfinite(output).all():
        raise RuntimeError(f"OpenCV DNN compatibility check failed: output shape {output.shape}")
    print(f"exported {onnx_path}")
    print(f"metadata {args.output / 'model.json'}")


if __name__ == "__main__":
    main()
