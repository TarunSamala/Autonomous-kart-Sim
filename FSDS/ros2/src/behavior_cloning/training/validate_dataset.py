#!/usr/bin/env python3
"""Validate FSDS behavior-cloning run integrity without requiring PyTorch."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    runs = [args.dataset] if (args.dataset / "samples.csv").is_file() else sorted(
        path.parent for path in args.dataset.glob("*/samples.csv")
    )
    if not runs:
        raise SystemExit(f"no samples.csv found under {args.dataset}")

    total = 0
    for run in runs:
        metadata_path = run / "metadata.json"
        if not metadata_path.is_file():
            raise SystemExit(f"missing {metadata_path}")
        metadata = json.loads(metadata_path.read_text())
        rows = list(csv.DictReader((run / "samples.csv").open(newline="", encoding="utf-8")))
        if not rows:
            raise SystemExit(f"no samples in {run}")
        previous_stamp = -1
        cone_counts: set[int] = set()
        steering_values: list[float] = []
        for index, row in enumerate(rows):
            stamp = int(row["stamp_ns"])
            if stamp <= previous_stamp:
                raise SystemExit(f"non-monotonic timestamp at {run}:{index + 2}")
            previous_stamp = stamp
            values = [float(row[key]) for key in ("steering", "throttle", "brake", "speed_mps")]
            steering_values.append(values[0])
            cone_counts.add(int(row["cone_count"]))
            if not all(math.isfinite(value) for value in values):
                raise SystemExit(f"non-finite label at {run}:{index + 2}")
            rgb_path, depth_path = run / row["rgb_file"], run / row["depth_file"]
            rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            if rgb is None or depth is None:
                raise SystemExit(f"cannot decode images at {run}:{index + 2}")
            if rgb.shape[:2] != depth.shape[:2] or depth.dtype.name != "uint16":
                raise SystemExit(f"RGB/depth contract mismatch at {run}:{index + 2}")
        if len(cone_counts) > 1:
            raise SystemExit(f"cone counter changed during {run}; do not train on this run")
        statistics = metadata.get("statistics", {})
        if statistics.get("samples") != len(rows):
            raise SystemExit(
                f"sample count mismatch in {run}: metadata={statistics.get('samples')}, "
                f"csv={len(rows)}"
            )
        if statistics.get("write_failures", 0) != 0:
            raise SystemExit(f"collector reported write failures in {run}")
        turning_fraction = sum(abs(value) >= 0.10 for value in steering_values) / len(rows)
        print(
            f"{run.name}: {len(rows)} valid synchronized samples, "
            f"turning={turning_fraction:.1%}"
        )
        total += len(rows)
    print(f"dataset valid: {len(runs)} runs, {total} samples")


if __name__ == "__main__":
    main()
