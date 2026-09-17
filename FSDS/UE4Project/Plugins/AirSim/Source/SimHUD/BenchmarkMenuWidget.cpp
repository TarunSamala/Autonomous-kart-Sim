#include "BenchmarkMenuWidget.h"

#include "Brushes/SlateColorBrush.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace BenchmarkTheme
{
// The simulator remains visible beneath a restrained, cinematic veil. The
// translucency is deliberate: this is a benchmark tool running over a live
// test environment, not a detached desktop settings window.
const FLinearColor Backdrop(0.004f, 0.004f, 0.005f, 0.84f);
const FLinearColor Surface(0.016f, 0.016f, 0.018f, 0.78f);
const FLinearColor SurfaceRaised(0.050f, 0.050f, 0.055f, 0.88f);
const FLinearColor SurfaceSoft(0.095f, 0.095f, 0.100f, 0.76f);
const FLinearColor Border(0.70f, 0.68f, 0.64f, 0.24f);
const FLinearColor Text(0.93f, 0.92f, 0.89f, 1.0f);
const FLinearColor TextMuted(0.55f, 0.54f, 0.51f, 1.0f);
const FLinearColor Accent(0.76f, 0.31f, 0.075f, 1.0f);
const FLinearColor AccentHover(0.92f, 0.43f, 0.12f, 1.0f);
const FLinearColor AccentPressed(0.58f, 0.20f, 0.040f, 1.0f);
const FLinearColor Success(0.84f, 0.83f, 0.79f, 1.0f);
const FLinearColor Warning(0.82f, 0.35f, 0.080f, 1.0f);
const FLinearColor Danger(0.96f, 0.45f, 0.14f, 1.0f);
}

void SBenchmarkMenu::Construct(const FArguments& InArgs)
{
    OnConnectBridge = InArgs._OnConnectBridge;
    OnToggleManualDrive = InArgs._OnToggleManualDrive;
    OnPrepareBenchmark = InArgs._OnPrepareBenchmark;
    OnLaunchProfileRviz = InArgs._OnLaunchProfileRviz;
    OnClose = InArgs._OnClose;

    ExperimentOptions = {
        MakeShared<FBenchmarkProfileOption>(
            TEXT("amz_style"), TEXT("AMZ Teach & Repeat"),
            TEXT("SLAM cone map · saved Delaunay centreline · Pure Pursuit"),
            TEXT("MAPPED")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("delaunay_reactive"), TEXT("Reactive Delaunay"),
            TEXT("Live LiDAR cones · local Delaunay planner · Pure Pursuit"),
            TEXT("MAPLESS")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("behavior_cloning"), TEXT("Behavior Cloning"),
            TEXT("RGB-D policy · C++ inference · safety limits"),
            TEXT("LEARNED")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("slam"), TEXT("SLAM Toolbox Track Drive"),
            TEXT("Pose graph · recorded route · Pure Pursuit"),
            TEXT("MAPPED")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("slam_stanley"), TEXT("SLAM + Stanley Track Drive"),
            TEXT("Same pose graph and route · Stanley controller"),
            TEXT("MAPPED")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("general_autonomy"), TEXT("General Autonomy Baseline"),
            TEXT("FSDS oracle · controller-isolation reference"),
            TEXT("ORACLE"))
    };

    LidarOptions = {
        MakeShared<FBenchmarkProfileOption>(
            TEXT("off"), TEXT("Disabled"),
            TEXT("Camera-only benchmark. No LiDAR point stream is generated.")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("a1m8"), TEXT("SLAMTEC RPLIDAR A1M8"),
            TEXT("2D / 12 m / 5 Hz / 8k samples per second")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("a3m1"), TEXT("SLAMTEC RPLIDAR A3M1"),
            TEXT("2D / 25 m / 10 Hz / 16k samples per second")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("fsds_16"), TEXT("FSDS 16-Layer"),
            TEXT("3D / 100 m / 10 Hz / 180 degree development profile"))
    };

    DepthOptions = {
        MakeShared<FBenchmarkProfileOption>(
            TEXT("off"), TEXT("Disabled"),
            TEXT("LiDAR-only benchmark. RGB-D topics are not generated.")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("hp60c"), TEXT("YDLIDAR HP60C"),
            TEXT("Structured light / 640x480 / 20 FPS / 0.2-4 m")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("d415"), TEXT("Intel RealSense D415"),
            TEXT("Stereo depth / 1280x720 / 30 FPS / 0.16-10 m")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("d435"), TEXT("Intel RealSense D435"),
            TEXT("Wide stereo depth / 848x480 / 30 FPS / 0.105-10 m")),
        MakeShared<FBenchmarkProfileOption>(
            TEXT("d455"), TEXT("Intel RealSense D455"),
            TEXT("Stereo depth / 848x480 / 30 FPS / 0.4-6 m"))
    };

    SelectedExperiment = ExperimentOptions[0];
    SelectedLidar = LidarOptions[3];
    SelectedDepth = DepthOptions[4];

    PrimaryButtonStyle = FCoreStyle::Get().GetWidgetStyle<FButtonStyle>("Button");
    PrimaryButtonStyle.SetNormal(FSlateColorBrush(BenchmarkTheme::Accent));
    PrimaryButtonStyle.SetHovered(FSlateColorBrush(BenchmarkTheme::AccentHover));
    PrimaryButtonStyle.SetPressed(FSlateColorBrush(BenchmarkTheme::AccentPressed));
    PrimaryButtonStyle.SetDisabled(FSlateColorBrush(BenchmarkTheme::SurfaceSoft));
    PrimaryButtonStyle.SetNormalPadding(FMargin(18.0f, 12.0f));
    PrimaryButtonStyle.SetPressedPadding(FMargin(18.0f, 13.0f, 18.0f, 11.0f));

    SecondaryButtonStyle = FCoreStyle::Get().GetWidgetStyle<FButtonStyle>("Button");
    SecondaryButtonStyle.SetNormal(FSlateColorBrush(BenchmarkTheme::SurfaceSoft));
    SecondaryButtonStyle.SetHovered(FSlateColorBrush(FLinearColor(0.14f, 0.105f, 0.080f, 1.0f)));
    SecondaryButtonStyle.SetPressed(FSlateColorBrush(BenchmarkTheme::Border));
    SecondaryButtonStyle.SetNormalPadding(FMargin(16.0f, 10.0f));
    SecondaryButtonStyle.SetPressedPadding(FMargin(16.0f, 11.0f, 16.0f, 9.0f));

    GhostButtonStyle = FCoreStyle::Get().GetWidgetStyle<FButtonStyle>("Button");
    GhostButtonStyle.SetNormal(FSlateColorBrush(FLinearColor::Transparent));
    GhostButtonStyle.SetHovered(FSlateColorBrush(BenchmarkTheme::SurfaceSoft));
    GhostButtonStyle.SetPressed(FSlateColorBrush(BenchmarkTheme::Border));

    ChildSlot
    [
        SNew(SOverlay)
        + SOverlay::Slot()
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(BenchmarkTheme::Backdrop)
        ]
        + SOverlay::Slot()
        .Padding(FMargin(52.0f, 38.0f))
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(FLinearColor::Transparent)
            .Padding(0.0f)
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    BuildHeader()
                ]
                + SVerticalBox::Slot()
                .FillHeight(1.0f)
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        BuildSidebar()
                    ]
                    + SHorizontalBox::Slot()
                    .FillWidth(1.0f)
                    [
                        BuildConfigurationPanel()
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    [
                        BuildSummaryPanel()
                    ]
                ]
            ]
        ]
    ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildHeader()
{
    return SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(0.012f, 0.012f, 0.014f, 0.56f))
        .Padding(FMargin(18.0f, 12.0f, 18.0f, 20.0f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .AutoWidth()
            .VAlign(VAlign_Center)
            [
                SNew(SBox)
                .WidthOverride(5.0f)
                .HeightOverride(46.0f)
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(BenchmarkTheme::Accent)
                ]
            ]
            + SHorizontalBox::Slot()
            .AutoWidth()
            .Padding(14.0f, 0.0f)
            .VAlign(VAlign_Center)
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("DRIVERLESS  /  BENCHMARK")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 22))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("AUTONOMY PERFORMANCE LAB  ·  FORMULA STUDENT DRIVERLESS SIMULATOR")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
            ]
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            [
                SNew(SSpacer)
            ]
            + SHorizontalBox::Slot()
            .AutoWidth()
            .Padding(0.0f, 0.0f, 14.0f, 0.0f)
            .VAlign(VAlign_Center)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                .BorderBackgroundColor(FLinearColor(0.10f, 0.060f, 0.030f, 0.82f))
                .Padding(FMargin(11.0f, 6.0f))
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("SINGLE TEST  /  CONFIGURATION")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::Accent)
                ]
            ]
            + SHorizontalBox::Slot()
            .AutoWidth()
            .VAlign(VAlign_Center)
            [
                SNew(SButton)
                .ButtonStyle(&GhostButtonStyle)
                .ContentPadding(FMargin(10.0f, 7.0f))
                .ToolTipText(FText::FromString(TEXT("Close benchmark menu (F2)")))
                .OnClicked(this, &SBenchmarkMenu::HandleClose)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("CLOSE  [F2]")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildSidebar()
{
    return SNew(SBox)
        .WidthOverride(252.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(FLinearColor(0.012f, 0.012f, 0.014f, 0.72f))
            .Padding(FMargin(18.0f, 24.0f, 24.0f, 18.0f))
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 0.0f, 0.0f, 13.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("TEST MODE")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.12f, 0.070f, 0.032f, 0.86f))
                    .Padding(FMargin(13.0f, 13.0f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("01    SINGLE TEST")))
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
                            .ColorAndOpacity(BenchmarkTheme::Text)
                        ]
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        .Padding(27.0f, 4.0f, 0.0f, 0.0f)
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("One vehicle / controlled comparison")))
                            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
                            .ColorAndOpacity(BenchmarkTheme::Accent)
                        ]
                    ]
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 28.0f, 0.0f, 12.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("AUTONOMY STACK")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("01")), FText::FromString(TEXT("METHOD")), FText::FromString(TEXT("Select profile")), FText::FromString(TEXT("ACTIVE")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("02")), FText::FromString(TEXT("SENSORS")), FText::FromString(TEXT("Configure")), FText::FromString(TEXT("ACTIVE")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("03")), FText::FromString(TEXT("PROTOCOL")), FText::FromString(TEXT("Manifest")), FText::FromString(TEXT("RECORDED")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("04")), FText::FromString(TEXT("EXECUTION")), FText::FromString(TEXT("Safe arm")), FText::FromString(TEXT("TERMINAL")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("05")), FText::FromString(TEXT("RESULTS")), FText::FromString(TEXT("Metrics JSON")), FText::FromString(TEXT("RECORDED")))]
                + SVerticalBox::Slot()
                .FillHeight(1.0f)
                [
                    SNew(SSpacer)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(BenchmarkTheme::SurfaceSoft)
                    .Padding(FMargin(12.0f))
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("F2  TOGGLE MENU\nF1  FSDS HELP")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                        .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        .LineHeightPercentage(1.35f)
                    ]
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildConfigurationPanel()
{
    return SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(0.010f, 0.010f, 0.012f, 0.64f))
        .Padding(FMargin(38.0f, 26.0f, 34.0f, 24.0f))
        [
            SNew(SScrollBox)
            + SScrollBox::Slot()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Autonomy Experiment")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 28))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 7.0f, 0.0f, 24.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Choose a reproducible autonomy method, then define its perception hardware contract.")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 11))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor::Transparent)
                    .Padding(FMargin(10.0f, 12.0f))
                    [
                        SNew(SHorizontalBox)
                        + SHorizontalBox::Slot()
                        .FillWidth(1.0f)
                        [
                            SNew(SVerticalBox)
                            + SVerticalBox::Slot().AutoHeight()
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString(TEXT("TEST ENVIRONMENT")))
                                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                                .ColorAndOpacity(BenchmarkTheme::TextMuted)
                            ]
                            + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 6.0f, 0.0f, 0.0f)
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString(TEXT("FSDS Training Track")))
                                .Font(FCoreStyle::GetDefaultFontStyle("Regular", 16))
                                .ColorAndOpacity(BenchmarkTheme::Text)
                            ]
                        ]
                        + SHorizontalBox::Slot()
                        .AutoWidth()
                        .VAlign(VAlign_Center)
                        [
                            SNew(SBorder)
                            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                            .BorderBackgroundColor(FLinearColor(0.095f, 0.060f, 0.034f, 0.72f))
                            .Padding(FMargin(10.0f, 6.0f))
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString(TEXT("RESEARCH PROTOCOL")))
                                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                                .ColorAndOpacity(BenchmarkTheme::Success)
                            ]
                        ]
                    ]
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 14.0f, 0.0f, 0.0f)
                [
                    BuildExperimentGrid()
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 14.0f, 0.0f, 0.0f)
                [
                    BuildSensorSelector(
                        FText::FromString(TEXT("RANGE SENSOR")),
                        FText::FromString(TEXT("LiDAR")),
                        FText::FromString(TEXT("LiDAR profile")),
                        &LidarOptions,
                        &SelectedLidar,
                        &LidarComboBox)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 14.0f, 0.0f, 0.0f)
                [
                    BuildSensorSelector(
                        FText::FromString(TEXT("VISION SENSOR")),
                        FText::FromString(TEXT("Depth Camera")),
                        FText::FromString(TEXT("RGB-D profile")),
                        &DepthOptions,
                        &SelectedDepth,
                        &DepthComboBox)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 18.0f, 0.0f, 0.0f)
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.10f, 0.060f, 0.025f, 0.58f))
                    .Padding(FMargin(15.0f, 12.0f))
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("SIMULATION NOTE  ·  Profiles reproduce geometry and timing. Hardware noise, optics and transport latency are not yet modelled.")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                        .ColorAndOpacity(BenchmarkTheme::Warning)
                        .AutoWrapText(true)
                    ]
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildSummaryPanel()
{
    return SNew(SBox)
        .WidthOverride(400.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(FLinearColor(0.010f, 0.010f, 0.012f, 0.78f))
            .Padding(FMargin(34.0f, 26.0f, 26.0f, 20.0f))
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Autonomy Benchmark")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 20))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 14.0f, 0.0f, 24.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Compares executable autonomy methods under a traceable protocol. Sensors, algorithms, metrics and simulator-oracle use are written into the run manifest.")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 11))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                    .LineHeightPercentage(1.35f)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.065f, 0.065f, 0.068f, 0.72f))
                    .Padding(FMargin(18.0f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot().AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("SELECTED LOADOUT")))
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                            .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 3.0f)
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetSelectedExperimentLabel)
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
                            .ColorAndOpacity(BenchmarkTheme::Accent)
                            .AutoWrapText(true)
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 8.0f, 0.0f, 3.0f)
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetExperimentDescription)
                            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
                            .ColorAndOpacity(BenchmarkTheme::TextMuted)
                            .AutoWrapText(true)
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 3.0f)
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetSelectedLidarLabel)
                            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 12))
                            .ColorAndOpacity(BenchmarkTheme::Text)
                        ]
                        + SVerticalBox::Slot().AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetSelectedDepthLabel)
                            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 12))
                            .ColorAndOpacity(BenchmarkTheme::Text)
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 14.0f, 0.0f, 0.0f)
                        [
                            SNew(SSeparator)
                            .SeparatorImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                            .ColorAndOpacity(BenchmarkTheme::Border)
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 0.0f)
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetConfigurationState)
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                            .ColorAndOpacity(this, &SBenchmarkMenu::GetConfigurationStateColor)
                        ]
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 18.0f, 0.0f, 6.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("SIMULATOR CONNECTION")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SAssignNew(BridgeStatusText, STextBlock)
                    .Text(FText::FromString(TEXT("NOT CONNECTED")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
                    .ColorAndOpacity(BenchmarkTheme::Warning)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 10.0f, 0.0f, 0.0f)
                [
                    SNew(SButton)
                    .ButtonStyle(&SecondaryButtonStyle)
                    .HAlign(HAlign_Center)
                    .OnClicked(this, &SBenchmarkMenu::HandleConnectBridge)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("CONNECT ROS 2")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
                        .ColorAndOpacity(BenchmarkTheme::Text)
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 18.0f, 0.0f, 6.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("VISUALIZATION")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SAssignNew(RvizStatusText, STextBlock)
                    .Text(FText::FromString(TEXT("NOT OPEN")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 10.0f, 0.0f, 0.0f)
                [
                    SNew(SButton)
                    .ButtonStyle(&SecondaryButtonStyle)
                    .HAlign(HAlign_Center)
                    .ToolTipText(FText::FromString(TEXT("Open RViz with the layout for the selected experiment. Does not start or arm autonomy.")))
                    .OnClicked(this, &SBenchmarkMenu::HandleLaunchRviz)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("OPEN PROFILE RVIZ")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
                        .ColorAndOpacity(BenchmarkTheme::Text)
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 20.0f, 0.0f, 6.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("MANUAL CONTROL")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SAssignNew(ManualDriveStatusText, STextBlock)
                    .Text(FText::FromString(TEXT("STOPPED")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 10.0f, 0.0f, 0.0f)
                [
                    SNew(SButton)
                    .ButtonStyle(&SecondaryButtonStyle)
                    .IsEnabled(this, &SBenchmarkMenu::CanToggleManualDrive)
                    .HAlign(HAlign_Center)
                    .ToolTipText(FText::FromString(TEXT("Launch C++ keyboard control. Press E to arm, W/A/S/D to drive, and Space to brake.")))
                    .OnClicked(this, &SBenchmarkMenu::HandleToggleManualDrive)
                    [
                        SNew(STextBlock)
                        .Text(this, &SBenchmarkMenu::GetManualDriveButtonLabel)
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 10))
                        .ColorAndOpacity(BenchmarkTheme::Text)
                    ]
                ]
                + SVerticalBox::Slot().FillHeight(1.0f)
                [
                    SNew(SSpacer)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 0.0f, 0.0f, 8.0f)
                [
                    SAssignNew(PreparationStatusText, STextBlock)
                    .Text(FText::FromString(TEXT("READY TO PREPARE")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(SButton)
                    .ButtonStyle(&PrimaryButtonStyle)
                    .IsEnabled(this, &SBenchmarkMenu::CanPrepare)
                    .HAlign(HAlign_Center)
                    .OnClicked(this, &SBenchmarkMenu::HandlePrepareBenchmark)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("APPLY & PREPARE TEST")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
                        .ColorAndOpacity(FLinearColor(0.035f, 0.020f, 0.010f, 1.0f))
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 10.0f, 0.0f, 0.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Sensor changes require an FSDS restart before the run can begin.")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildExperimentGrid()
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot()
        .AutoHeight()
        .Padding(2.0f, 0.0f, 0.0f, 9.0f)
        [
            SNew(STextBlock)
            .Text(FText::FromString(TEXT("AUTONOMY PROFILE")))
            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
            .ColorAndOpacity(BenchmarkTheme::TextMuted)
        ]
        + SVerticalBox::Slot()
        .AutoHeight()
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(0.0f, 0.0f, 6.0f, 6.0f)
            [
                BuildExperimentCard(ExperimentOptions[0])
            ]
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(6.0f, 0.0f, 0.0f, 6.0f)
            [
                BuildExperimentCard(ExperimentOptions[1])
            ]
        ]
        + SVerticalBox::Slot()
        .AutoHeight()
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(0.0f, 6.0f, 6.0f, 0.0f)
            [
                BuildExperimentCard(ExperimentOptions[2])
            ]
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(6.0f, 6.0f, 0.0f, 0.0f)
            [
                BuildExperimentCard(ExperimentOptions[3])
            ]
        ]
        + SVerticalBox::Slot()
        .AutoHeight()
        .Padding(0.0f, 12.0f, 0.0f, 0.0f)
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(0.0f, 0.0f, 6.0f, 0.0f)
            [
                BuildExperimentCard(ExperimentOptions[4])
            ]
            + SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(6.0f, 0.0f, 0.0f, 0.0f)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                .BorderBackgroundColor(BenchmarkTheme::Surface)
                .Padding(16.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("MANUAL TEACH LAP\nRecord the map and route once, then compare both mapped controllers.")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildExperimentCard(FProfileOptionPtr Option)
{
    return SNew(SBox)
        .HeightOverride(112.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(
                this,
                &SBenchmarkMenu::GetExperimentCardBorderColor,
                Option)
            .Padding(1.0f)
            [
                SNew(SButton)
                .ButtonStyle(&SecondaryButtonStyle)
                .ContentPadding(FMargin(16.0f, 13.0f))
                .ToolTipText(FText::FromString(Option->Description))
                .OnClicked(this, &SBenchmarkMenu::HandleExperimentCardClicked, Option)
                [
                    SNew(SVerticalBox)
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(SHorizontalBox)
                        + SHorizontalBox::Slot()
                        .FillWidth(1.0f)
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(Option->Tag))
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                            .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        ]
                        + SHorizontalBox::Slot()
                        .AutoWidth()
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetExperimentCardState, Option)
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                            .ColorAndOpacity(
                                this,
                                &SBenchmarkMenu::GetExperimentCardStateColor,
                                Option)
                        ]
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0.0f, 8.0f, 0.0f, 4.0f)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(Option->Label))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
                        .ColorAndOpacity(BenchmarkTheme::Text)
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(Option->Description))
                        .Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
                        .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        .AutoWrapText(true)
                    ]
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildSensorSelector(
    const FText& Eyebrow,
    const FText& Title,
    const FText& Description,
    const TArray<FProfileOptionPtr>* Options,
    FProfileOptionPtr* Selected,
    TSharedPtr<SComboBox<FProfileOptionPtr>>* ComboBox)
{
    const bool IsLidar = Options == &LidarOptions;

    return SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(0.035f, 0.035f, 0.038f, 0.52f))
        .Padding(FMargin(18.0f, 16.0f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .FillWidth(0.42f)
            .VAlign(VAlign_Center)
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(Eyebrow)
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                    .ColorAndOpacity(BenchmarkTheme::Accent)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 7.0f, 0.0f, 3.0f)
                [
                    SNew(STextBlock)
                    .Text(Title)
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 18))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(Description)
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
            ]
            + SHorizontalBox::Slot()
            .FillWidth(0.58f)
            .Padding(24.0f, 0.0f, 0.0f, 0.0f)
            .VAlign(VAlign_Center)
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SAssignNew(*ComboBox, SComboBox<FProfileOptionPtr>)
                    .OptionsSource(Options)
                    .InitiallySelectedItem(*Selected)
                    .OnGenerateWidget(this, &SBenchmarkMenu::GenerateProfileOption)
                    .OnSelectionChanged(
                        this,
                        IsLidar ? &SBenchmarkMenu::SelectLidar : &SBenchmarkMenu::SelectDepth)
                    .ContentPadding(FMargin(14.0f, 9.0f))
                    [
                        SNew(STextBlock)
                        .Text(IsLidar
                            ? TAttribute<FText>::Create(
                                TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetSelectedLidarLabel))
                            : TAttribute<FText>::Create(
                                TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetSelectedDepthLabel)))
                        .Font(FCoreStyle::GetDefaultFontStyle("Regular", 12))
                        .ColorAndOpacity(BenchmarkTheme::Text)
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(2.0f, 9.0f, 0.0f, 0.0f)
                [
                    SNew(STextBlock)
                    .Text(IsLidar
                        ? TAttribute<FText>::Create(
                            TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetLidarDescription))
                        : TAttribute<FText>::Create(
                            TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetDepthDescription)))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::BuildStackRow(
    const FText& Index,
    const FText& Label,
    const FText& Value,
    const FText& State)
{
    return SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor::Transparent)
        .Padding(FMargin(8.0f, 8.0f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
            [
                SNew(STextBlock)
                .Text(Index)
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                .ColorAndOpacity(BenchmarkTheme::TextMuted)
            ]
            + SHorizontalBox::Slot().FillWidth(1.0f).Padding(10.0f, 0.0f)
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(Label)
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(Value)
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
            ]
            + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
            [
                SNew(STextBlock)
                .Text(State)
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 7))
                .ColorAndOpacity(State.EqualTo(FText::FromString(TEXT("ACTIVE")))
                    ? BenchmarkTheme::Accent
                    : BenchmarkTheme::TextMuted)
            ]
        ];
}

TSharedRef<SWidget> SBenchmarkMenu::GenerateProfileOption(FProfileOptionPtr Option) const
{
    return SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(BenchmarkTheme::SurfaceRaised)
        .Padding(FMargin(10.0f, 8.0f))
        [
            SNew(STextBlock)
            .Text(FText::FromString(Option.IsValid() ? Option->Label : TEXT("Unavailable")))
            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 10))
            .ColorAndOpacity(BenchmarkTheme::Text)
        ];
}

void SBenchmarkMenu::SelectLidar(FProfileOptionPtr Option, ESelectInfo::Type)
{
    if (Option.IsValid())
    {
        SelectedLidar = Option;
        SetPreparationStatus(
            FText::FromString(TEXT("CONFIGURATION CHANGED / PREPARE TO APPLY")),
            BenchmarkTheme::Warning);
    }
}

void SBenchmarkMenu::SelectDepth(FProfileOptionPtr Option, ESelectInfo::Type)
{
    if (Option.IsValid())
    {
        SelectedDepth = Option;
        SetPreparationStatus(
            FText::FromString(TEXT("CONFIGURATION CHANGED / PREPARE TO APPLY")),
            BenchmarkTheme::Warning);
    }
}

void SBenchmarkMenu::SelectExperiment(FProfileOptionPtr Option, ESelectInfo::Type)
{
    if (!Option.IsValid())
    {
        return;
    }

    SelectedExperiment = Option;
    if (Option->Id == TEXT("behavior_cloning"))
    {
        SelectedLidar = LidarOptions[0];
        SelectedDepth = DepthOptions[1];
    }
    else if (Option->Id == TEXT("slam") || Option->Id == TEXT("slam_stanley"))
    {
        SelectedLidar = LidarOptions[2];
        SelectedDepth = DepthOptions[4];
    }
    else
    {
        SelectedLidar = LidarOptions[3];
        SelectedDepth = DepthOptions[4];
    }
    if (LidarComboBox.IsValid())
    {
        LidarComboBox->SetSelectedItem(SelectedLidar);
    }
    if (DepthComboBox.IsValid())
    {
        DepthComboBox->SetSelectedItem(SelectedDepth);
    }
    SetPreparationStatus(
        FText::FromString(TEXT("EXPERIMENT CHANGED / PREPARE TO APPLY")),
        BenchmarkTheme::Warning);
}

FText SBenchmarkMenu::GetSelectedExperimentLabel() const
{
    return FText::FromString(
        SelectedExperiment.IsValid() ? SelectedExperiment->Label : TEXT("Unavailable"));
}

FText SBenchmarkMenu::GetSelectedLidarLabel() const
{
    return FText::FromString(SelectedLidar.IsValid() ? SelectedLidar->Label : TEXT("Unavailable"));
}

FText SBenchmarkMenu::GetSelectedDepthLabel() const
{
    return FText::FromString(SelectedDepth.IsValid() ? SelectedDepth->Label : TEXT("Unavailable"));
}

FText SBenchmarkMenu::GetLidarDescription() const
{
    return FText::FromString(SelectedLidar.IsValid() ? SelectedLidar->Description : TEXT(""));
}

FText SBenchmarkMenu::GetDepthDescription() const
{
    return FText::FromString(SelectedDepth.IsValid() ? SelectedDepth->Description : TEXT(""));
}

FText SBenchmarkMenu::GetExperimentDescription() const
{
    return FText::FromString(
        SelectedExperiment.IsValid() ? SelectedExperiment->Description : TEXT(""));
}

FText SBenchmarkMenu::GetExperimentCardState(FProfileOptionPtr Option) const
{
    return Option.IsValid() && Option == SelectedExperiment
        ? FText::FromString(TEXT("SELECTED"))
        : FText::FromString(TEXT("SELECT"));
}

FSlateColor SBenchmarkMenu::GetExperimentCardStateColor(FProfileOptionPtr Option) const
{
    return FSlateColor(
        Option.IsValid() && Option == SelectedExperiment
            ? BenchmarkTheme::Accent
            : BenchmarkTheme::TextMuted);
}

FSlateColor SBenchmarkMenu::GetExperimentCardBorderColor(FProfileOptionPtr Option) const
{
    return FSlateColor(
        Option.IsValid() && Option == SelectedExperiment
            ? BenchmarkTheme::Accent
            : BenchmarkTheme::Border);
}

FText SBenchmarkMenu::GetConfigurationState() const
{
    if (!SelectedExperiment.IsValid())
    {
        return FText::FromString(TEXT("SELECT AN AUTONOMY PROFILE"));
    }
    if (!SelectedLidar.IsValid() || !SelectedDepth.IsValid() ||
        (SelectedLidar->Id == TEXT("off") && SelectedDepth->Id == TEXT("off")))
    {
        return FText::FromString(TEXT("SELECT AT LEAST ONE SENSOR"));
    }
    if (SelectedExperiment->Id == TEXT("behavior_cloning") &&
        SelectedDepth->Id == TEXT("off"))
    {
        return FText::FromString(TEXT("BEHAVIOR CLONING REQUIRES RGB-D"));
    }
    if ((SelectedExperiment->Id == TEXT("amz_style") ||
         SelectedExperiment->Id == TEXT("delaunay_reactive") ||
         SelectedExperiment->Id == TEXT("slam") ||
         SelectedExperiment->Id == TEXT("slam_stanley")) &&
        SelectedLidar->Id == TEXT("off"))
    {
        return FText::FromString(TEXT("THIS PROFILE REQUIRES LIDAR"));
    }
    return FText::FromString(TEXT("READY TO PREPARE"));
}

FSlateColor SBenchmarkMenu::GetConfigurationStateColor() const
{
    return FSlateColor(CanPrepare() ? BenchmarkTheme::Success : BenchmarkTheme::Danger);
}

FText SBenchmarkMenu::GetManualDriveButtonLabel() const
{
    return FText::FromString(
        IsManualDriveRunning ? TEXT("STOP MANUAL DRIVE") : TEXT("START MANUAL DRIVE"));
}

bool SBenchmarkMenu::CanToggleManualDrive() const
{
    return IsBridgeRunning || IsManualDriveRunning;
}

bool SBenchmarkMenu::CanPrepare() const
{
    if (!SelectedExperiment.IsValid() || !SelectedLidar.IsValid() || !SelectedDepth.IsValid())
    {
        return false;
    }
    if (SelectedExperiment->Id == TEXT("behavior_cloning") &&
        SelectedDepth->Id == TEXT("off"))
    {
        return false;
    }
    if ((SelectedExperiment->Id == TEXT("amz_style") ||
         SelectedExperiment->Id == TEXT("slam") ||
         SelectedExperiment->Id == TEXT("slam_stanley")) &&
        SelectedLidar->Id == TEXT("off"))
    {
        return false;
    }
    return !(SelectedLidar->Id == TEXT("off") && SelectedDepth->Id == TEXT("off"));
}

FReply SBenchmarkMenu::HandleConnectBridge()
{
    OnConnectBridge.ExecuteIfBound();
    return FReply::Handled();
}

FReply SBenchmarkMenu::HandleToggleManualDrive()
{
    OnToggleManualDrive.ExecuteIfBound();
    return FReply::Handled();
}

FReply SBenchmarkMenu::HandleExperimentCardClicked(FProfileOptionPtr Option)
{
    SelectExperiment(Option, ESelectInfo::Direct);
    return FReply::Handled();
}

FReply SBenchmarkMenu::HandlePrepareBenchmark()
{
    if (CanPrepare() && OnPrepareBenchmark.IsBound())
    {
        SetPreparationStatus(
            FText::FromString(TEXT("GENERATING SETTINGS AND MANIFEST...")),
            BenchmarkTheme::Warning);
        OnPrepareBenchmark.Execute(
            SelectedExperiment->Id,
            SelectedLidar->Id,
            SelectedDepth->Id);
    }
    return FReply::Handled();
}

FReply SBenchmarkMenu::HandleLaunchRviz()
{
    if (SelectedExperiment.IsValid())
    {
        OnLaunchProfileRviz.ExecuteIfBound(SelectedExperiment->Id);
    }
    return FReply::Handled();
}

FReply SBenchmarkMenu::HandleClose()
{
    OnClose.ExecuteIfBound();
    return FReply::Handled();
}

void SBenchmarkMenu::SetBridgeStatus(const FText& Status, const FLinearColor& Color)
{
    IsBridgeRunning = Status.EqualTo(FText::FromString(TEXT("RUNNING")));
    if (BridgeStatusText.IsValid())
    {
        BridgeStatusText->SetText(Status);
        BridgeStatusText->SetColorAndOpacity(Color);
    }
}

void SBenchmarkMenu::SetManualDriveStatus(
    const FText& Status,
    const FLinearColor& Color,
    bool IsRunning)
{
    IsManualDriveRunning = IsRunning;
    if (ManualDriveStatusText.IsValid())
    {
        ManualDriveStatusText->SetText(Status);
        ManualDriveStatusText->SetColorAndOpacity(Color);
    }
}

void SBenchmarkMenu::SetPreparationStatus(const FText& Status, const FLinearColor& Color)
{
    if (PreparationStatusText.IsValid())
    {
        PreparationStatusText->SetText(Status);
        PreparationStatusText->SetColorAndOpacity(Color);
    }
}

void SBenchmarkMenu::SetRvizStatus(const FText& Status, const FLinearColor& Color)
{
    if (RvizStatusText.IsValid())
    {
        RvizStatusText->SetText(Status);
        RvizStatusText->SetColorAndOpacity(Color);
    }
}
