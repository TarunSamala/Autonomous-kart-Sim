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
const FLinearColor Backdrop(0.004f, 0.007f, 0.012f, 0.94f);
const FLinearColor Surface(0.018f, 0.026f, 0.039f, 0.98f);
const FLinearColor SurfaceRaised(0.028f, 0.039f, 0.056f, 1.0f);
const FLinearColor SurfaceSoft(0.038f, 0.051f, 0.071f, 0.9f);
const FLinearColor Border(0.09f, 0.13f, 0.18f, 1.0f);
const FLinearColor Text(0.91f, 0.94f, 0.97f, 1.0f);
const FLinearColor TextMuted(0.47f, 0.55f, 0.64f, 1.0f);
const FLinearColor Accent(0.0f, 0.78f, 0.94f, 1.0f);
const FLinearColor AccentHover(0.08f, 0.88f, 1.0f, 1.0f);
const FLinearColor AccentPressed(0.0f, 0.62f, 0.78f, 1.0f);
const FLinearColor Success(0.22f, 0.91f, 0.55f, 1.0f);
const FLinearColor Warning(1.0f, 0.68f, 0.18f, 1.0f);
const FLinearColor Danger(1.0f, 0.28f, 0.28f, 1.0f);
}

void SBenchmarkMenu::Construct(const FArguments& InArgs)
{
    OnConnectBridge = InArgs._OnConnectBridge;
    OnPrepareBenchmark = InArgs._OnPrepareBenchmark;
    OnClose = InArgs._OnClose;

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

    SelectedLidar = LidarOptions[2];
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
    SecondaryButtonStyle.SetHovered(FSlateColorBrush(FLinearColor(0.06f, 0.09f, 0.12f, 1.0f)));
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
        .Padding(FMargin(34.0f, 28.0f))
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(BenchmarkTheme::Surface)
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
        .BorderBackgroundColor(BenchmarkTheme::SurfaceRaised)
        .Padding(FMargin(26.0f, 17.0f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot()
            .AutoWidth()
            .VAlign(VAlign_Center)
            [
                SNew(SBox)
                .WidthOverride(5.0f)
                .HeightOverride(38.0f)
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
                    .Text(FText::FromString(TEXT("AUTONOMY BENCHMARK")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 19))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("FORMULA STUDENT DRIVERLESS SIMULATOR")))
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
                .BorderBackgroundColor(FLinearColor(0.03f, 0.11f, 0.12f, 1.0f))
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
        .WidthOverride(220.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(FLinearColor(0.012f, 0.018f, 0.028f, 1.0f))
            .Padding(FMargin(16.0f, 22.0f))
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 0.0f, 0.0f, 13.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("BENCHMARK MODES")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.02f, 0.16f, 0.19f, 1.0f))
                    .Padding(FMargin(13.0f, 13.0f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("01   SINGLE TEST")))
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
                .Padding(0.0f, 8.0f, 0.0f, 0.0f)
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(BenchmarkTheme::Surface)
                    .Padding(FMargin(13.0f, 13.0f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("02   MULTI TEST")))
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
                            .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        ]
                        + SVerticalBox::Slot()
                        .AutoHeight()
                        .Padding(27.0f, 4.0f, 0.0f, 0.0f)
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("Planned / competitive benchmark")))
                            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 8))
                            .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        ]
                    ]
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(8.0f, 28.0f, 0.0f, 12.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("PIPELINE")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("01")), FText::FromString(TEXT("SENSORS")), FText::FromString(TEXT("Configure")), FText::FromString(TEXT("ACTIVE")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("02")), FText::FromString(TEXT("CONTROL")), FText::FromString(TEXT("Manual")), FText::FromString(TEXT("BASELINE")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("03")), FText::FromString(TEXT("SLAM")), FText::FromString(TEXT("Not selected")), FText::FromString(TEXT("NEXT")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("04")), FText::FromString(TEXT("NAVIGATION")), FText::FromString(TEXT("Not selected")), FText::FromString(TEXT("LOCKED")))]
                + SVerticalBox::Slot().AutoHeight()[BuildStackRow(FText::FromString(TEXT("05")), FText::FromString(TEXT("FILTER")), FText::FromString(TEXT("None")), FText::FromString(TEXT("LOCKED")))]
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
        .BorderBackgroundColor(BenchmarkTheme::Surface)
        .Padding(FMargin(28.0f, 24.0f))
        [
            SNew(SScrollBox)
            + SScrollBox::Slot()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("CONFIGURE SINGLE TEST")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 26))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 7.0f, 0.0f, 24.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Build a reproducible sensor contract. Every run stores the exact profile in its benchmark manifest.")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 10))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                    .AutoWrapText(true)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(BenchmarkTheme::SurfaceRaised)
                    .Padding(FMargin(18.0f, 15.0f))
                    [
                        SNew(SHorizontalBox)
                        + SHorizontalBox::Slot()
                        .FillWidth(1.0f)
                        [
                            SNew(SVerticalBox)
                            + SVerticalBox::Slot().AutoHeight()
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString(TEXT("TRACK / ENVIRONMENT")))
                                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 9))
                                .ColorAndOpacity(BenchmarkTheme::TextMuted)
                            ]
                            + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 6.0f, 0.0f, 0.0f)
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString(TEXT("Training Map")))
                                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 15))
                                .ColorAndOpacity(BenchmarkTheme::Text)
                            ]
                        ]
                        + SHorizontalBox::Slot()
                        .AutoWidth()
                        .VAlign(VAlign_Center)
                        [
                            SNew(SBorder)
                            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                            .BorderBackgroundColor(FLinearColor(0.04f, 0.12f, 0.13f, 1.0f))
                            .Padding(FMargin(10.0f, 6.0f))
                            [
                                SNew(STextBlock)
                                .Text(FText::FromString(TEXT("MANUAL MAPPING")))
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
                    BuildSensorSelector(
                        FText::FromString(TEXT("PRIMARY RANGE SENSOR")),
                        FText::FromString(TEXT("LiDAR")),
                        FText::FromString(TEXT("Select a hardware-inspired scan contract.")),
                        &LidarOptions,
                        &SelectedLidar,
                        &LidarComboBox)
                ]
                + SVerticalBox::Slot()
                .AutoHeight()
                .Padding(0.0f, 14.0f, 0.0f, 0.0f)
                [
                    BuildSensorSelector(
                        FText::FromString(TEXT("PRIMARY VISION SENSOR")),
                        FText::FromString(TEXT("RGB-D Camera")),
                        FText::FromString(TEXT("Select a depth-camera stream and optical contract.")),
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
                    .BorderBackgroundColor(FLinearColor(0.055f, 0.045f, 0.022f, 1.0f))
                    .Padding(FMargin(15.0f, 12.0f))
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("SIMULATION FIDELITY  /  Profiles reproduce stream geometry and timing. Physical optics, weather response, transport latency and hardware noise are not yet modelled.")))
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
        .WidthOverride(330.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(FLinearColor(0.014f, 0.022f, 0.032f, 1.0f))
            .Padding(FMargin(22.0f, 24.0f))
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("RUN SUMMARY")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 16))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 5.0f, 0.0f, 20.0f)
                [
                    SNew(STextBlock)
                    .Text(FText::FromString(TEXT("Configuration preview")))
                    .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                    .ColorAndOpacity(BenchmarkTheme::TextMuted)
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(BenchmarkTheme::SurfaceRaised)
                    .Padding(FMargin(15.0f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot().AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("SENSOR LOADOUT")))
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                            .ColorAndOpacity(BenchmarkTheme::TextMuted)
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 12.0f, 0.0f, 3.0f)
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetSelectedLidarLabel)
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
                            .ColorAndOpacity(BenchmarkTheme::Text)
                        ]
                        + SVerticalBox::Slot().AutoHeight()
                        [
                            SNew(STextBlock)
                            .Text(this, &SBenchmarkMenu::GetSelectedDepthLabel)
                            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
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
                    .Text(FText::FromString(TEXT("ROS 2 BRIDGE")))
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
                        .Text(FText::FromString(TEXT("CONNECT ROS 2 BRIDGE")))
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
                    .Text(FText::FromString(TEXT("SELECT A CONFIGURATION")))
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
                        .Text(FText::FromString(TEXT("PREPARE BENCHMARK")))
                        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
                        .ColorAndOpacity(FLinearColor(0.0f, 0.06f, 0.08f, 1.0f))
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
        .BorderBackgroundColor(BenchmarkTheme::SurfaceRaised)
        .Padding(FMargin(18.0f))
        [
            SNew(SVerticalBox)
            + SVerticalBox::Slot().AutoHeight()
            [
                SNew(STextBlock)
                .Text(Eyebrow)
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 8))
                .ColorAndOpacity(BenchmarkTheme::Accent)
            ]
            + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 5.0f, 0.0f, 2.0f)
            [
                SNew(STextBlock)
                .Text(Title)
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 17))
                .ColorAndOpacity(BenchmarkTheme::Text)
            ]
            + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 0.0f, 0.0f, 13.0f)
            [
                SNew(STextBlock)
                .Text(Description)
                .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                .ColorAndOpacity(BenchmarkTheme::TextMuted)
            ]
            + SVerticalBox::Slot().AutoHeight()
            [
                SAssignNew(*ComboBox, SComboBox<FProfileOptionPtr>)
                .OptionsSource(Options)
                .InitiallySelectedItem(*Selected)
                .OnGenerateWidget(this, &SBenchmarkMenu::GenerateProfileOption)
                .OnSelectionChanged(
                    this,
                    IsLidar ? &SBenchmarkMenu::SelectLidar : &SBenchmarkMenu::SelectDepth)
                .ContentPadding(FMargin(12.0f, 8.0f))
                [
                    SNew(STextBlock)
                    .Text(IsLidar
                        ? TAttribute<FText>::Create(
                            TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetSelectedLidarLabel))
                        : TAttribute<FText>::Create(
                            TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetSelectedDepthLabel)))
                    .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
                    .ColorAndOpacity(BenchmarkTheme::Text)
                ]
            ]
            + SVerticalBox::Slot().AutoHeight().Padding(0.0f, 10.0f, 0.0f, 0.0f)
            [
                SNew(STextBlock)
                .Text(IsLidar
                    ? TAttribute<FText>::Create(
                        TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetLidarDescription))
                    : TAttribute<FText>::Create(
                        TAttribute<FText>::FGetter::CreateSP(this, &SBenchmarkMenu::GetDepthDescription)))
                .Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
                .ColorAndOpacity(BenchmarkTheme::TextMuted)
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

FText SBenchmarkMenu::GetConfigurationState() const
{
    return CanPrepare()
        ? FText::FromString(TEXT("VALID SENSOR CONTRACT"))
        : FText::FromString(TEXT("SELECT AT LEAST ONE SENSOR"));
}

FSlateColor SBenchmarkMenu::GetConfigurationStateColor() const
{
    return FSlateColor(CanPrepare() ? BenchmarkTheme::Success : BenchmarkTheme::Danger);
}

bool SBenchmarkMenu::CanPrepare() const
{
    return SelectedLidar.IsValid()
        && SelectedDepth.IsValid()
        && !(SelectedLidar->Id == TEXT("off") && SelectedDepth->Id == TEXT("off"));
}

FReply SBenchmarkMenu::HandleConnectBridge()
{
    OnConnectBridge.ExecuteIfBound();
    return FReply::Handled();
}

FReply SBenchmarkMenu::HandlePrepareBenchmark()
{
    if (CanPrepare() && OnPrepareBenchmark.IsBound())
    {
        SetPreparationStatus(
            FText::FromString(TEXT("GENERATING SETTINGS AND MANIFEST...")),
            BenchmarkTheme::Warning);
        OnPrepareBenchmark.Execute(SelectedLidar->Id, SelectedDepth->Id);
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
    if (BridgeStatusText.IsValid())
    {
        BridgeStatusText->SetText(Status);
        BridgeStatusText->SetColorAndOpacity(Color);
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
