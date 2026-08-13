#include "SimHUD.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "HAL/PlatformMisc.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"

#include "Vehicles/Car/SimModeCar.h"

#include "common/AirSimSettings.hpp"
#include <stdexcept>

ASimHUD::ASimHUD()
{
    static ConstructorHelpers::FClassFinder<UUserWidget> hud_widget_class(TEXT("WidgetBlueprint'/AirSim/Blueprints/BP_SimHUDWidget'"));
    widget_class_ = hud_widget_class.Succeeded() ? hud_widget_class.Class : nullptr;
}

void ASimHUD::BeginPlay()
{
    Super::BeginPlay();

    try
    {
        createMainWidget();
        createBridgeControlWidget();
        setupInputBindings();
    }
    catch (std::exception &ex)
    {
        UAirBlueprintLib::LogMessageString("Error at startup: ", ex.what(), LogDebugLevel::Failure);
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("Error at startup: ") + ex.what(), "Error");
    }
}


void ASimHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GetWorldTimerManager().ClearTimer(bridge_process_timer_);
    removeBridgeControlWidget();

    if (bridge_process_.IsValid())
    {
        FPlatformProcess::CloseProc(bridge_process_);
        bridge_process_.Reset();
    }

    if (widget_)
    {
        widget_->Destruct();
        widget_ = nullptr;
    }
 
    UAirBlueprintLib::OnEndPlay();

    Super::EndPlay(EndPlayReason);
}


void ASimHUD::inputEventToggleHelp()
{
    widget_->toggleHelpVisibility();
}

void ASimHUD::createMainWidget()
{
    //create main widget
    if (widget_class_ != nullptr)
    {
        APlayerController *player_controller = this->GetWorld()->GetFirstPlayerController();
        widget_ = CreateWidget<USimHUDWidget>(player_controller, widget_class_);
    }
    else
    {
        widget_ = nullptr;
        UAirBlueprintLib::LogMessage(TEXT("Cannot instantiate BP_SimHUDWidget blueprint!"), TEXT(""), LogDebugLevel::Failure);
    }

    widget_->AddToViewport();

    //synchronize PIP views
    widget_->initializeForPlay();
}

void ASimHUD::createBridgeControlWidget()
{
    if (!GEngine || !GEngine->GameViewport)
    {
        UAirBlueprintLib::LogMessage(
            TEXT("Cannot create ROS bridge controls: game viewport is unavailable"),
            TEXT(""),
            LogDebugLevel::Failure);
        return;
    }

    TSharedPtr<SOverlay> root_overlay;
    SAssignNew(root_overlay, SOverlay)
        + SOverlay::Slot()
        .HAlign(HAlign_Right)
        .VAlign(VAlign_Top)
        .Padding(FMargin(24.0f))
        [
            SNew(SBox)
            .WidthOverride(280.0f)
            [
                SNew(SBorder)
                .BorderBackgroundColor(FLinearColor(0.015f, 0.02f, 0.03f, 0.92f))
                .Padding(FMargin(16.0f, 12.0f))
                [
                    SNew(SVerticalBox)
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0.0f, 0.0f, 0.0f, 3.0f)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString(TEXT("ROS 2 BRIDGE")))
                        .ColorAndOpacity(FLinearColor(0.85f, 0.9f, 1.0f))
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding(0.0f, 0.0f, 0.0f, 10.0f)
                    [
                        SAssignNew(bridge_status_text_, STextBlock)
                        .Text(FText::FromString(TEXT("READY")))
                        .ColorAndOpacity(FLinearColor(0.95f, 0.65f, 0.1f))
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    [
                        SNew(SButton)
                        .HAlign(HAlign_Center)
                        .VAlign(VAlign_Center)
                        .ContentPadding(FMargin(14.0f, 8.0f))
                        .OnClicked(FOnClicked::CreateUObject(
                            this,
                            &ASimHUD::connectRosBridge))
                        [
                            SNew(STextBlock)
                            .Text(FText::FromString(TEXT("CONNECT ROS BRIDGE")))
                        ]
                    ]
                ]
            ]
        ];

    bridge_control_widget_ = root_overlay;
    GEngine->GameViewport->AddViewportWidgetContent(
        bridge_control_widget_.ToSharedRef(),
        100);
}

void ASimHUD::removeBridgeControlWidget()
{
    if (bridge_control_widget_.IsValid() && GEngine && GEngine->GameViewport)
    {
        GEngine->GameViewport->RemoveViewportWidgetContent(
            bridge_control_widget_.ToSharedRef());
    }

    bridge_status_text_.Reset();
    bridge_control_widget_.Reset();
}

FReply ASimHUD::connectRosBridge()
{
    if (bridge_process_.IsValid() && FPlatformProcess::IsProcRunning(bridge_process_))
    {
        setBridgeStatus(
            FText::FromString(TEXT("RUNNING")),
            FLinearColor(0.2f, 0.9f, 0.35f));
        return FReply::Handled();
    }

    const FString script_path = findBridgeStartScript();
    if (script_path.IsEmpty())
    {
        setBridgeStatus(
            FText::FromString(TEXT("START SCRIPT NOT FOUND")),
            FLinearColor(1.0f, 0.2f, 0.15f));
        UAirBlueprintLib::LogMessage(
            TEXT("Set FSDS_BRIDGE_START_SCRIPT to the absolute bridge-start path"),
            TEXT(""),
            LogDebugLevel::Failure);
        return FReply::Handled();
    }

    setBridgeStatus(
        FText::FromString(TEXT("STARTING...")),
        FLinearColor(0.95f, 0.65f, 0.1f));

    uint32 process_id = 0;
    bridge_process_ = FPlatformProcess::CreateProc(
        *script_path,
        TEXT(""),
        false,
        false,
        false,
        &process_id,
        0,
        nullptr,
        nullptr);

    if (!bridge_process_.IsValid())
    {
        setBridgeStatus(
            FText::FromString(TEXT("FAILED TO START")),
            FLinearColor(1.0f, 0.2f, 0.15f));
        return FReply::Handled();
    }

    bridge_process_was_running_ = true;
    GetWorldTimerManager().SetTimer(
        bridge_process_timer_,
        this,
        &ASimHUD::updateBridgeProcessState,
        0.5f,
        true);
    updateBridgeProcessState();
    return FReply::Handled();
}

FString ASimHUD::findBridgeStartScript() const
{
    const FString configured_path = FPlatformMisc::GetEnvironmentVariable(
        TEXT("FSDS_BRIDGE_START_SCRIPT"));

    const TArray<FString> candidates = {
        configured_path,
        FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::ProjectDir(), TEXT("../ros2/scripts/bridge-start"))),
        FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::LaunchDir(), TEXT("ros2/scripts/bridge-start"))),
        FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::LaunchDir(), TEXT("../ros2/scripts/bridge-start")))
    };

    for (const FString& candidate : candidates)
    {
        if (!candidate.IsEmpty() && FPaths::FileExists(candidate))
        {
            return candidate;
        }
    }

    return FString();
}

void ASimHUD::updateBridgeProcessState()
{
    const bool is_running = bridge_process_.IsValid()
        && FPlatformProcess::IsProcRunning(bridge_process_);

    if (is_running)
    {
        bridge_process_was_running_ = true;
        setBridgeStatus(
            FText::FromString(TEXT("RUNNING")),
            FLinearColor(0.2f, 0.9f, 0.35f));
        return;
    }

    if (bridge_process_was_running_)
    {
        int32 return_code = 0;
        const bool has_return_code = bridge_process_.IsValid()
            && FPlatformProcess::GetProcReturnCode(bridge_process_, &return_code);
        const FString status = has_return_code
            ? FString::Printf(TEXT("STOPPED (%d)"), return_code)
            : FString(TEXT("STOPPED"));
        setBridgeStatus(
            FText::FromString(status),
            FLinearColor(1.0f, 0.2f, 0.15f));
        bridge_process_was_running_ = false;
    }

    GetWorldTimerManager().ClearTimer(bridge_process_timer_);
}

void ASimHUD::setBridgeStatus(const FText& status, const FLinearColor& color)
{
    if (bridge_status_text_.IsValid())
    {
        bridge_status_text_->SetText(status);
        bridge_status_text_->SetColorAndOpacity(color);
    }
}



void ASimHUD::setupInputBindings()
{
    UAirBlueprintLib::EnableInput(this);

    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::inputEventToggleHelp);
}
