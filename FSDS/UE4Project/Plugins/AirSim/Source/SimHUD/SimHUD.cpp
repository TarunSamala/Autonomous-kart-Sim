#include "SimHUD.h"
#include "BenchmarkMenuWidget.h"
#include "BenchmarkMenuHost.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformMisc.h"
#include "UObject/ConstructorHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

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
        GetWorldTimerManager().SetTimerForNextTick(
            this,
            &ASimHUD::createBridgeControlWidget);
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
    GetWorldTimerManager().ClearTimer(preparation_process_timer_);
    removeBridgeControlWidget();

    if (screen_message_suppression_active_ && GEngine)
    {
        GEngine->Exec(GetWorld(), TEXT("ENABLEALLSCREENMESSAGES"));
        GEngine->bEnableOnScreenDebugMessages = screen_messages_were_enabled_;
        screen_message_suppression_active_ = false;
    }

    if (bridge_process_.IsValid())
    {
        if (FPlatformProcess::IsProcRunning(bridge_process_))
        {
            FPlatformProcess::TerminateProc(bridge_process_, true);
        }
        FPlatformProcess::CloseProc(bridge_process_);
        bridge_process_.Reset();
    }

    if (preparation_process_.IsValid())
    {
        if (FPlatformProcess::IsProcRunning(preparation_process_))
        {
            FPlatformProcess::TerminateProc(preparation_process_, true);
        }
        FPlatformProcess::CloseProc(preparation_process_);
        preparation_process_.Reset();
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
    if (widget_)
    {
        widget_->toggleHelpVisibility();
    }
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

    if (!widget_)
    {
        UE_LOG(LogTemp, Error, TEXT("SimHUD: legacy HUD widget is unavailable"));
        return;
    }

    widget_->AddToViewport();

    //synchronize PIP views
    widget_->initializeForPlay();
}

void ASimHUD::createBridgeControlWidget()
{
    if (benchmark_menu_host_)
    {
        return;
    }

    if (!GEngine || !GEngine->GameViewport)
    {
        UE_LOG(LogTemp, Error, TEXT("Benchmark menu: game viewport is unavailable"));
        UAirBlueprintLib::LogMessage(
            TEXT("Cannot create ROS bridge controls: game viewport is unavailable"),
            TEXT(""),
            LogDebugLevel::Failure);
        return;
    }

    APlayerController* player_controller = GetWorld()->GetFirstPlayerController();
    if (!player_controller)
    {
        UE_LOG(LogTemp, Error, TEXT("Benchmark menu: player controller is unavailable"));
        return;
    }

    benchmark_menu_host_ = CreateWidget<UBenchmarkMenuHost>(
        player_controller,
        UBenchmarkMenuHost::StaticClass());
    if (!benchmark_menu_host_)
    {
        UE_LOG(LogTemp, Error, TEXT("Benchmark menu: failed to create native UMG host"));
        return;
    }

    benchmark_menu_host_->Configure(
        FSimpleDelegate::CreateLambda([this]() {
            connectRosBridge();
        }),
        FOnPrepareBenchmark::CreateUObject(
            this,
            &ASimHUD::prepareSingleTest),
        FSimpleDelegate::CreateUObject(
            this,
            &ASimHUD::inputEventToggleBenchmarkMenu));

    benchmark_menu_host_->AddToViewport(1000);
    benchmark_menu_ = benchmark_menu_host_->GetBenchmarkMenu();
    UE_LOG(LogTemp, Display, TEXT("Benchmark menu: added native UMG host to viewport"));
    setBenchmarkMenuVisible(true);
}

void ASimHUD::removeBridgeControlWidget()
{
    if (benchmark_menu_host_)
    {
        benchmark_menu_host_->RemoveFromParent();
        benchmark_menu_host_ = nullptr;
    }

    benchmark_menu_.Reset();
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
            FPaths::Combine(FPaths::ProjectDir(), TEXT("../ros2/scripts/single-test-bridge"))),
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
    if (benchmark_menu_.IsValid())
    {
        benchmark_menu_->SetBridgeStatus(status, color);
    }
}

void ASimHUD::prepareSingleTest(
    const FString& experiment,
    const FString& lidar_profile,
    const FString& depth_profile)
{
    if (preparation_process_.IsValid()
        && FPlatformProcess::IsProcRunning(preparation_process_))
    {
        if (benchmark_menu_.IsValid())
        {
            benchmark_menu_->SetPreparationStatus(
                FText::FromString(TEXT("PROFILE GENERATION ALREADY RUNNING")),
                FLinearColor(1.0f, 0.68f, 0.18f));
        }
        return;
    }

    const FString script_path = findSingleTestPrepareScript();
    if (script_path.IsEmpty())
    {
        if (benchmark_menu_.IsValid())
        {
            benchmark_menu_->SetPreparationStatus(
                FText::FromString(TEXT("SINGLE TEST PREPARE SCRIPT NOT FOUND")),
                FLinearColor(1.0f, 0.28f, 0.28f));
        }
        return;
    }

    const FString arguments = FString::Printf(
        TEXT("--experiment %s --lidar %s --depth-camera %s"),
        *experiment,
        *lidar_profile,
        *depth_profile);

    uint32 process_id = 0;
    preparation_process_ = FPlatformProcess::CreateProc(
        *script_path,
        *arguments,
        false,
        false,
        false,
        &process_id,
        0,
        nullptr,
        nullptr);

    if (!preparation_process_.IsValid())
    {
        if (benchmark_menu_.IsValid())
        {
            benchmark_menu_->SetPreparationStatus(
                FText::FromString(TEXT("FAILED TO START PROFILE GENERATOR")),
                FLinearColor(1.0f, 0.28f, 0.28f));
        }
        return;
    }

    preparation_started_at_seconds_ = FPlatformTime::Seconds();
    preparation_process_was_running_ = true;
    UE_LOG(
        LogTemp,
        Display,
        TEXT("Benchmark prepare: started %s %s"),
        *script_path,
        *arguments);
    GetWorldTimerManager().SetTimer(
        preparation_process_timer_,
        this,
        &ASimHUD::updatePreparationProcessState,
        0.35f,
        true);
    updatePreparationProcessState();
}

FString ASimHUD::findSingleTestPrepareScript() const
{
    const FString configured_path = FPlatformMisc::GetEnvironmentVariable(
        TEXT("FSDS_SINGLE_TEST_PREPARE_SCRIPT"));

    const TArray<FString> candidates = {
        configured_path,
        FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::ProjectDir(), TEXT("../ros2/scripts/single-test-prepare"))),
        FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::LaunchDir(), TEXT("ros2/scripts/single-test-prepare"))),
        FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::LaunchDir(), TEXT("../ros2/scripts/single-test-prepare")))
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

void ASimHUD::updatePreparationProcessState()
{
    const bool is_running = preparation_process_.IsValid()
        && FPlatformProcess::IsProcRunning(preparation_process_);

    if (is_running)
    {
        constexpr double PreparationTimeoutSeconds = 30.0;
        const double elapsed_seconds =
            FPlatformTime::Seconds() - preparation_started_at_seconds_;
        if (elapsed_seconds > PreparationTimeoutSeconds)
        {
            FPlatformProcess::TerminateProc(preparation_process_, true);
            preparation_process_was_running_ = false;
            GetWorldTimerManager().ClearTimer(preparation_process_timer_);
            UE_LOG(LogTemp, Error, TEXT("Benchmark prepare: timed out after %.1f seconds"), elapsed_seconds);
            if (benchmark_menu_.IsValid())
            {
                benchmark_menu_->SetPreparationStatus(
                    FText::FromString(TEXT("PREPARATION TIMED OUT / CHECK DISTROBOX")),
                    FLinearColor(1.0f, 0.28f, 0.28f));
            }
            return;
        }

        preparation_process_was_running_ = true;
        if (benchmark_menu_.IsValid())
        {
            benchmark_menu_->SetPreparationStatus(
                FText::FromString(TEXT("GENERATING SETTINGS AND MANIFEST...")),
                FLinearColor(1.0f, 0.68f, 0.18f));
        }
        return;
    }

    if (preparation_process_was_running_)
    {
        int32 return_code = -1;
        const bool has_return_code = preparation_process_.IsValid()
            && FPlatformProcess::GetProcReturnCode(preparation_process_, &return_code);
        const bool succeeded = has_return_code && return_code == 0;

        if (succeeded)
        {
            UE_LOG(
                LogTemp,
                Display,
                TEXT("Benchmark prepare: finished with return code %d"),
                return_code);
        }
        else
        {
            UE_LOG(
                LogTemp,
                Error,
                TEXT("Benchmark prepare: finished with return code %d"),
                has_return_code ? return_code : -1);
        }

        if (benchmark_menu_.IsValid())
        {
            benchmark_menu_->SetPreparationStatus(
                succeeded
                    ? FText::FromString(TEXT("PROFILE READY / RESTART FSDS TO APPLY"))
                    : FText::FromString(TEXT("PROFILE GENERATION FAILED / CHECK LOG")),
                succeeded
                    ? FLinearColor(0.22f, 0.91f, 0.55f)
                    : FLinearColor(1.0f, 0.28f, 0.28f));
        }
        preparation_process_was_running_ = false;
    }

    GetWorldTimerManager().ClearTimer(preparation_process_timer_);
}

void ASimHUD::setBenchmarkMenuVisible(bool visible)
{
    benchmark_menu_visible_ = visible;

    // AirSim continuously emits vehicle telemetry through Unreal's on-screen
    // debug channel. Suppress it while the full-screen menu is open so those
    // messages cannot draw over the navigation, then restore the user's prior
    // engine setting when returning to the simulation.
    if (GEngine)
    {
        if (visible && !screen_message_suppression_active_)
        {
            screen_messages_were_enabled_ = GEngine->bEnableOnScreenDebugMessages;
            GEngine->bEnableOnScreenDebugMessages = false;
            GEngine->ClearOnScreenDebugMessages();
            GEngine->Exec(GetWorld(), TEXT("DISABLEALLSCREENMESSAGES"));
            screen_message_suppression_active_ = true;
        }
        else if (!visible && screen_message_suppression_active_)
        {
            GEngine->Exec(GetWorld(), TEXT("ENABLEALLSCREENMESSAGES"));
            GEngine->bEnableOnScreenDebugMessages = screen_messages_were_enabled_;
            screen_message_suppression_active_ = false;
        }
    }

    if (benchmark_menu_host_)
    {
        benchmark_menu_host_->SetVisibility(
            visible ? ESlateVisibility::Visible : ESlateVisibility::Collapsed);
    }

    APlayerController* player_controller = GetWorld()
        ? GetWorld()->GetFirstPlayerController()
        : nullptr;
    if (!player_controller)
    {
        return;
    }

    player_controller->bShowMouseCursor = visible;
    if (visible)
    {
        FInputModeGameAndUI input_mode;
        if (benchmark_menu_.IsValid())
        {
            input_mode.SetWidgetToFocus(benchmark_menu_);
        }
        input_mode.SetHideCursorDuringCapture(false);
        player_controller->SetInputMode(input_mode);
    }
    else
    {
        player_controller->SetInputMode(FInputModeGameOnly());
    }
}

void ASimHUD::inputEventToggleBenchmarkMenu()
{
    setBenchmarkMenuVisible(!benchmark_menu_visible_);
}



void ASimHUD::setupInputBindings()
{
    // BindActionToKey registers directly on the local player controller. AHUD
    // is not a pawn, so calling AActor::EnableInput() here only emits an engine
    // error and is neither required nor valid.
    UAirBlueprintLib::BindActionToKey("InputEventToggleHelp", EKeys::F1, this, &ASimHUD::inputEventToggleHelp);
    UAirBlueprintLib::BindActionToKey(
        "InputEventToggleBenchmarkMenu",
        EKeys::F2,
        this,
        &ASimHUD::inputEventToggleBenchmarkMenu);
}
