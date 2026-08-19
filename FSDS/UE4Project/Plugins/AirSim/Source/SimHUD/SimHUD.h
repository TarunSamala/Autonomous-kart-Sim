#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUDWidget.h"
#include "SimMode/SimModeBase.h"
#include "PIPCamera.h"
#include "api/ApiServerBase.hpp"
#include "HAL/PlatformProcess.h"
#include <memory>
#include "SimHUD.generated.h"

class STextBlock;
class SWidget;
class SBenchmarkMenu;
class UBenchmarkMenuHost;


UENUM(BlueprintType)
enum class ESimulatorMode : uint8
{
    SIM_MODE_HIL 	UMETA(DisplayName = "Hardware-in-loop")
};

UCLASS()
class AIRSIM_API ASimHUD : public AHUD
{
    GENERATED_BODY()

public:
    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;
    typedef msr::airlib::AirSimSettings AirSimSettings;

public:
    void inputEventToggleHelp();
    void inputEventToggleBenchmarkMenu();
    
    ASimHUD();
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

protected:
    virtual void setupInputBindings();
    void updateWidgetSubwindowVisibility();
    bool isWidgetSubwindowVisible(int window_index);

private:
    void initializeSubWindows();
    void createMainWidget();
    void createBridgeControlWidget();
    void removeBridgeControlWidget();
    FReply connectRosBridge();
    FString findBridgeStartScript() const;
    void updateBridgeProcessState();
    void setBridgeStatus(const FText& status, const FLinearColor& color);
    void prepareSingleTest(const FString& lidar_profile, const FString& depth_profile);
    FString findSingleTestPrepareScript() const;
    void updatePreparationProcessState();
    void setBenchmarkMenuVisible(bool visible);
    
private:
    typedef common_utils::Utils Utils;
    UClass* widget_class_;

    UPROPERTY() USimHUDWidget* widget_;

    UPROPERTY() UBenchmarkMenuHost* benchmark_menu_host_;
    TSharedPtr<SBenchmarkMenu> benchmark_menu_;
    FProcHandle bridge_process_;
    FTimerHandle bridge_process_timer_;
    bool bridge_process_was_running_ = false;
    FProcHandle preparation_process_;
    FTimerHandle preparation_process_timer_;
    bool preparation_process_was_running_ = false;
    bool benchmark_menu_visible_ = true;
    bool screen_message_suppression_active_ = false;
    bool screen_messages_were_enabled_ = true;
};
