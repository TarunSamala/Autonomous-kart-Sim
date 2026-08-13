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
    
private:
    typedef common_utils::Utils Utils;
    UClass* widget_class_;

    UPROPERTY() USimHUDWidget* widget_;

    TSharedPtr<SWidget> bridge_control_widget_;
    TSharedPtr<STextBlock> bridge_status_text_;
    FProcHandle bridge_process_;
    FTimerHandle bridge_process_timer_;
    bool bridge_process_was_running_ = false;
};
