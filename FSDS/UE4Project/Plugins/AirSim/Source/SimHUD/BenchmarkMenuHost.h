#pragma once

#include "Blueprint/UserWidget.h"
#include "BenchmarkMenuWidget.h"
#include "BenchmarkMenuHost.generated.h"

UCLASS()
class AIRSIM_API UBenchmarkMenuHost : public UUserWidget
{
    GENERATED_BODY()

public:
    void Configure(
        const FSimpleDelegate& InConnectBridge,
        const FSimpleDelegate& InToggleManualDrive,
        const FOnPrepareBenchmark& InPrepareBenchmark,
        const FOnLaunchProfileRviz& InLaunchProfileRviz,
        const FSimpleDelegate& InClose);

    TSharedPtr<SBenchmarkMenu> GetBenchmarkMenu() const;

protected:
    virtual TSharedRef<SWidget> RebuildWidget() override;
    virtual void ReleaseSlateResources(bool bReleaseChildren) override;

private:
    FSimpleDelegate ConnectBridge;
    FSimpleDelegate ToggleManualDrive;
    FOnPrepareBenchmark PrepareBenchmark;
    FOnLaunchProfileRviz LaunchProfileRviz;
    FSimpleDelegate Close;
    TSharedPtr<SBenchmarkMenu> BenchmarkMenu;
};
