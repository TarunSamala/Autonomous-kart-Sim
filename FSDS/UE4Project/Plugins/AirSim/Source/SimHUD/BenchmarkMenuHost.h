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
        const FOnPrepareBenchmark& InPrepareBenchmark,
        const FSimpleDelegate& InClose);

    TSharedPtr<SBenchmarkMenu> GetBenchmarkMenu() const;

protected:
    virtual TSharedRef<SWidget> RebuildWidget() override;
    virtual void ReleaseSlateResources(bool bReleaseChildren) override;

private:
    FSimpleDelegate ConnectBridge;
    FOnPrepareBenchmark PrepareBenchmark;
    FSimpleDelegate Close;
    TSharedPtr<SBenchmarkMenu> BenchmarkMenu;
};
