#include "BenchmarkMenuHost.h"

void UBenchmarkMenuHost::Configure(
    const FSimpleDelegate& InConnectBridge,
    const FOnPrepareBenchmark& InPrepareBenchmark,
    const FSimpleDelegate& InClose)
{
    ConnectBridge = InConnectBridge;
    PrepareBenchmark = InPrepareBenchmark;
    Close = InClose;
}

TSharedRef<SWidget> UBenchmarkMenuHost::RebuildWidget()
{
    SAssignNew(BenchmarkMenu, SBenchmarkMenu)
        .OnConnectBridge(ConnectBridge)
        .OnPrepareBenchmark(PrepareBenchmark)
        .OnClose(Close);

    return BenchmarkMenu.ToSharedRef();
}

void UBenchmarkMenuHost::ReleaseSlateResources(bool bReleaseChildren)
{
    Super::ReleaseSlateResources(bReleaseChildren);
    BenchmarkMenu.Reset();
}

TSharedPtr<SBenchmarkMenu> UBenchmarkMenuHost::GetBenchmarkMenu() const
{
    return BenchmarkMenu;
}
