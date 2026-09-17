#include "BenchmarkMenuHost.h"

void UBenchmarkMenuHost::Configure(
    const FSimpleDelegate& InConnectBridge,
    const FSimpleDelegate& InToggleManualDrive,
    const FOnPrepareBenchmark& InPrepareBenchmark,
    const FOnLaunchProfileRviz& InLaunchProfileRviz,
    const FSimpleDelegate& InClose)
{
    ConnectBridge = InConnectBridge;
    ToggleManualDrive = InToggleManualDrive;
    PrepareBenchmark = InPrepareBenchmark;
    LaunchProfileRviz = InLaunchProfileRviz;
    Close = InClose;
}

TSharedRef<SWidget> UBenchmarkMenuHost::RebuildWidget()
{
    SAssignNew(BenchmarkMenu, SBenchmarkMenu)
        .OnConnectBridge(ConnectBridge)
        .OnToggleManualDrive(ToggleManualDrive)
        .OnPrepareBenchmark(PrepareBenchmark)
        .OnLaunchProfileRviz(LaunchProfileRviz)
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
