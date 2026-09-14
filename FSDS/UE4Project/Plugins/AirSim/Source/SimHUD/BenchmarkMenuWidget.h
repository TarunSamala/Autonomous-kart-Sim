#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class SComboBoxBase;
class STextBlock;

DECLARE_DELEGATE_ThreeParams(
    FOnPrepareBenchmark,
    const FString&,
    const FString&,
    const FString&);

struct FBenchmarkProfileOption
{
    FBenchmarkProfileOption(
        const TCHAR* InId,
        const TCHAR* InLabel,
        const TCHAR* InDescription,
        const TCHAR* InTag = TEXT(""))
        : Id(InId), Label(InLabel), Description(InDescription), Tag(InTag)
    {
    }

    FString Id;
    FString Label;
    FString Description;
    FString Tag;
};

class SBenchmarkMenu : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SBenchmarkMenu) {}
        SLATE_EVENT(FSimpleDelegate, OnConnectBridge)
        SLATE_EVENT(FOnPrepareBenchmark, OnPrepareBenchmark)
        SLATE_EVENT(FSimpleDelegate, OnClose)
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs);

    void SetBridgeStatus(const FText& Status, const FLinearColor& Color);
    void SetPreparationStatus(const FText& Status, const FLinearColor& Color);

private:
    typedef TSharedPtr<FBenchmarkProfileOption> FProfileOptionPtr;

    TSharedRef<SWidget> BuildHeader();
    TSharedRef<SWidget> BuildSidebar();
    TSharedRef<SWidget> BuildConfigurationPanel();
    TSharedRef<SWidget> BuildSummaryPanel();
    TSharedRef<SWidget> BuildExperimentGrid();
    TSharedRef<SWidget> BuildExperimentCard(FProfileOptionPtr Option);
    TSharedRef<SWidget> BuildSensorSelector(
        const FText& Eyebrow,
        const FText& Title,
        const FText& Description,
        const TArray<FProfileOptionPtr>* Options,
        FProfileOptionPtr* Selected,
        TSharedPtr<class SComboBox<FProfileOptionPtr>>* ComboBox);
    TSharedRef<SWidget> BuildStackRow(
        const FText& Index,
        const FText& Label,
        const FText& Value,
        const FText& State);

    TSharedRef<SWidget> GenerateProfileOption(FProfileOptionPtr Option) const;
    void SelectLidar(FProfileOptionPtr Option, ESelectInfo::Type SelectInfo);
    void SelectDepth(FProfileOptionPtr Option, ESelectInfo::Type SelectInfo);
    void SelectExperiment(FProfileOptionPtr Option, ESelectInfo::Type SelectInfo);

    FText GetSelectedExperimentLabel() const;
    FText GetSelectedLidarLabel() const;
    FText GetSelectedDepthLabel() const;
    FText GetExperimentDescription() const;
    FText GetExperimentCardState(FProfileOptionPtr Option) const;
    FSlateColor GetExperimentCardStateColor(FProfileOptionPtr Option) const;
    FSlateColor GetExperimentCardBorderColor(FProfileOptionPtr Option) const;
    FText GetLidarDescription() const;
    FText GetDepthDescription() const;
    FText GetConfigurationState() const;
    FSlateColor GetConfigurationStateColor() const;
    bool CanPrepare() const;

    FReply HandleConnectBridge();
    FReply HandlePrepareBenchmark();
    FReply HandleExperimentCardClicked(FProfileOptionPtr Option);
    FReply HandleClose();

private:
    TArray<FProfileOptionPtr> ExperimentOptions;
    TArray<FProfileOptionPtr> LidarOptions;
    TArray<FProfileOptionPtr> DepthOptions;
    FProfileOptionPtr SelectedExperiment;
    FProfileOptionPtr SelectedLidar;
    FProfileOptionPtr SelectedDepth;

    TSharedPtr<class SComboBox<FProfileOptionPtr>> ExperimentComboBox;
    TSharedPtr<class SComboBox<FProfileOptionPtr>> LidarComboBox;
    TSharedPtr<class SComboBox<FProfileOptionPtr>> DepthComboBox;
    TSharedPtr<STextBlock> BridgeStatusText;
    TSharedPtr<STextBlock> PreparationStatusText;

    FSimpleDelegate OnConnectBridge;
    FOnPrepareBenchmark OnPrepareBenchmark;
    FSimpleDelegate OnClose;

    FButtonStyle PrimaryButtonStyle;
    FButtonStyle SecondaryButtonStyle;
    FButtonStyle GhostButtonStyle;
};
