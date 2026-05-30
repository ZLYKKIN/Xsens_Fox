// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnSourceEditor.h"
#include "LiveLinkMessages.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "MvnLiveLinkPresenceDetector.h"
#include "MessageEndpointBuilder.h"
#include <LiveLinkMessageBusFinder.h>
#include "LiveLinkMvnPlugin.h"
#include "Misc/App.h"

#define LOCTEXT_NAMESPACE "LiveLinkMvnSourceEditor"

//////////////////////////////////////////////////////////////////////////
//// CPP Only Classes
//////////////////////////////////////////////////////////////////////////

struct FMVNLiveLinkSource
{
	FProviderPollResultPtr Result;
	double Time = 0.0;

	FMVNLiveLinkSource(FProviderPollResultPtr InResult)
		: Result(MoveTemp(InResult))
		, Time(FApp::GetCurrentTime())
	{
	}
};

namespace
{
	bool operator==(const FProviderPollResult& LHS, const FProviderPollResult& RHS)
	{
		return LHS.Name == RHS.Name && LHS.MachineName == RHS.MachineName;
	}

	const FName TypeColumnName(TEXT("Type"));
	const FName MachineColumnName(TEXT("Machine"));
	const double UIRefreshRate = 0.5;
}

class SMVNLiveLinkProviderRow : public SMultiColumnTableRow<FProviderPollResultPtr>
{
public:
	SLATE_BEGIN_ARGS(SMVNLiveLinkProviderRow) {}
		SLATE_ARGUMENT(FProviderPollResultPtr, ResultPtr)
	SLATE_END_ARGS()

	void Construct(const FArguments& Args, const TSharedRef<STableViewBase>& TableView)
	{
		ResultPtr = Args._ResultPtr;

		SMultiColumnTableRow<FProviderPollResultPtr>::Construct(FSuperRowType::FArguments()
			.Padding(1.0f),
			TableView);

		if (!ResultPtr->bIsValidProvider)
		{
			SetToolTipText(LOCTEXT("InvalidProvider", "Invalid provider, please make sure you are using the latest Unreal Engine Version"));
			SetEnabled(false);
		}
	}

	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& ColumnName) override
	{
		if (ColumnName == TypeColumnName)
		{
			return SNew(STextBlock)
				.Text(FText::FromString(ResultPtr->Name));
		}
		else if (ColumnName == MachineColumnName)
		{
			return SNew(STextBlock)
				.Text(FText::FromString(ResultPtr->MachineName));
		}

		return SNullWidget::NullWidget;
	}

private:
	FProviderPollResultPtr ResultPtr;
};

//////////////////////////////////////////////////////////////////////////
//// Slate Class
//////////////////////////////////////////////////////////////////////////

SLiveLinkMvnSourceEditor::~SLiveLinkMvnSourceEditor()
{
	if (FLiveLinkMvnPluginModule* ModulePtr = FModuleManager::GetModulePtr<FLiveLinkMvnPluginModule>("LiveLinkMvnPlugin"))
	{
		ModulePtr->GetPresenceDetector().RemovePresenceRequest();
	}
}

void SLiveLinkMvnSourceEditor::Construct(const FArguments& Args)
{
	OnSourceSelected = Args._OnSourceSelected;
	LastTickTime = 0.0;
	FLiveLinkMvnPluginModule::Get().GetPresenceDetector().AddPresenceRequest();

	ChildSlot
		[
			SNew(SVerticalBox)
				+ SVerticalBox::Slot()
				.FillHeight(1)
				[
					SNew(SBox)
						.HeightOverride(200)
						.WidthOverride(200)
						[
							SAssignNew(ListView, SListView<TSharedPtr<FMVNLiveLinkSource>>)
								.OnSelectionChanged(this, &SLiveLinkMvnSourceEditor::OnSelectionChanged)
								.OnGenerateRow(this, &SLiveLinkMvnSourceEditor::CreateListView)
								.SelectionMode(ESelectionMode::SingleToggle)
								.ListItemsSource(&Sources)
								.HeaderRow
								(
									SNew(SHeaderRow)
									+ SHeaderRow::Column(TypeColumnName)
									.FillWidth(50.0f)
									.DefaultLabel(LOCTEXT("MVNLiveLinkTypeName", "Type"))
									+ SHeaderRow::Column(MachineColumnName)
									.FillWidth(50.0f)
									.DefaultLabel(LOCTEXT("MVNLiveLinkMachineName", "Machine Name"))
								)
						]
				]
		];
}

void SLiveLinkMvnSourceEditor::OnSelectionChanged(TSharedPtr<struct FMVNLiveLinkSource> Source, ESelectInfo::Type Type)
{
	SelectedResult = Source->Result;
	OnSourceSelected.ExecuteIfBound(SelectedResult);

	if (FLiveLinkMvnPluginModule::Get().GetPresenceDetector().GetMessageEndpoint().IsValid())
	{
		FLiveLinkMvnPluginModule::Get().GetPresenceDetector().GetMessageEndpoint()->Publish<FLiveLinkConnectMessage>(FMessageEndpoint::MakeMessage<FLiveLinkConnectMessage>());
	}
}

TSharedRef<class ITableRow> SLiveLinkMvnSourceEditor::CreateListView(TSharedPtr<struct FMVNLiveLinkSource> Source, const TSharedRef<class STableViewBase>& TableView) const
{
	return SNew(SMVNLiveLinkProviderRow, TableView).ResultPtr(Source->Result);
}

void SLiveLinkMvnSourceEditor::Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime)
{
	if (FApp::GetCurrentTime() - LastTickTime > UIRefreshRate)
	{
		// Get the sources
		Results.Reset();
		FLiveLinkMvnPluginModule::Get().GetPresenceDetector().GetResults(Results);

		// Remove all sources that took too long to send their presence
		Sources.RemoveAllSwap([this](TSharedPtr<FMVNLiveLinkSource> Source)
			{
				return FApp::GetCurrentTime() - Source->Time > 2.0;
			});

		for (const FProviderPollResultPtr& Result : Results)
		{
			TSharedPtr<FMVNLiveLinkSource>* FoundSource = Sources.FindByPredicate([&Result](const TSharedPtr<FMVNLiveLinkSource> Source)
				{
					return Source && Result && *Source->Result == *Result;
				});

			if (FoundSource && *FoundSource)
			{
				(*FoundSource)->Time = FApp::GetCurrentTime();
			}
			else
			{
				Sources.Add(MakeShared<FMVNLiveLinkSource>(Result));
			}
		}

		Sources.StableSort([](const TSharedPtr<FMVNLiveLinkSource>& LHS, const TSharedPtr<FMVNLiveLinkSource>& RHS)
			{
				if (LHS.IsValid() && RHS.IsValid() &&
					LHS->Result.IsValid() && RHS->Result.IsValid())
				{
					const int32 CompareTest = LHS->Result->Name.Compare(RHS->Result->Name);
					return CompareTest == 0 ?
						LHS->Result->MachineName.Compare(RHS->Result->MachineName) <= 0 :
						CompareTest <= 0;
				}
				return true;
			});

		// Refresh the list view with the new sources
		ListView->RequestListRefresh();
		LastTickTime = FApp::GetCurrentTime();
	}
}

#undef LOCTEXT_NAMESPACE
