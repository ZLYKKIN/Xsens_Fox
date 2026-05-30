// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "Customizations/SLiveLinkMvnSubjectRepresentationPicker.h"

#include "Widgets/Views/STableRow.h"

#include "AssetRegistry/AssetData.h"
#include "EditorStyleSet.h"
#include "Features/IModularFeatures.h"
#include "ILiveLinkClient.h"
#include "Misc/FeedbackContext.h"
#include "PropertyCustomizationHelpers.h"
#include "Styling/SlateIconFinder.h"
#include "LiveLinkMvnMetadataService.h"

#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Framework/Application/SlateApplication.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SComboButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STableRow.h"

#include "Runtime/Launch/Resources/Version.h"

#define LOCTEXT_NAMESPACE "SLiveLinkMvnSubjectRepresentationPicker"

namespace MvnSubjectUI
{
	static const FName EnabledColumnName(TEXT("Enabled"));
	static const FName SourceColumnName(TEXT("Source"));
	static const FName NameColumnName(TEXT("Name"));
	static const FName RoleColumnName(TEXT("Role"));
	static const FName SegmentColumnName(TEXT("Segment"));
	static const FName DetachColumnName(TEXT("Detach"));
};

struct FLiveLinkMvnSubjectRepresentationPickerEntry
{
	FLiveLinkMvnSubjectRepresentationPickerEntry() = default;
	FLiveLinkMvnSubjectRepresentationPickerEntry(const SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment& InSourceSubjectRole,
		const FText& InSourceType, bool bInEnabled)
		: SourceSubjectRole(InSourceSubjectRole)
		, SourceType(InSourceType)
		, bEnabled(bInEnabled)
	{}

	SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment SourceSubjectRole;
	FText SourceType;
	bool bEnabled;
};

class SLiveLinkMvnSubjectEntryRow : public SMultiColumnTableRow<FLiveLinkMvnSubjectRepresentationPickerEntryPtr>
{
public:
	DECLARE_DELEGATE_OneParam(FOnValueChanged, const SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment&);
	DECLARE_DELEGATE(FOnDetach);

	SLATE_BEGIN_ARGS(SLiveLinkMvnSubjectEntryRow) {}
		SLATE_ARGUMENT(FLiveLinkMvnSubjectRepresentationPickerEntryPtr, Entry)
		/** Delegate for handling when for when the current value changes. */
		SLATE_EVENT(FOnValueChanged, OnValueChanged)

		/** Delegate for handling detach action. */
		SLATE_EVENT(FOnDetach, OnDetach)

		SLATE_END_ARGS()

	void Construct(const FArguments& Args, const TSharedRef<STableViewBase>& OwnerTableView)
	{
		EntryPtr = Args._Entry;
		OnValueChangedDelegate = Args._OnValueChanged;
		OnDetachDelegate = Args._OnDetach;

		SMultiColumnTableRow<FLiveLinkMvnSubjectRepresentationPickerEntryPtr>::Construct(
			FSuperRowType::FArguments()
			.Padding(0.f),
			OwnerTableView
		);
	}

	/** Overridden from SMultiColumnTableRow.  Generates a widget for this column of the list view. */
	virtual TSharedRef<SWidget> GenerateWidgetForColumn(const FName& ColumnName) override
	{
		if (ColumnName == MvnSubjectUI::EnabledColumnName)
		{
			return SNew(SCheckBox)
				.IsChecked(EntryPtr->bEnabled ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
				.IsEnabled(false);
		}
		else if (ColumnName == MvnSubjectUI::SourceColumnName)
		{
			return
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.VAlign(VAlign_Center)
				[
					SNew(STextBlock)
						.Text(EntryPtr->SourceType)
				];
		}
		else if (ColumnName == MvnSubjectUI::NameColumnName)
		{
			return
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot()
				.VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Text(FText::FromName(EntryPtr->SourceSubjectRole.Subject))
				];
		}
		else if (ColumnName == MvnSubjectUI::RoleColumnName)
		{
			if (EntryPtr->SourceSubjectRole.Role)
			{
				return SNew(SHorizontalBox)
					+ SHorizontalBox::Slot()
					.VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(EntryPtr->SourceSubjectRole.Role->GetDefaultObject<ULiveLinkRole>()->GetDisplayName())
					];
			}
		}
		else if (ColumnName == MvnSubjectUI::SegmentColumnName)
		{
			TSharedPtr<SWidget> ComboButtonContent = GetCurrentValue();

			return 	SNew(SHorizontalBox)
					+ SHorizontalBox::Slot()
					.VAlign(VAlign_Center)
					[
						SNew(SComboButton)
						.ComboButtonStyle(&FCoreStyle::Get().GetWidgetStyle< FComboButtonStyle >("ComboButton"))
						.ButtonStyle((FButtonStyle*)nullptr)
						.VAlign(VAlign_Center)
						.OnGetMenuContent(this, &SLiveLinkMvnSubjectEntryRow::BuildSegmentsList)
						.ButtonContent()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot()
							[
								ComboButtonContent.ToSharedRef()
							]
						]
					];
		}
		else if (ColumnName == MvnSubjectUI::DetachColumnName)
		{
			int32 SegmentIndex = EntryPtr->SourceSubjectRole.SegmentIndex;
			if (SegmentIndex <= 0) 
			{
				return 	SNew(SHorizontalBox);
			}
			else
			{
				return SNew(SHorizontalBox)
					+ SHorizontalBox::Slot()
					.VAlign(VAlign_Center)
					[
						SNew(SButton)
						.ContentPadding(0)
						.ButtonStyle(FAppStyle::Get(), "Button") // Use the tool bar item style for this button
						.OnClicked(this, &SLiveLinkMvnSubjectEntryRow::DetachCliecked)
						[
							SNew(STextBlock)
								.Text(LOCTEXT("Detach", "Detach"))
						]

					];
			}
		}

		return SNullWidget::NullWidget;
	}

	TSharedRef<SWidget> BuildSegmentsList()
	{
		const bool CloseAfterSelection = true;
		const bool CloseSelfOnly = true;
		FMenuBuilder MenuBuilder(CloseAfterSelection, nullptr, nullptr, CloseSelfOnly);

		MenuBuilder.BeginSection("SegmentSection", LOCTEXT("Segments", "Segments"));

		TArray<FText> SegmentNames;
		bool HasMetadata = FLiveLinkMvnMetadataService::getInstance().GetSegmentNames(EntryPtr->SourceSubjectRole.Source,
			EntryPtr->SourceSubjectRole.Subject, SegmentNames);

		if (HasMetadata && SegmentNames.Num() > 0)
		{
			const int32 SegmentsCount = SegmentNames.Num();
			for (int32 SegmentIdx = 0; SegmentIdx < SegmentsCount; ++SegmentIdx)
			{
				MenuBuilder.AddMenuEntry(
					SegmentNames[SegmentIdx],
					LOCTEXT("SegmentTooltip", "SegmentTooltip"),
					FSlateIcon(),
					FUIAction(
						FExecuteAction::CreateSP(this, &SLiveLinkMvnSubjectEntryRow::SegmentSelected, SegmentIdx + 1)
					),
					NAME_None,
					EUserInterfaceActionType::Button);
			}
		}
		else
		{
			const int32 SegmentsCount = FLiveLinkMvnMetadataService::getInstance().GetSegmentsCount(EntryPtr->SourceSubjectRole.Source, EntryPtr->SourceSubjectRole.Subject);
			for (int32 SegmentIdx = 0; SegmentIdx < SegmentsCount; ++SegmentIdx)
			{
				FString SegmentLable = FString::Printf(L"Segment #%d", SegmentIdx);
				MenuBuilder.AddMenuEntry(
					FText::FromString(SegmentLable),
					LOCTEXT("SegmentTooltip", "SegmentTooltip"),
					FSlateIcon(),
					FUIAction(
						FExecuteAction::CreateSP(this, &SLiveLinkMvnSubjectEntryRow::SegmentSelected, SegmentIdx + 1)
					),
					NAME_None,
					EUserInterfaceActionType::Button);
			}
		}

		MenuBuilder.EndSection();

		return MenuBuilder.MakeWidget();
	}

	TSharedPtr<SWidget> GetCurrentValue()
	{
		int32 SegmentIndex = EntryPtr->SourceSubjectRole.SegmentIndex;
		if (SegmentIndex > 0) {
			TArray<FText> SegmentNames;
			bool HasMetadata = FLiveLinkMvnMetadataService::getInstance().GetSegmentNames(EntryPtr->SourceSubjectRole.Source,
				EntryPtr->SourceSubjectRole.Subject, SegmentNames);

			FText SegmentText;
            bool bValidIndex = true;
            if ( SegmentIndex < 0 || SegmentIndex >= SegmentNames.Num() )
            {
                bValidIndex = false;
            }
            if ( HasMetadata && bValidIndex )
			{
				SegmentText = SegmentNames[SegmentIndex - 1];
			}
			else
			{
				FString PlaceHolder = FString("Segment #") + FString::FromInt(SegmentIndex);
				SegmentText = FText::AsCultureInvariant(PlaceHolder);
			}
			return SNew(STextBlock)
					.Text(SegmentText);
		}
		else {
			return SNew(STextBlock)
					.Text(LOCTEXT("NoSegment", "<None>"));
		}
	}

	void SegmentSelected(int32 SegmentIndex)
	{
		SelectedSegmentIdx = SegmentIndex;
		EntryPtr->SourceSubjectRole.SegmentIndex = SelectedSegmentIdx;
		OnValueChangedDelegate.ExecuteIfBound(EntryPtr->SourceSubjectRole);
	}

	FReply DetachCliecked()
	{
		OnDetachDelegate.ExecuteIfBound();

//		TSharedRef<SWindow> ParentContextMenuWindow = FSlateApplication::Get().FindWidgetWindow(AsShared()).ToSharedRef();
//		FSlateApplication::Get().RequestDestroyWindow(ParentContextMenuWindow);
		FSlateApplication::Get().DismissAllMenus();

		return FReply::Handled();
	}

private:

	FLiveLinkMvnSubjectRepresentationPickerEntryPtr EntryPtr;
	FOnValueChanged OnValueChangedDelegate;
	FOnDetach OnDetachDelegate;
	int32 SelectedSegmentIdx = 0;
};


void SLiveLinkMvnSubjectRepresentationPicker::Construct(const FArguments& InArgs)
{
	ValueAttribute = InArgs._Value;
	OnValueChangedDelegate = InArgs._OnValueChanged;
	OnDetachDelegate = InArgs._OnDetach;
	HasMultipleValuesAttribute = InArgs._HasMultipleValues;

	SubjectRepData.Reset();

	ILiveLinkClient& LiveLinkClient = IModularFeatures::Get().GetModularFeature<ILiveLinkClient>(ILiveLinkClient::ModularFeatureName);
	CachedSourceType = LiveLinkClient.GetSourceType(ValueAttribute.Get().Source);

	TSharedPtr<SWidget> ComboButtonContent;
	
	ComboButtonContent = SNew(SBorder)
		.BorderImage(FAppStyle::GetBrush("NoBorder"))
		.Padding(FMargin(0, 0, 5, 0))
		[
			SNew(SEditableTextBox)
			.Text(this, &SLiveLinkMvnSubjectRepresentationPicker::GetSubjectNameValueText)
			.OnTextCommitted(this, &SLiveLinkMvnSubjectRepresentationPicker::OnComboTextCommitted)
			.SelectAllTextWhenFocused(true)
			.SelectAllTextOnCommit(true)
			.ClearKeyboardFocusOnCommit(false)
			.Font(InArgs._Font)
		];

	ChildSlot
		[
			SAssignNew(PickerComboButton, SComboButton)
			.ComboButtonStyle(InArgs._ComboButtonStyle)
			.ButtonStyle(InArgs._ButtonStyle)
//			.ForegroundColor(InArgs._ForegroundColor) - this color makes button invisible in UE5
			.ContentPadding(InArgs._ContentPadding)
			.VAlign(VAlign_Fill)
			.OnGetMenuContent(this, &SLiveLinkMvnSubjectRepresentationPicker::BuildMenu)
			.ButtonContent()
			[
				ComboButtonContent.ToSharedRef()
			]
		];
}

FText SLiveLinkMvnSubjectRepresentationPicker::GetSourceNameValueText() const
{
	const bool bHasMultipleValues = HasMultipleValuesAttribute.Get();
	if (bHasMultipleValues)
	{
		return LOCTEXT("MultipleValuesText", "<multiple values>");
	}

	return CachedSourceType;
}

FText SLiveLinkMvnSubjectRepresentationPicker::GetSubjectNameValueText() const
{
	const bool bHasMultipleValues = HasMultipleValuesAttribute.Get();
	if (bHasMultipleValues)
	{
		return LOCTEXT("MultipleValuesText", "<multiple values>");
	}

	return FText::FromName(ValueAttribute.Get().Subject);
}

TSharedRef<SWidget> SLiveLinkMvnSubjectRepresentationPicker::BuildMenu()
{
	SubjectRepData.Reset();
	BuildSubjectRepDataList();

	TSharedRef<SHeaderRow> HeaderRow = SNew(SHeaderRow)
		+ SHeaderRow::Column(MvnSubjectUI::EnabledColumnName)
		.ManualWidth(20.f)
		.DefaultLabel(LOCTEXT("EnabledColumnHeaderName", ""));

	HeaderRow->AddColumn(SHeaderRow::Column(MvnSubjectUI::NameColumnName)
		.FillWidth(60.f)
		.DefaultLabel(LOCTEXT("SubjectColumnHeaderName", "Subject")));
	
	HeaderRow->AddColumn(SHeaderRow::Column(MvnSubjectUI::RoleColumnName)
		.FillWidth(50.f)
		.DefaultLabel(LOCTEXT("RoleColumnHeaderName", "Role")));
	
	HeaderRow->AddColumn(SHeaderRow::Column(MvnSubjectUI::SegmentColumnName)
		.FillWidth(50.0f)
		.DefaultLabel(LOCTEXT("SegmentColumnHeaderName", "Segment")));
	HeaderRow->AddColumn(SHeaderRow::Column(MvnSubjectUI::DetachColumnName)
		.FillWidth(40.0f)
		.DefaultLabel(LOCTEXT("Detach", "")));

	return SNew(SBox)
		.Padding(0)
		.WidthOverride(375.f)
		.HeightOverride(300.f)
		[
			SNew(SBorder)
			.ForegroundColor(FCoreStyle::Get().GetSlateColor("DefaultForeground"))
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot()
				.AutoHeight()
				+ SVerticalBox::Slot()
				[
					SNew(SBorder)
					.Padding(FMargin(4.0f, 4.0f))
					[
						SAssignNew(SubjectListView, SListView<FLiveLinkMvnSubjectRepresentationPickerEntryPtr>)
						.ListItemsSource(&SubjectRepData)
						.SelectionMode(ESelectionMode::None)
						.OnGenerateRow(this, &SLiveLinkMvnSubjectRepresentationPicker::MakeSubjectRepListViewWidget)
						.HeaderRow(HeaderRow)
					]
				]
			]
		];
}

TSharedRef<ITableRow> SLiveLinkMvnSubjectRepresentationPicker::MakeSubjectRepListViewWidget(FLiveLinkMvnSubjectRepresentationPickerEntryPtr Entry, const TSharedRef<STableViewBase>& OwnerTable)
{
	return SNew(SLiveLinkMvnSubjectEntryRow, OwnerTable)
		.Entry(Entry)
		.OnValueChanged(SLiveLinkMvnSubjectEntryRow::FOnValueChanged::CreateSP(this, &SLiveLinkMvnSubjectRepresentationPicker::SetValue))
		.OnDetach(SLiveLinkMvnSubjectEntryRow::FOnDetach::CreateSP(this, &SLiveLinkMvnSubjectRepresentationPicker::Detach));
}

void SLiveLinkMvnSubjectRepresentationPicker::OnSubjectRepListSelectionChanged(FLiveLinkMvnSubjectRepresentationPickerEntryPtr Entry, 
	int32 SegmentIdx, ESelectInfo::Type SelectionType)
{
	if (Entry.IsValid())
	{
		SetValue(Entry->SourceSubjectRole);
		CachedSourceType = Entry->SourceType;
	}
	else
	{
		SetValue(FLiveLinkMvnSourceSubjectRoleSegment());
		CachedSourceType = FText::GetEmpty();
	}
}

void SLiveLinkMvnSubjectRepresentationPicker::OnComboTextCommitted(const FText& NewText, ETextCommit::Type InTextCommit)
{
	FLiveLinkMvnSourceSubjectRoleSegment SourceSubjectRole;
	SourceSubjectRole.Subject.Name = *NewText.ToString();
	SetValue(SourceSubjectRole);

	CachedSourceType = FText::GetEmpty();
}

void SLiveLinkMvnSubjectRepresentationPicker::SetValue(const SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment& InValue)
{
	if (OnValueChangedDelegate.IsBound())
	{
		OnValueChangedDelegate.ExecuteIfBound(InValue);
	}

	// Update value indicated in UI
	ValueAttribute = InValue;

	TSharedPtr<SComboButton> PickerComboButtonPin = PickerComboButton.Pin();
	if (PickerComboButtonPin.IsValid())
	{
		PickerComboButtonPin->SetIsOpen(false);
	}
}

void SLiveLinkMvnSubjectRepresentationPicker::Detach() const
{
	if (OnDetachDelegate.IsBound())
	{
		OnDetachDelegate.ExecuteIfBound();
	}
}

void SLiveLinkMvnSubjectRepresentationPicker::BuildSubjectRepDataList()
{
	SubjectRepData.Reset();

	if (IModularFeatures::Get().IsModularFeatureAvailable(ILiveLinkClient::ModularFeatureName))
	{
		ILiveLinkClient& LiveLinkClient = IModularFeatures::Get().GetModularFeature<ILiveLinkClient>(ILiveLinkClient::ModularFeatureName);
		TArray<FLiveLinkSubjectKey> SubjectKeys = LiveLinkClient.GetSubjects(true, true);

		TMap<FLiveLinkSubjectName, int32> UniqueSubjectName;
		TMap<FLiveLinkSubjectName, FGuid> Sources;
		for (const FLiveLinkSubjectKey& Key : SubjectKeys)
		{
			if (!Key.SubjectName.IsNone())
			{
				int32* FoundValue = UniqueSubjectName.Find(Key.SubjectName);
				if (FoundValue)
				{
					++(*FoundValue);
				}
				else
				{
					UniqueSubjectName.Add(Key.SubjectName, 1);
					Sources.Add(Key.SubjectName, Key.Source);
				}
			}
		}

		for (const auto& Item : UniqueSubjectName)
		{
			bool bEnabled = LiveLinkClient.IsSubjectEnabled(Item.Key);

			SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment SrcSubRole;
			SrcSubRole.Source = Sources[Item.Key];
			SrcSubRole.Subject = Item.Key;
#if ENGINE_MAJOR_VERSION >= 5
			SrcSubRole.Role = LiveLinkClient.GetSubjectRole_AnyThread(Item.Key);
#else
			SrcSubRole.Role = LiveLinkClient.GetSubjectRole(Item.Key);
#endif

			if (SrcSubRole.Role == nullptr && Item.Value != 1)
			{
				bool bIsFirstEntry = true;
				for (const FLiveLinkSubjectKey& Key : SubjectKeys)
				{
					if (Key.SubjectName == Item.Key)
					{
#if ENGINE_MAJOR_VERSION >= 5
						TSubclassOf<ULiveLinkRole> NewRole = LiveLinkClient.GetSubjectRole_AnyThread(Key);
#else
						TSubclassOf<ULiveLinkRole> NewRole = LiveLinkClient.GetSubjectRole(Key);
#endif
						if (!bIsFirstEntry && SrcSubRole.Role != NewRole)
						{
							SrcSubRole.Role = nullptr;
							break;
						}

						bIsFirstEntry = false;
						SrcSubRole.Role = NewRole;
					}
				}
			}

			if (ValueAttribute.Get().Subject == SrcSubRole.Subject)
			{
				SrcSubRole.SegmentIndex = ValueAttribute.Get().SegmentIndex;
			}

			SubjectRepData.Add(MakeShared<FLiveLinkMvnSubjectRepresentationPickerEntry>(SrcSubRole, FText::GetEmpty(), bEnabled));
		}

		struct FEntryLess
		{
			FORCEINLINE bool operator()(const FLiveLinkMvnSubjectRepresentationPickerEntryPtr& A, const FLiveLinkMvnSubjectRepresentationPickerEntryPtr& B) const
			{
				return A->SourceSubjectRole.Subject.Name.Compare(B->SourceSubjectRole.Subject.Name) < 0;
			}
		};
		SubjectRepData.Sort(FEntryLess());
	}

	TSharedPtr<SListView<FLiveLinkMvnSubjectRepresentationPickerEntryPtr>> SubjectListViewPin = SubjectListView.Pin();
	if (SubjectListViewPin.IsValid())
	{
		SubjectListViewPin->RebuildList();
	}
}

#undef LOCTEXT_NAMESPACE