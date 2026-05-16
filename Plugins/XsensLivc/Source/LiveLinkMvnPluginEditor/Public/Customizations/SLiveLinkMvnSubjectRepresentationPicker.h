// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SCompoundWidget.h"

#include "Input/Reply.h"
#include "LiveLinkRole.h"
#include "Styling/CoreStyle.h"
#include "Styling/SlateTypes.h"
#include "Styling/SlateWidgetStyleAsset.h"
#include "Types/SlateEnums.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Input/SComboButton.h"


struct FLiveLinkMvnSubjectRepresentationPickerEntry;
typedef TSharedPtr<FLiveLinkMvnSubjectRepresentationPickerEntry> FLiveLinkMvnSubjectRepresentationPickerEntryPtr;

class SLiveLinkMvnSubjectRepresentationPicker : public SCompoundWidget
{
public:
	struct FLiveLinkMvnSourceSubjectRoleSegment
	{
		FLiveLinkMvnSourceSubjectRoleSegment() = default;
		FLiveLinkMvnSourceSubjectRoleSegment(FLiveLinkSubjectRepresentation InSubjectRepresentation, int32 SegmentIdx)
			: Subject(InSubjectRepresentation.Subject), Role(InSubjectRepresentation.Role), SegmentIndex(SegmentIdx)
		{}

		FLiveLinkSubjectRepresentation ToSubjectRepresentation() const { return FLiveLinkSubjectRepresentation(Subject, Role); }
		FLiveLinkSubjectKey ToSubjectKey() const { return FLiveLinkSubjectKey(Source, Subject); }

		FGuid Source;
		FLiveLinkSubjectName Subject;
		TSubclassOf<ULiveLinkRole> Role;
		int32 SegmentIndex = -1;
	};

	DECLARE_DELEGATE_OneParam(FOnValueChanged, FLiveLinkMvnSourceSubjectRoleSegment);
	DECLARE_DELEGATE(FOnDetach);

	SLATE_BEGIN_ARGS(SLiveLinkMvnSubjectRepresentationPicker)
		: _ComboButtonStyle(&FCoreStyle::Get().GetWidgetStyle< FComboButtonStyle >("ComboButton"))
		, _ButtonStyle(nullptr)
		, _ForegroundColor(FCoreStyle::Get().GetSlateColor("InvertedForeground"))
		, _ContentPadding(FMargin(2.f, 0.f))
		, _HasMultipleValues(false)
		, _Font()
	{}

	/** The visual style of the combo button */
	SLATE_STYLE_ARGUMENT(FComboButtonStyle, ComboButtonStyle)

		/** The visual style of the button (overrides ComboButtonStyle) */
		SLATE_STYLE_ARGUMENT(FButtonStyle, ButtonStyle)

		/** Foreground color for the picker */
		SLATE_ATTRIBUTE(FSlateColor, ForegroundColor)

		/** Content padding for the picker */
		SLATE_ATTRIBUTE(FMargin, ContentPadding)

		/** Attribute used to retrieve the current value. */
		SLATE_ATTRIBUTE(FLiveLinkMvnSourceSubjectRoleSegment, Value)

		/** Delegate for handling when for when the current value changes. */
		SLATE_EVENT(FOnValueChanged, OnValueChanged)

		/** Delegate for handling detach action. */
		SLATE_EVENT(FOnDetach, OnDetach)

		/** Attribute used to retrieve whether the picker has multiple values. */
		SLATE_ATTRIBUTE(bool, HasMultipleValues)

		/** Sets the font used to draw the text on the button */
		SLATE_ATTRIBUTE(FSlateFontInfo, Font)

		SLATE_END_ARGS()

		/**
		 * Slate widget construction method
		 */
		void Construct(const FArguments& InArgs);


private:
	FText GetSourceNameValueText() const;
	FText GetSubjectNameValueText() const;

	TSharedRef<SWidget> BuildMenu();

	TSharedRef<ITableRow> MakeSubjectRepListViewWidget(FLiveLinkMvnSubjectRepresentationPickerEntryPtr Entry, const TSharedRef<STableViewBase>& OwnerTable);
	void OnSubjectRepListSelectionChanged(FLiveLinkMvnSubjectRepresentationPickerEntryPtr Entry, int32 SegmentIdx, ESelectInfo::Type SelectionType);

	void OnComboTextCommitted(const FText& NewText, ETextCommit::Type InTextCommit);
	void SetValue(const FLiveLinkMvnSourceSubjectRoleSegment& InValue);
	void Detach() const;

	void BuildSubjectRepDataList();

private:
	TWeakPtr<SComboButton> PickerComboButton;
	TWeakPtr<SListView<FLiveLinkMvnSubjectRepresentationPickerEntryPtr>> SubjectListView;
	TArray<FLiveLinkMvnSubjectRepresentationPickerEntryPtr> SubjectRepData;

	FText CachedSourceType;
	TAttribute<FLiveLinkMvnSourceSubjectRoleSegment> ValueAttribute;
	FOnValueChanged OnValueChangedDelegate;
	FOnDetach OnDetachDelegate;
	TAttribute<bool> HasMultipleValuesAttribute;
};