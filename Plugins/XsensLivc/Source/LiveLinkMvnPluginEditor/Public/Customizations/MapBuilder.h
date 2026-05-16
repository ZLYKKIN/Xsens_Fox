// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "PropertyHandle.h"
#include "SResetToDefaultMenu.h"
#include "Components/HorizontalBox.h"
#include "IDetailCustomNodeBuilder.h"
#include "DetailWidgetRow.h"
#include "IDetailPropertyRow.h"
#include "IDetailChildrenBuilder.h"
#include "Templates/SharedPointer.h"
#include "Widgets/Input/SComboBox.h"
#include "DetailLayoutBuilder.h"
#include "EditorFontGlyphs.h"
#include "EditorStyleSet.h"

#define LOCTEXT_NAMESPACE "DetailMapBuilder"

// This Macro is used to get the display name out of UEnum
#define GETENUM_DISPLAYNAME(EnumType, EnumValue) \
( \
    StaticEnum<EnumType>() \
        ? StaticEnum<EnumType>()->GetDisplayNameTextByValue((int64)EnumValue) \
        : FText::FromString(TEXT("Invalid - are you sure enum uses UENUM()?")) \
)

class IDetailMapBuilderCallback
{
public:
	virtual void GenerateChildWidget(IDetailChildrenBuilder& ChildrenBuilder, TSharedRef<IPropertyHandle> ItemProperty) = 0;
};

class FDetailMapBuilder : public IDetailCustomNodeBuilder
{
public:

	DECLARE_DELEGATE_RetVal(bool, FOnGetBool);
	DECLARE_DELEGATE_RetVal(FText, FOnGetText);

	FDetailMapBuilder(TSharedRef<IPropertyHandle> InBaseProperty, IDetailMapBuilderCallback* ChildGeneratorFunc, bool InGenerateHeader = true, bool InDisplayResetToDefault = true, bool InDisplayElementNum = true)
		: MapProperty(InBaseProperty->AsMap())
		, BaseProperty(InBaseProperty)
		, MapBuilderCallbackClass(ChildGeneratorFunc)
		, bGenerateHeader(InGenerateHeader)
		, bDisplayResetToDefault(InDisplayResetToDefault)
		, bDisplayElementNum(InDisplayElementNum)
	{
		check(MapProperty.IsValid());

		// Delegate for when the number of children in the array changes
		FSimpleDelegate OnNumChildrenChanged = FSimpleDelegate::CreateRaw(this, &FDetailMapBuilder::OnNumChildrenChanged);
		MapProperty->SetOnNumElementsChanged(OnNumChildrenChanged);

		BaseProperty->MarkHiddenByCustomization();
	}

	~FDetailMapBuilder()
	{
		FSimpleDelegate Empty;
		MapProperty->SetOnNumElementsChanged(Empty);
	}

	void SetDisplayName(const FText& InDisplayName)
	{
		DisplayName = InDisplayName;
	}

	void SetInfoMessage(FOnGetText InMessage, FOnGetBool bInIsWarning)
	{
		OnGetInfoMessageText = InMessage;
		OnGetInfoMessageWarning = bInIsWarning;
	}

	virtual bool RequiresTick() const override { return false; }

	virtual void Tick(float DeltaTime) override {}

	virtual FName GetName() const override
	{
		return BaseProperty->GetProperty()->GetFName();
	}

	virtual bool InitiallyCollapsed() const override { return false; }

	virtual void GenerateHeaderRowContent(FDetailWidgetRow& NodeRow) override
	{
		if (bGenerateHeader)
		{
			TSharedPtr<SHorizontalBox> ContentHorizontalBox;
			SAssignNew(ContentHorizontalBox, SHorizontalBox);
			if (bDisplayElementNum)
			{
				ContentHorizontalBox->AddSlot()
					[
						BaseProperty->CreatePropertyValueWidget()
					];
			}

			NodeRow
			.FilterString(!DisplayName.IsEmpty() ? DisplayName : BaseProperty->GetPropertyDisplayName())
			.NameContent()
			[
				BaseProperty->CreatePropertyNameWidget(DisplayName, FText::GetEmpty())
			]
			.ValueContent()
			[
				ContentHorizontalBox.ToSharedRef()
			];

			if (bDisplayResetToDefault)
			{
				TSharedPtr<SResetToDefaultMenu> ResetToDefaultMenu;
				ContentHorizontalBox->AddSlot()
					.AutoWidth()
					.Padding(FMargin(2.0f, 0.0f, 0.0f, 0.0f))
					[
						SAssignNew(ResetToDefaultMenu, SResetToDefaultMenu)
					];
				ResetToDefaultMenu->AddProperty(BaseProperty);
			}
		}
	}

	virtual void GenerateChildContent(IDetailChildrenBuilder& ChildrenBuilder) override
	{
		// Add a header message
		{
			TAttribute<EVisibility> InfoVisibility = TAttribute<EVisibility>::Create(TAttribute<EVisibility>::FGetter::CreateLambda(
				[this]
				{
					bool bVisible = OnGetInfoMessageText.IsBound() ? !OnGetInfoMessageText.Execute().IsEmpty() : false;
					return bVisible
						? EVisibility::Visible
						: EVisibility::Hidden;
				}));

			ChildrenBuilder.AddCustomRow(FText())
				.Visibility(InfoVisibility)
				.WholeRowContent()
				[
					SNew(SHorizontalBox)

					+ SHorizontalBox::Slot()
					.Padding(FMargin(5.f, 0.f, 5.f, 0.f))
					.VAlign(VAlign_Center)
					.AutoWidth()
					[
						SNew(STextBlock)
						.TextStyle(FAppStyle::Get(), "Log.Warning")
						.Font(FAppStyle::GetFontStyle("FontAwesome.10"))
						.Text(TAttribute<FText>::Create([this]
							{
								bool bWarning = OnGetInfoMessageWarning.IsBound() ? OnGetInfoMessageWarning.Execute() : false;
								return bWarning ? FEditorFontGlyphs::Exclamation_Triangle : FEditorFontGlyphs::Info_Circle;
							}))
					]

					+ SHorizontalBox::Slot()
					.Padding(FMargin(0.f, 0.f, 5.f, 0.f))
					.VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.TextStyle(FAppStyle::Get(), "Log.Warning")
						.Text(TAttribute<FText>::Create([this]
							{
								return OnGetInfoMessageText.IsBound() ? OnGetInfoMessageText.Execute() : FText();
							}))
					]
				];
		}

		uint32 NumChildren = 0;
		MapProperty->GetNumElements(NumChildren);

		for (uint32 ChildIndex = 0; ChildIndex < NumChildren; ++ChildIndex)
		{
			TSharedRef<IPropertyHandle> ItemProperty = BaseProperty->GetChildHandle(ChildIndex).ToSharedRef();
			if (ItemProperty->IsValidHandle())
			{
				MapBuilderCallbackClass->GenerateChildWidget(ChildrenBuilder, ItemProperty);
			}
		}
	}

	virtual void RefreshChildren()
	{
		OnRebuildChildren.ExecuteIfBound();
	}

	virtual TSharedPtr<IPropertyHandle> GetPropertyHandle() const
	{
		return BaseProperty;
	}

protected:

	virtual void SetOnRebuildChildren(FSimpleDelegate InOnRebuildChildren) override { OnRebuildChildren = InOnRebuildChildren; }

	void OnNumChildrenChanged()
	{
		OnRebuildChildren.ExecuteIfBound();
	}

private:

	FText DisplayName;
	TSharedPtr<IPropertyHandleMap> MapProperty;
	TSharedRef<IPropertyHandle> BaseProperty;
	FSimpleDelegate OnRebuildChildren;
	IDetailMapBuilderCallback* MapBuilderCallbackClass;
	bool bGenerateHeader;
	bool bDisplayResetToDefault;
	bool bDisplayElementNum;
	FOnGetBool OnGetInfoMessageWarning;
	FOnGetText OnGetInfoMessageText;
};

#undef LOCTEXT_NAMESPACE
