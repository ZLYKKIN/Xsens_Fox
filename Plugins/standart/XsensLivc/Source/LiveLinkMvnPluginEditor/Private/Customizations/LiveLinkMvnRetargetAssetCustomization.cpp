// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "Customizations/LiveLinkMvnRetargetAssetCustomization.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "IDetailChildrenBuilder.h"
#include "IDetailPropertyRow.h"

#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SComboBox.h"
#include "PropertyCustomizationHelpers.h"
#include "PropertyHandle.h"
#include "Customizations/MapBuilder.h"

#include "LiveLinkMvnRetargetAsset.h"
#include "Animation/Skeleton.h"


#define LOCTEXT_NAMESPACE "LiveLinkMvnRetargetAssetCustomization"

static FComboBoxStyle* DefaultComboBoxStyle = nullptr;


TSharedRef<IDetailCustomization> LiveLinkMvnRetargetAssetCustomization::MakeInstance()
{
	return MakeShareable(new LiveLinkMvnRetargetAssetCustomization);
}

void LiveLinkMvnRetargetAssetCustomization::CustomizeDetails(IDetailLayoutBuilder & DetailBuilder)
{
	// Create a category so this is displayed early in the properties
	DetailBuilder.EditCategory("Default");
	DetailBuilder.EditCategory("Reference Pose");
	IDetailCategoryBuilder& LiveLinkCategory = DetailBuilder.EditCategory("Live Link Mvn Remap");
	
	TSharedPtr<IPropertyHandle> RecipesProperty = DetailBuilder.GetProperty((GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRetargetAsset, m_remapping_rows)));
	TSharedRef<FDetailMapBuilder> MapPropertyBuilder = MakeShareable(new FDetailMapBuilder(RecipesProperty.ToSharedRef(), this, true, false, false));
	
	// Re-add this property so it is displayed above the remapping table
	LiveLinkCategory.AddCustomBuilder(MapPropertyBuilder, false);

	// Bind info message handlers
	FDetailMapBuilder::FOnGetBool OnIsWarning;
	OnIsWarning.BindLambda([this]
		{
			// Show warning when skeleton is not set, or when convention is not custom
			if (Skeleton == nullptr)
			{
				return true;
			}
			if (RetargetAsset != nullptr)
			{
				if (RetargetAsset->m_remapping_convention != EXsensRetargetNamingConvention::Manual)
				{
					return false;
				}
				if (RetargetAsset->m_skeleton != Skeleton)
				{
					return true;
				}
			}
			return false;
		});
	FDetailMapBuilder::FOnGetText OnGetText;
	OnGetText.BindLambda([this]
		{
			if (Skeleton == nullptr)
			{
				return LOCTEXT("NoSkeleton", "Skeleton is not set, remapping bone list is not available");
			}
			if (RetargetAsset != nullptr)
			{
				if (RetargetAsset->m_remapping_convention != EXsensRetargetNamingConvention::Manual)
				{
					return LOCTEXT("NoEditing", "The mapping is auto-generated. Any edits to the list will switch Naming Convention to Manual mode");
				}
				if (RetargetAsset->m_skeleton != Skeleton)
				{
					return LOCTEXT("SkeletonChanged", "Skeleton has been changed. You should reopen the window to get an updated bone list");
				}
			}
			return FText();
		});
	MapPropertyBuilder->SetInfoMessage(OnGetText, OnIsWarning);

	// Get the edited ULiveLinkMvnRetargetAsset
	RetargetAsset = nullptr;
	Skeleton = nullptr;

	TArray<TWeakObjectPtr<UObject>> WeakObjectsBeingCustomized;
	DetailBuilder.GetObjectsBeingCustomized(WeakObjectsBeingCustomized);
	if (WeakObjectsBeingCustomized.Num() == 1)
	{
		UObject* ObjectBeingCustomized = WeakObjectsBeingCustomized[0].Get();
		if (ObjectBeingCustomized != nullptr)
		{
			RetargetAsset = Cast<ULiveLinkMvnRetargetAsset>(ObjectBeingCustomized);
		}
	}

	// Fill bone name list
	if (RetargetAsset != nullptr)
	{
		Skeleton = RetargetAsset->m_skeleton;
	}
	FetchSkeletonBoneNames(Skeleton);

	// Generate default combo box style
	if (DefaultComboBoxStyle == nullptr)
	{
		DefaultComboBoxStyle = new FComboBoxStyle(FCoreStyle::Get().GetWidgetStyle<FComboBoxStyle>("ComboBox"));
	}
}

void LiveLinkMvnRetargetAssetCustomization::GenerateChildWidget(IDetailChildrenBuilder& ChildrenBuilder, TSharedRef<IPropertyHandle> ItemProperty)
{
	TSharedPtr<IPropertyHandle> ChildHandle = ItemProperty->GetKeyHandle();
	uint8 xsensRemapId;
	ItemProperty->GetKeyHandle()->GetValue(xsensRemapId);

	TSharedPtr<IPropertyHandle> RemapIdHandle = ItemProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FRemappingRowHandle, RemapId));

	// Generate our UI properties.
	ChildrenBuilder.AddCustomRow(LOCTEXT("Remaping", "Remapping Row"))
	.NameContent()
	[
		SNew(STextBlock).ToolTipText(LOCTEXT("RemapFrom", "Xsens bone name to remap from.")).Text(GETENUM_DISPLAYNAME(EXsensMapping, static_cast<EXsensMapping>(xsensRemapId)))
	]
	.ValueContent().MinDesiredWidth(300.0f)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot()
		[
			SNew(SComboBox<TSharedPtr<FString>>)
			.ComboBoxStyle(DefaultComboBoxStyle)
			.ToolTipText(LOCTEXT("RemapTo", "Choose the bone to remap to."))
			.OptionsSource(&BoneNames)
			.OnGenerateWidget_Lambda ( [](TSharedPtr<FString> InItem) {
				return SNew(STextBlock).Margin(FMargin(4, 2))
				.Text(FText::FromString(*InItem));
			})
			.OnSelectionChanged_Lambda([=](TSharedPtr<FString> Selection, ESelectInfo::Type){
				if (RemapIdHandle->IsValidHandle()) {
					RemapIdHandle->SetValue(*Selection);
				}
			})
			.ContentPadding(FMargin(4,2))
			[
				SNew(STextBlock)
				.Font(IDetailLayoutBuilder::GetDetailFont())
				.Text_Lambda( [=]() -> FText
				{
					if (RemapIdHandle->IsValidHandle())
					{
						FName val;
						RemapIdHandle->GetValue(val);
						return FText::FromName(val);
					}

					return FText::GetEmpty();
				} )
			]
		]
	];
}

// Reference: ULiveLinkMvnRetargetAsset::FetchSkeletonBoneNames()
void LiveLinkMvnRetargetAssetCustomization::FetchSkeletonBoneNames(USkeleton* InSkeleton)
{
	if (InSkeleton != nullptr)
	{
		Skeleton = InSkeleton;
		const FReferenceSkeleton& RefSkeleton = Skeleton->GetReferenceSkeleton();

		BoneNames.Empty(RefSkeleton.GetNum() + 1);
		// Explicitly add "None"
		BoneNames.Add(MakeShareable(new FString(TEXT("None"))));

		for(int32 BoneIndex = 0; BoneIndex < RefSkeleton.GetNum(); BoneIndex++)
		{
			BoneNames.Add(MakeShareable(new FString(RefSkeleton.GetBoneName(BoneIndex).ToString())));
		}
	}
	else
	{
		BoneNames.Empty(1);
		BoneNames.Add(MakeShareable(new FString(TEXT("None"))));
	}
}

#undef LOCTEXT_NAMESPACE