// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "Factories/LiveLinkMvnRetargetAssetFactory.h"
#include "LiveLinkMvnRetargetAsset.h"
#include "UObject/Object.h"
#include "UnrealEd.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "KismetCompilerModule.h"

#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "Modules/ModuleManager.h"

#define LOCTEXT_NAMESPACE "LiveLinkMvnRetargetAssetFactory"

ULiveLinkMvnRetargetAssetFactory::ULiveLinkMvnRetargetAssetFactory(const FObjectInitializer& ObjectInitializer) :Super(ObjectInitializer)
{
	bCreateNew = true;
	bEditAfterNew = true;
	MvnRetargetAsset = nullptr;

	// Fool the engine saying that this factory will create a blueprint, but remember actual class for creation
	SupportedClass = UBlueprint::StaticClass();
	ParentClass = ULiveLinkMvnRetargetAsset::StaticClass();
}

UObject* ULiveLinkMvnRetargetAssetFactory::FactoryCreateNew(UClass* Class, UObject* InParent, FName Name, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn)
{
	if (MvnRetargetAsset != nullptr)
	{
		MvnRetargetAsset->SetFlags(Flags | RF_NoFlags);
		MvnRetargetAsset->Modify();
		MvnRetargetAsset->Rename(*Name.ToString(), InParent);
	}
	else
	{
		UClass* BlueprintClass = nullptr;
		UClass* BlueprintGeneratedClass = nullptr;

		// We'll substitute the SupportedClass with own class, then create a new blueprint
		ensure(Class == SupportedClass);
		IKismetCompilerInterface& KismetCompilerModule = FModuleManager::LoadModuleChecked<IKismetCompilerInterface>("KismetCompiler");
		KismetCompilerModule.GetBlueprintTypesForClass(ParentClass, BlueprintClass, BlueprintGeneratedClass);

		MvnRetargetAsset = FKismetEditorUtilities::CreateBlueprint(ParentClass, InParent, Name, BPTYPE_Normal, BlueprintClass, BlueprintGeneratedClass);

	}
	return MvnRetargetAsset;
}

FString ULiveLinkMvnRetargetAssetFactory::GetDefaultNewAssetName() const
{
	return FString(TEXT("New")) + ParentClass->GetName();
}

FText ULiveLinkMvnRetargetAssetFactory::GetDisplayName() const
{
	return LOCTEXT("DisplayName", "LiveLink Mvn Retarget Asset");
}

uint32 ULiveLinkMvnRetargetAssetFactory::GetMenuCategories() const
{
	// Borrow the menu category from LiveLink plugin
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
	return AssetTools.RegisterAdvancedAssetCategory("LiveLink", NSLOCTEXT("LiveLinkBlueprintVirtualSubjectFactory", "AssetCategoryName", "Live Link"));
}

#undef LOCTEXT_NAMESPACE
