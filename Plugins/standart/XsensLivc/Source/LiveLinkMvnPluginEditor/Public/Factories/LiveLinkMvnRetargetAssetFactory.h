// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once
#include "LiveLinkMvnRetargetAsset.h"
#include "Factories/Factory.h"
#include "LiveLinkMvnRetargetAssetFactory.generated.h"


UCLASS()
class LIVELINKMVNPLUGINEDITOR_API ULiveLinkMvnRetargetAssetFactory : public UFactory
{
	GENERATED_BODY()

public:
	ULiveLinkMvnRetargetAssetFactory(const FObjectInitializer& ObjectInitializer);
	virtual UObject* FactoryCreateNew(UClass* InClass, UObject* InParent, FName InName, EObjectFlags Flags, UObject* Context, FFeedbackContext* Warn) override;

	virtual FString GetDefaultNewAssetName() const override;

	virtual FText GetDisplayName() const override;

	virtual uint32 GetMenuCategories() const override;

	TSubclassOf<UObject> ParentClass;

	UBlueprint* MvnRetargetAsset;
};
