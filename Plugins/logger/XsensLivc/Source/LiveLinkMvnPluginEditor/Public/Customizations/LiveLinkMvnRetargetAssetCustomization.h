// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"
#include "XsensMappingEnum.h"
#include "MapBuilder.h"

class IPropertyHandle;
class IDetailChildrenBuilder;
class ULiveLinkMvnRetargetAsset;
class USkeleton;

class LiveLinkMvnRetargetAssetCustomization : public IDetailCustomization, IDetailMapBuilderCallback
{
public:

	static TSharedRef<IDetailCustomization> MakeInstance();

	// IDetailCustomization
	// This is where all the UI overriding magic is hapenning.
	virtual void CustomizeDetails(IDetailLayoutBuilder & DetailBuilder) override;

	// IDetailMapBuilderCallback
	virtual void GenerateChildWidget(IDetailChildrenBuilder& ChildrenBuilder, TSharedRef<IPropertyHandle> ItemProperty) override;
	
private:

	void FetchSkeletonBoneNames(class USkeleton* InSkeleton);

	TArray<TSharedPtr<FString>> BoneNames;
	ULiveLinkMvnRetargetAsset* RetargetAsset;
	USkeleton* Skeleton;
};
