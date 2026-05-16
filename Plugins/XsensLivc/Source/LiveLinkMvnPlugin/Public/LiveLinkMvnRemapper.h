

#pragma once

#include "CoreMinimal.h"

#include "Remapper/LiveLinkSkeletonRemapper.h"
#include "SegmentInformation.h"
#include <map>
#include "SegmentIndices.h"
#include "XsensMappingEnum.h"
#include "LiveLinkMvnRemapper.generated.h"

class FLiveLinkMvnRemapperWorker : public FLiveLinkSkeletonRemapperWorker
{
public:

	virtual void RemapSkeletonStaticData(FLiveLinkSkeletonStaticData& InOutSkeletonData);
	TArray<FName> PopulateBoneNames(const FLiveLinkSkeletonStaticData* InSkeletonData);
	void calculateTposeValues(const FLiveLinkSkeletonStaticData& InSkeletonData, const FLiveLinkAnimationFrameData& InFrameData, FBlendedCurve& OutCurve);
	float calculateVectorScale(FVector xsensVec, FVector unrealVec);
	virtual void RemapSkeletonFrameData(const FLiveLinkSkeletonStaticData& InOutSkeletonData, FLiveLinkAnimationFrameData& InOutFrameData) override;
	
	TStrongObjectPtr<USkeletalMesh> ReferenceSkeleton;

	TArray<FTransform> m_tposeWorld;

	TArray<FTransform> m_mvnToUnrealTpose;

	bool IsForwardY = false;

	TArray<TEnumAsByte<SegmentIndexes>> IgnorableBones;

	int m_retarget;
};

/**
 * Remapper class used when streaming via LiveLinkHub
 */
UCLASS(EditInlineNew)
class LIVELINKMVNPLUGIN_API ULiveLinkMvnRemapper : public ULiveLinkSkeletonRemapper
{
public:

	GENERATED_BODY()

	ULiveLinkMvnRemapper();
	~ULiveLinkMvnRemapper();

	virtual FWorkerSharedPtr GetWorker() const override
	{
		return Instance;
	}

	virtual FWorkerSharedPtr CreateWorker() override
	{
		Instance = MakeShared<FLiveLinkMvnRemapperWorker>();
		Instance->BoneNameMap = BoneNameMap;
		Instance->ReferenceSkeleton = TStrongObjectPtr<USkeletalMesh>(ReferenceSkeleton.LoadSynchronous());
		Instance->IsForwardY = IsForwardY;
		Instance->IgnorableBones = IgnorableBones;
		return Instance;
	}

	virtual void Initialize(const FLiveLinkSubjectKey& SubjectKey) override;

	/** Blueprint Implementable function for getting a custom remapped bone name from the original */
	UFUNCTION(BlueprintNativeEvent, Category = "Live Link Remap")
	FName GetCustomRemappedBoneName(EXsensMapping Bone) const;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	void UpdateBoneMap();
	FName GetRemappedBoneNameByConvention(EXsensMapping Bone, EXsensRetargetNamingConvention Convention) const;

	static std::map<FName, EXsensMapping, FNameFastLess> s_remap_bones_names;

	static std::map<EXsensMapping, FName> s_bones_map_default;

	static std::map<EXsensMapping, FName> s_bones_map_maya;

	static std::map<EXsensMapping, FName> s_bones_map_xsens;

	static const std::map<EXsensMapping, FName>& GetNamingConventionBoneMap(EXsensRetargetNamingConvention namingConvention);

	TSharedPtr<FLiveLinkMvnRemapperWorker> Instance;

	UPROPERTY(EditAnywhere, DisplayName = "Is Forward Y", Category = "Reference Pose")
	bool IsForwardY = false;

	UPROPERTY(EditAnywhere, DisplayName = "Naming Convention", BlueprintReadWrite, Category = "Bones Names")
	EXsensRetargetNamingConvention MappingConvention = EXsensRetargetNamingConvention::Default;

	/** Adding a part to this array will prevent the part from receiving data from the live link source. */
	UPROPERTY(EditAnywhere, DisplayName = "Ignored Bones", Category = "Reference Pose")
	TArray<TEnumAsByte<SegmentIndexes>> IgnorableBones;
};
