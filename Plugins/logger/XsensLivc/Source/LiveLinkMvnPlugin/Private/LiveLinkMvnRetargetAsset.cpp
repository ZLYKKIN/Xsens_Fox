// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnRetargetAsset.h"

#include "Roles/LiveLinkAnimationTypes.h"
#include "LiveLinkTypes.h"
#include "BonePose.h"
#include "Engine/Blueprint.h"

#include "Animation/AnimSequence.h"
#include "Animation/Skeleton.h"
#include "Engine/SkeletalMesh.h"

#include "SegmentInformation.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "Runtime/Launch/Resources/Version.h"

#if ENGINE_MAJOR_VERSION >= 5
#include "Animation/AttributesRuntime.h"
#else
#include "Animation/CustomAttributesRuntime.h"
#endif
#include "Animation/AnimationPoseData.h"

#include <map>

// todo: this could be built from EXsensMapping reflection information
std::map<FName, EXsensMapping, FNameFastLess> ULiveLinkMvnRetargetAsset::s_remap_bones_names{ {"Root", EXsensMapping::Root}, {"Pelvis", EXsensMapping::Pelvis}, {"L5", EXsensMapping::L5}, {"L3", EXsensMapping::L3}, {"T12", EXsensMapping::T12}, {"T8", EXsensMapping::T8}, {"Neck", EXsensMapping::Neck}, {"Head", EXsensMapping::Head},
					{"RightShoulder", EXsensMapping::RightShoulder}, {"RightUpperArm", EXsensMapping::RightUpperArm},{"RightForeArm", EXsensMapping::RightForeArm},{"RightHand", EXsensMapping::RightHand},
					{"LeftShoulder", EXsensMapping::LeftShoulder}, {"LeftUpperArm", EXsensMapping::LeftUpperArm},{"LeftForeArm", EXsensMapping::LeftForeArm},{"LeftHand", EXsensMapping::LeftHand},
					{"RightUpperLeg", EXsensMapping::RightUpperLeg},{"RightLowerLeg", EXsensMapping::RightLowerLeg},{"RightFoot", EXsensMapping::RightFoot},{"RightToe", EXsensMapping::RightToe},
					{"LeftUpperLeg", EXsensMapping::LeftUpperLeg},{"LeftLowerLeg", EXsensMapping::LeftLowerLeg},{"LeftFoot", EXsensMapping::LeftFoot},{"LeftToe", EXsensMapping::LeftToe},
					{"Prop1", EXsensMapping::Prop1},{"Prop2", EXsensMapping::Prop2},{"Prop3", EXsensMapping::Prop3},{"Prop4", EXsensMapping::Prop4},
					{"LeftCarpus", EXsensMapping::LeftCarpus},{"LeftFirstMC", EXsensMapping::LeftFirstMC},{"LeftFirstPP", EXsensMapping::LeftFirstPP},{"LeftFirstDP", EXsensMapping::LeftFirstDP},
					{"LeftSecondMC", EXsensMapping::LeftSecondMC},{"LeftSecondPP", EXsensMapping::LeftSecondPP},{"LeftSecondMP", EXsensMapping::LeftSecondMP,},{"LeftSecondDP", EXsensMapping::LeftSecondDP},
					{"LeftThirdMC", EXsensMapping::LeftThirdMC},{"LeftThirdPP", EXsensMapping::LeftThirdPP},{"LeftThirdMP", EXsensMapping::LeftThirdMP},{"LeftThirdDP", EXsensMapping::LeftThirdDP},
					{"LeftFourthMC", EXsensMapping::LeftFourthMC},{"LeftFourthPP", EXsensMapping::LeftFourthPP},{"LeftFourthMP", EXsensMapping::LeftFourthMP},{"LeftFourthDP", EXsensMapping::LeftFourthDP},
					{"LeftFifthMC", EXsensMapping::LeftFifthMC},{"LeftFifthPP", EXsensMapping::LeftFifthPP},{"LeftFifthMP", EXsensMapping::LeftFifthMP},{"LeftFifthDP", EXsensMapping::LeftFifthDP},
					{"RightCarpus", EXsensMapping::RightCarpus},{"RightFirstMC", EXsensMapping::RightFirstMC},{"RightFirstPP", EXsensMapping::RightFirstPP},{"RightFirstDP", EXsensMapping::RightFirstDP},
					{"RightSecondMC", EXsensMapping::RightSecondMC},{"RightSecondPP", EXsensMapping::RightSecondPP},{"RightSecondMP", EXsensMapping::RightSecondMP},{"RightSecondDP", EXsensMapping::RightSecondDP},
					{"RightThirdMC", EXsensMapping::RightThirdMC},{"RightThirdPP", EXsensMapping::RightThirdPP},{"RightThirdMP", EXsensMapping::RightThirdMP},{"RightThirdDP", EXsensMapping::RightThirdDP},
					{"RightFourthMC", EXsensMapping::RightFourthMC},{"RightFourthPP", EXsensMapping::RightFourthPP},{"RightFourthMP", EXsensMapping::RightFourthMP},{"RightFourthDP", EXsensMapping::RightFourthDP},
					{"RightFifthMC", EXsensMapping::RightFifthMC},{"RightFifthPP", EXsensMapping::RightFifthPP},{"RightFifthMP", EXsensMapping::RightFifthMP},{"RightFifthDP", EXsensMapping::RightFifthDP}, };


std::map<EXsensMapping, FName> ULiveLinkMvnRetargetAsset::s_bones_map_default{ {EXsensMapping::Root, "root"},
								{EXsensMapping::Pelvis, "pelvis" },
								{EXsensMapping::L5, "spine_01" }, {EXsensMapping::L3, "spine_02", }, {EXsensMapping::T12, "none" }, { EXsensMapping::T8, "spine_03"},
								{EXsensMapping::Neck, "neck_01" }, {EXsensMapping::Head, "head" },
								{EXsensMapping::RightShoulder, "clavicle_r" }, {EXsensMapping::RightUpperArm, "upperarm_r" }, {EXsensMapping::RightForeArm, "lowerarm_r" }, {EXsensMapping::RightHand, "hand_r" },
								{EXsensMapping::LeftShoulder, "clavicle_l" }, {EXsensMapping::LeftUpperArm, "upperarm_l" }, {EXsensMapping::LeftForeArm, "lowerarm_l" }, {EXsensMapping::LeftHand, "hand_l" },
								{EXsensMapping::RightUpperLeg, "thigh_r" }, {EXsensMapping::RightLowerLeg, "calf_r" }, {EXsensMapping::RightFoot, "foot_r" }, {EXsensMapping::RightToe, "ball_r" },
								{EXsensMapping::LeftUpperLeg, "thigh_l" }, {EXsensMapping::LeftLowerLeg, "calf_l" }, {EXsensMapping::LeftFoot, "foot_l" }, {EXsensMapping::LeftToe, "ball_l" },
								{EXsensMapping::LeftCarpus, "hand_l"}, 
								{EXsensMapping::LeftFirstMC, "thumb_01_l"}, {EXsensMapping::LeftFirstPP, "thumb_02_l"},{EXsensMapping::LeftFirstDP, "thumb_03_l"},
								{EXsensMapping::LeftSecondPP, "index_01_l"}, {EXsensMapping::LeftSecondMP, "index_02_l"},{EXsensMapping::LeftSecondDP, "index_03_l"},
								{EXsensMapping::LeftThirdPP, "middle_01_l"}, {EXsensMapping::LeftThirdMP, "middle_02_l"},{EXsensMapping::LeftThirdDP, "middle_03_l"},
								{EXsensMapping::LeftFourthPP, "ring_01_l"}, {EXsensMapping::LeftFourthMP, "ring_02_l"},{EXsensMapping::LeftFourthDP, "ring_03_l"},
								{EXsensMapping::LeftFifthPP, "pinky_01_l"}, {EXsensMapping::LeftFifthMP, "pinky_02_l"},{EXsensMapping::LeftFifthDP, "pinky_03_l"},

								{EXsensMapping::RightCarpus, "hand_r"},
								{EXsensMapping::RightFirstMC, "thumb_01_r"}, {EXsensMapping::RightFirstPP, "thumb_02_r"},{EXsensMapping::RightFirstDP, "thumb_03_r"},
								{EXsensMapping::RightSecondPP, "index_01_r"}, {EXsensMapping::RightSecondMP, "index_02_r"},{EXsensMapping::RightSecondDP, "index_03_r"},
								{EXsensMapping::RightThirdPP, "middle_01_r"}, {EXsensMapping::RightThirdMP, "middle_02_r"},{EXsensMapping::RightThirdDP, "middle_03_r"},
								{EXsensMapping::RightFourthPP, "ring_01_r"}, {EXsensMapping::RightFourthMP, "ring_02_r"},{EXsensMapping::RightFourthDP, "ring_03_r"},
								{EXsensMapping::RightFifthPP, "pinky_01_r"}, {EXsensMapping::RightFifthMP, "pinky_02_r"},{EXsensMapping::RightFifthDP, "pinky_03_r"},

								};

std::map<EXsensMapping, FName> ULiveLinkMvnRetargetAsset::s_bones_map_maya{ {EXsensMapping::Root, "Reference"},
								{EXsensMapping::Pelvis, "Hips" },
								{EXsensMapping::L5, "spine" }, {EXsensMapping::L3, "spine1", }, {EXsensMapping::T12, "spine2" }, { EXsensMapping::T8, "spine3"},
								{EXsensMapping::Neck, "neck" }, {EXsensMapping::Head, "head" },
								{EXsensMapping::RightShoulder, "RightShoulder" }, {EXsensMapping::RightUpperArm, "RightArm" }, {EXsensMapping::RightForeArm, "RightForeArm" }, {EXsensMapping::RightHand, "RightHand" },
								{EXsensMapping::LeftShoulder, "LeftShoulder" }, {EXsensMapping::LeftUpperArm, "LeftArm" }, {EXsensMapping::LeftForeArm, "LeftForeArm" }, {EXsensMapping::LeftHand, "LeftHand" },
								{EXsensMapping::RightUpperLeg, "RightUpLeg" }, {EXsensMapping::RightLowerLeg, "RightLeg" }, {EXsensMapping::RightFoot, "RightFoot" }, {EXsensMapping::RightToe, "RightToeBase" },
								{EXsensMapping::LeftUpperLeg, "LeftUpLeg" }, {EXsensMapping::LeftLowerLeg, "LeftLeg" }, {EXsensMapping::LeftFoot, "LeftFoot" }, {EXsensMapping::LeftToe, "LeftToeBase" },
								
								{EXsensMapping::LeftCarpus, "LeftHand"},
								{EXsensMapping::LeftFirstMC, "LeftHandThumb1"}, {EXsensMapping::LeftFirstPP, "LeftHandThumb2"},{EXsensMapping::LeftFirstDP, "LeftHandThumb3"},
								{EXsensMapping::LeftSecondPP, "LeftHandIndex1"}, {EXsensMapping::LeftSecondMP, "LeftHandIndex2"},{EXsensMapping::LeftSecondDP, "LeftHandIndex3"},
								{EXsensMapping::LeftThirdPP, "LeftHandMiddle1"}, {EXsensMapping::LeftThirdMP, "LeftHandMiddle2"},{EXsensMapping::LeftThirdDP, "LeftHandMiddle3"},
								{EXsensMapping::LeftFourthPP, "LeftHandRing1"}, {EXsensMapping::LeftFourthMP, "LeftHandRing2"},{EXsensMapping::LeftFourthDP, "LeftHandRing3"},
								{EXsensMapping::LeftFifthPP, "LeftHandPinky1"}, {EXsensMapping::LeftFifthMP, "LeftHandPinky2"},{EXsensMapping::LeftFifthDP, "LeftHandPinky3"},

								{EXsensMapping::RightCarpus, "RightHand"},
								{EXsensMapping::RightFirstMC, "RightHandThumb1"}, {EXsensMapping::RightFirstPP, "RightHandThumb2"},{EXsensMapping::RightFirstDP, "RightHandThumb3"},
								{EXsensMapping::RightSecondPP, "RightHandIndex1"}, {EXsensMapping::RightSecondMP, "RightHandIndex2"},{EXsensMapping::RightSecondDP, "RightHandIndex3"},
								{EXsensMapping::RightThirdPP, "RightHandMiddle1"}, {EXsensMapping::RightThirdMP, "RightHandMiddle2"},{EXsensMapping::RightThirdDP, "RightHandMiddle3"},
								{EXsensMapping::RightFourthPP, "RightHandRing1"}, {EXsensMapping::RightFourthMP, "RightHandRing2"},{EXsensMapping::RightFourthDP, "RightHandRing3"},
								{EXsensMapping::RightFifthPP, "RightHandPinky1"}, {EXsensMapping::RightFifthMP, "RightHandPinky2"},{EXsensMapping::RightFifthDP, "RightHandPinky3"},

								};

// todo: this is a reverse of s_remap_bones_names, with exception Root = Reference
std::map<EXsensMapping, FName> ULiveLinkMvnRetargetAsset::s_bones_map_xsens{ {EXsensMapping::Root, "Reference"},
								{EXsensMapping::Pelvis, "Pelvis" },
								{EXsensMapping::L5, "L5" }, {EXsensMapping::L3, "L3", }, {EXsensMapping::T12, "T12" }, { EXsensMapping::T8, "T8"},
								{EXsensMapping::Neck, "Neck" }, {EXsensMapping::Head, "Head" },
								{EXsensMapping::RightShoulder, "RightShoulder" }, {EXsensMapping::RightUpperArm, "RightUpperArm" }, {EXsensMapping::RightForeArm, "RightForeArm" }, {EXsensMapping::RightHand, "RightCarpus" },
								{EXsensMapping::LeftShoulder, "LeftShoulder" }, {EXsensMapping::LeftUpperArm, "LeftUpperArm" }, {EXsensMapping::LeftForeArm, "LeftForeArm" }, {EXsensMapping::LeftHand, "LeftCarpus" },
								{EXsensMapping::RightUpperLeg, "RightUpperLeg" }, {EXsensMapping::RightLowerLeg, "RightLowerLeg" }, {EXsensMapping::RightFoot, "RightFoot" }, {EXsensMapping::RightToe, "RightToe" },
								{EXsensMapping::LeftUpperLeg, "LeftUpperLeg" }, {EXsensMapping::LeftLowerLeg, "LeftLowerLeg" }, {EXsensMapping::LeftFoot, "LeftFoot" }, {EXsensMapping::LeftToe, "LeftToe" },

								{EXsensMapping::LeftCarpus, "LeftCarpus"},
								{EXsensMapping::LeftFirstMC, "LeftFirstMC"}, {EXsensMapping::LeftFirstPP, "LeftFirstPP"},{EXsensMapping::LeftFirstDP, "LeftFirstDP"},
								{EXsensMapping::LeftSecondPP, "LeftSecondPP"}, {EXsensMapping::LeftSecondMP, "LeftSecondMP"},{EXsensMapping::LeftSecondDP, "LeftSecondDP"},
								{EXsensMapping::LeftThirdPP, "LeftThirdPP"}, {EXsensMapping::LeftThirdMP, "LeftThirdMP"},{EXsensMapping::LeftThirdDP, "LeftThirdDP"},
								{EXsensMapping::LeftFourthPP, "LeftFourthPP"}, {EXsensMapping::LeftFourthMP, "LeftFourthMP"},{EXsensMapping::LeftFourthDP, "LeftFourthDP"},
								{EXsensMapping::LeftFifthPP, "LeftFifthPP"}, {EXsensMapping::LeftFifthMP, "LeftFifthMP"},{EXsensMapping::LeftFifthDP, "LeftFifthDP"},

								{EXsensMapping::RightCarpus, "RightCarpus"},
								{EXsensMapping::RightFirstMC, "RightFirstMC"}, {EXsensMapping::RightFirstPP, "RightFirstPP"},{EXsensMapping::RightFirstDP, "RightFirstDP"},
								{EXsensMapping::RightSecondPP, "RightSecondPP"}, {EXsensMapping::RightSecondMP, "RightSecondMP"},{EXsensMapping::RightSecondDP, "RightSecondDP"},
								{EXsensMapping::RightThirdPP, "RightThirdPP"}, {EXsensMapping::RightThirdMP, "RightThirdMP"},{EXsensMapping::RightThirdDP, "RightThirdDP"},
								{EXsensMapping::RightFourthPP, "RightFourthPP"}, {EXsensMapping::RightFourthMP, "RightFourthMP"},{EXsensMapping::RightFourthDP, "RightFourthDP"},
								{EXsensMapping::RightFifthPP, "RightFifthPP"}, {EXsensMapping::RightFifthMP, "RightFifthMP"},{EXsensMapping::RightFifthDP, "RightFifthDP"},

};


ULiveLinkMvnRetargetAsset::ULiveLinkMvnRetargetAsset(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer),
	m_retarget(100),
	m_doLog(false)
{

#if WITH_EDITOR
	UBlueprint* Blueprint = Cast<UBlueprint> ( GetClass ()->ClassGeneratedBy );
	if ( Blueprint )
	{
		OnBlueprintCompiledDelegate = Blueprint->OnCompiled ().AddUObject( this, &ULiveLinkMvnRetargetAsset::OnBlueprintClassCompiled );
	}
#endif
}

void ULiveLinkMvnRetargetAsset::PostLoad()
{
	Super::PostLoad();

	// Upgrade the asset
	if (m_skeletal_mesh_DEPRECATED != nullptr)
	{
		m_skeleton = m_skeletal_mesh_DEPRECATED->GetSkeleton();
		m_skeletal_mesh_DEPRECATED = nullptr;

		if (m_remapping_convention != EXsensRetargetNamingConvention::Custom && m_remapping_convention != EXsensRetargetNamingConvention::Manual)
		{
			// Verify the correctness of remapping convention - if it doesn't match, set it to Manual
			TMap<EXsensMapping, FRemappingRowHandle> SavedRemappingRows = m_remapping_rows;
			AddBoneNamesToRemapTable();
			if (m_remapping_rows.OrderIndependentCompareEqual(SavedRemappingRows) == false)
			{
				// If regeneration of bone list will produce a different result, revert results and force remapping convention to Manual
				m_remapping_rows = SavedRemappingRows;
				m_remapping_convention = EXsensRetargetNamingConvention::Manual;
			}
		}
	}

	FetchSkeletonBoneNames();
}

void ULiveLinkMvnRetargetAsset::BeginDestroy()
{
#if WITH_EDITOR
	if ( OnBlueprintCompiledDelegate.IsValid() )
	{
		if ( UBlueprint* Blueprint = Cast<UBlueprint>( GetClass ()->ClassGeneratedBy ) )
		{
			Blueprint->OnCompiled().Remove( OnBlueprintCompiledDelegate );
		}
		OnBlueprintCompiledDelegate.Reset();
	}
#endif

	Super::BeginDestroy();
}

// Converts Xsens bone name to remapped one
FName ULiveLinkMvnRetargetAsset::GetRemappedBoneName(const FName& BoneName) const
{
	const std::map<FName, EXsensMapping, FNameFastLess>& bonesMap = s_remap_bones_names;
	auto it = bonesMap.find(BoneName);

	if(it != bonesMap.end())
	{
		auto it_row = m_remapping_rows.Find(it->second);
		if(it_row)
			return it_row->RemapId;
	}

	return BoneName;
}

#if WITH_EDITOR

void ULiveLinkMvnRetargetAsset::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property != nullptr)
	{
		FName PropertyName = PropertyChangedEvent.GetPropertyName();
		FName MemberPropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;

		if (PropertyName == GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRetargetAsset, m_skeleton))
		{
			AddBoneNamesToRemapTable();
		}
		else if (PropertyName == GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRetargetAsset, m_remapping_convention))
		{
			AddBoneNamesToRemapTable();
		}
		else if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRetargetAsset, m_remapping_rows))
		{
			// When user changes any row mapping, switch to Manual mapping type, so these changes won't be lost
			m_remapping_convention = EXsensRetargetNamingConvention::Manual;
		}
	}
}

void ULiveLinkMvnRetargetAsset::OnBlueprintClassCompiled( UBlueprint* TargetBlueprint )
{
	AddBoneNamesToRemapTable();
}

#endif // WITH_EDITOR

TArray<FName> ULiveLinkMvnRetargetAsset::PopulateBoneNames(const FLiveLinkSkeletonStaticData* InSkeletonData)
{
	const auto& source_bone_names = InSkeletonData->GetBoneNames();

	TArray<FName> transformed_bone_names;
	transformed_bone_names.Reserve(source_bone_names.Num() + 4);

	//todo: BoneNameMap is never emptied
	for(const auto& bone_name : source_bone_names)
	{
		FName* TargetBoneName = BoneNameMap.Find(bone_name);
		if(TargetBoneName == nullptr)
		{
			FName new_name = GetRemappedBoneName(bone_name);
			transformed_bone_names.Add(new_name);
			BoneNameMap.Add(bone_name, new_name);
		}
		else
			transformed_bone_names.Add(*TargetBoneName);
	}

	return transformed_bone_names;
}

float ULiveLinkMvnRetargetAsset::calculateVectorScale(FVector xsensVec, FVector unrealVec)
{
	float xsensLength = xsensVec.Size();
	float unrealLength = unrealVec.Size();

	return (unrealLength / xsensLength);
}

inline FQuat isRotationFromVecToVec(FVector a, FVector b)
{
	a.Normalize();
	b.Normalize();

	return FQuat::FindBetweenNormals(a, b);
}

#define MYLOG(a, ...) if (m_doLog) UE_LOG(LogTemp, Warning, TEXT(a), __VA_ARGS__)

void ULiveLinkMvnRetargetAsset::calculateTposeValues(FCompactPose OutPose, const FLiveLinkSkeletonStaticData& InSkeletonData, const FLiveLinkAnimationFrameData& InFrameData, FBlendedCurve& OutCurve)
{
	bool logValid = false;
	std::map<int, int> parentOverride;

	auto TransformedBoneNames = PopulateBoneNames(&InSkeletonData);

	//get reference pose values
	TArray<FTransform> TPose = OutPose.GetBoneContainer().GetReferenceSkeleton().GetRefBonePose();
	if(TPoseAnimation)
	{
		MYLOG("TPoseAnimation %p", TPoseAnimation);
		//when upgrading from UE4.23 to 4.26 api changed for GetAnimationPose
		//TPoseAnimation->GetAnimationPose(OutPose, OutCurve, FAnimExtractContext(0, true));
#if ENGINE_MAJOR_VERSION >= 5
		UE::Anim::FStackAttributeContainer attributes;
#else
        FStackCustomAttributes attributes;
#endif
        FAnimationPoseData outAnimationPoseData(OutPose, OutCurve, attributes);
        TPoseAnimation->GetAnimationPose(outAnimationPoseData, FAnimExtractContext( 0.0, true ));
		TPose = OutPose.GetBones();
	}
	FVector uniformScale = OutPose.GetBoneContainer().GetReferenceSkeleton().GetRefBonePose()[0].GetScale3D();
	//calculate the character tpose rotation and position in world space
	if(m_tposeWorld.Num() != 0 && m_tposeWorld.Num() != TPose.Num())
		return;
	m_tposeWorld = TPose;
	for (int32 i = 0; i < TPose.Num(); ++i)
	{
		if (m_tposeWorld.IsValidIndex(i))
		{
			int parentBoneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().GetParentIndex(i);
			if (m_tposeWorld.IsValidIndex(parentBoneIndex))
			{
				m_tposeWorld[i].SetRotation(m_tposeWorld[parentBoneIndex].GetRotation() * TPose[i].GetRotation());
				m_tposeWorld[i].SetTranslation(m_tposeWorld[parentBoneIndex].GetRotation().RotateVector(TPose[i].GetTranslation() * uniformScale) + m_tposeWorld[parentBoneIndex].GetTranslation());
			}
		}
	}

	m_mvnToUnrealTpose.SetNum(InFrameData.Transforms.Num());
	for(int32 i = 0; i < InFrameData.Transforms.Num(); ++i)
	{
		FName BoneName = TransformedBoneNames[i];
		auto boneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().FindBoneIndex(BoneName);
		if(m_tposeWorld.IsValidIndex(boneIndex))
		{
			auto parentBoneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().GetParentIndex(boneIndex);
			m_mvnToUnrealTpose[i].SetRotation(m_tposeWorld[boneIndex].GetRotation());

			FVector tposeVec;

			if(i > 0 && m_tposeWorld.IsValidIndex(parentBoneIndex))
			{
				m_mvnToUnrealTpose[i].SetTranslation(TPose[boneIndex].GetTranslation());
			}
			else
			{
				tposeVec = m_tposeWorld[boneIndex].GetTranslation();
				m_mvnToUnrealTpose[i].SetTranslation(tposeVec);
			}
			MYLOG("m_mvnToUnrealTpose (%f,%f,%f)", m_mvnToUnrealTpose[i].GetTranslation()[0], m_mvnToUnrealTpose[i].GetTranslation()[1], m_mvnToUnrealTpose[i].GetTranslation()[2]);
		}
	}
}

void ULiveLinkMvnRetargetAsset::AddBoneNamesToRemapTable()
{
	if (m_remapping_convention == EXsensRetargetNamingConvention::Manual)
	{
		// Don't change user edited values
		return;
	}

	if (m_skeleton != nullptr)
	{
		FetchSkeletonBoneNames();

		// Attempt to fill according to naming convention
		for (int32 i = 0; i < static_cast<int>(EXsensMapping::Count); i++)
		{
			EXsensMapping CurrBone = EXsensMapping(i);
			// GetRemappedBoneNameByConvention will return 'None' if bone doesn't exist in a mesh
			FName LookupName = GetRemappedBoneNameByConvention( CurrBone, m_remapping_convention );
			m_remapping_rows.Add(CurrBone, FRemappingRowHandle(CurrBone, LookupName));
		}
	}
}

void ULiveLinkMvnRetargetAsset::SetSkeleton(USkeleton* Skeleton)
{
	m_skeleton = Skeleton;
	AddBoneNamesToRemapTable();
}

//todo: unused
void MakeCurveMapFromFrame(const FCompactPose& InPose, const FLiveLinkSkeletonStaticData& InSkeletonData, const FLiveLinkAnimationFrameData& InFrameData, const TArray<FName, TMemStackAllocator<>>& TransformedCurveNames, TMap<FName, float>& OutCurveMap)
{
	OutCurveMap.Reset();
	OutCurveMap.Reserve(InSkeletonData.PropertyNames.Num());

	const USkeleton* Skeleton = InPose.GetBoneContainer().GetSkeletonAsset();

	if(InSkeletonData.PropertyNames.Num() == InFrameData.PropertyValues.Num())
	{
		for(int32 CurveIdx = 0; CurveIdx < InSkeletonData.PropertyNames.Num(); ++CurveIdx)
		{
			float PropertyValue = InFrameData.PropertyValues[CurveIdx];
			OutCurveMap.Add(TransformedCurveNames[CurveIdx]) = PropertyValue;
		}
	}
}

void ULiveLinkMvnRetargetAsset::BuildPoseFromAnimationData(float DeltaTime, const FLiveLinkSkeletonStaticData* InSkeletonData, const FLiveLinkAnimationFrameData* InFrameData, FCompactPose& OutPose)
{
	MYLOG("%hs", "*********************************************");
	MYLOG("Building for %p", this);

	bool logValid = false;
	std::map<int, int> parentOverride;

	auto TransformedBoneNames = PopulateBoneNames(InSkeletonData);

	FVector uniformScale = OutPose.GetBoneContainer().GetReferenceSkeleton().GetRefBonePose()[0].GetScale3D();

	if(m_retarget++ >= 100 || (InFrameData->Transforms.Num() != m_mvnToUnrealTpose.Num()))
	{
		FBlendedCurve OutCurve;
		calculateTposeValues(OutPose, *InSkeletonData, *InFrameData, OutCurve);
		m_retarget = 0;
	}

	TArray<FTransform> SegData;
	for(int32 i = 0; i < InFrameData->Transforms.Num(); ++i)
	{
		FName BoneName = TransformedBoneNames[i];

		FTransform BoneTransform = FTransform::Identity;
		if (!IgnorableBones.Contains(i))
		{
			BoneTransform = InFrameData->Transforms[i];
		}

		int parent = 0;
		SegData.Add(BoneTransform);
		const FName MapBoneName = *BoneNameMap.FindKey(BoneName);

		float fFwdFixAngle = IsForwardY ? -PI / 2.0f : 0;
		FQuat fwdYRotation( FVector::UpVector, fFwdFixAngle );
		FQuat currRot = BoneTransform.GetRotation();
		currRot = currRot * fwdYRotation;
		BoneTransform.SetRotation(currRot);

		//set translation and rotation for the pelvis
		if(MapBoneName == "Pelvis")
		{
			auto boneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().FindBoneIndex(BoneName);
			MYLOG("%d Bone index = %d", i, boneIndex);
			if(boneIndex >= 0)
			{
				//scale the pelvis to the correct height with the Xsens data and Unreal skeleton
				float scale = calculateVectorScale(BoneTransform.GetScale3D(), m_tposeWorld[boneIndex].GetTranslation());
				if(isinf(scale))
					scale = calculateVectorScale(OutPose.GetBoneContainer().GetReferenceSkeleton().GetRefBonePose()[boneIndex].GetTranslation(), m_tposeWorld[boneIndex].GetTranslation());
				//Calculate the pelvis rotation using the mvn and tpose rotation
				SegData[i].SetRotation(BoneTransform.GetRotation() * m_mvnToUnrealTpose[i].GetRotation());
				//scale the position so the pelvis is at the correct height
				SegData[i].SetTranslation(BoneTransform.GetTranslation() * scale);
				SegData[i].SetScale3D(uniformScale);
				MYLOG("final dpos (%f,%f,%f)", SegData[i].GetTranslation()[0], SegData[i].GetTranslation()[1], SegData[i].GetTranslation()[2]);
				BoneTransform = SegData[i];
			}
		}
		else if(i > 23 && i < 29 && MapBoneName.ToString().Contains("Prop"))
		{
			MYLOG("%hs", "Prop");
			//for all props
			auto boneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().FindBoneIndex(BoneName);
			if(boneIndex >= 0)
			{
				//find the parent bone
				auto parentBoneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().GetParentIndex(boneIndex);
				FName parentBoneName = OutPose.GetBoneContainer().GetReferenceSkeleton().GetBoneName(parentBoneIndex);

				const FName* TargetBoneName = BoneNameMap.FindKey(parentBoneName);
				// if the parent is not in the BoneNameMap (retargeted) find the parent of the parent
				while(!TargetBoneName && parentBoneIndex >= 0)
				{
					parentBoneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().GetParentIndex(parentBoneIndex);
					if(parentBoneIndex >= 0)
					{
						parentBoneName = OutPose.GetBoneContainer().GetReferenceSkeleton().GetBoneName(parentBoneIndex);
						TargetBoneName = BoneNameMap.FindKey(parentBoneName);
					}
				}
				if(parentBoneIndex >= 0)
				{
					parent = SegmentInformation::SegmentBoneNames.Find(*TargetBoneName);
					FQuat drot = BoneTransform.GetRotation() * m_mvnToUnrealTpose[i].GetRotation();
					drot = SegData[parent].GetRotation().Inverse() * drot;
					BoneTransform.SetRotation(drot);
					BoneTransform.SetTranslation(m_mvnToUnrealTpose[i].GetTranslation());
					BoneTransform.SetScale3D(OutPose.GetBoneContainer().GetReferenceSkeleton().GetRefBonePose()[boneIndex].GetScale3D());
				}
			}
		}
		else if(i > 0)
		{
			//if an Xsens bone is not mapped to a bone from the current character find a parent that is for the child to use
			if(BoneName == "None")
			{
				parent = SegmentInformation::parentIndex[i];
				int cur = parent;
				FName parentBoneName = *BoneNameMap.Find(SegmentInformation::SegmentBoneNames[parent]);
				while(parentBoneName == "None")
				{
					cur = parent;
					parent = SegmentInformation::parentIndex[parent];
					if ( parent < 0 )
					{
						break;
					}
					parentBoneName = *BoneNameMap.Find(SegmentInformation::SegmentBoneNames[parent]);
				}
				auto parentBoneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().FindBoneIndex(parentBoneName);

				parentOverride[i] = cur;
			}
			else if(BoneName != "p1" && BoneName != "p2" && BoneName != "p3" && BoneName != "p4" && m_mvnToUnrealTpose.Num() > i)
			{
				//xsens parent
				parent = SegmentInformation::parentIndex[i];// -1] + 1;
				//character bone index
				auto boneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().FindBoneIndex(BoneName);
				if(boneIndex >= 0)
				{
					//character parent bone index
					auto parentBoneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().GetParentIndex(boneIndex);
					MYLOG("%d Bone index = %d, parent = %d, parentBoneIndex = %d", i, boneIndex, parent, parentBoneIndex);

					//if parent is not mapped override it with a parent that is
					auto it = parentOverride.find(parent);
					if(it != parentOverride.end())
					{
						parent = it->second;
						MYLOG("%d Overriding parent index with %d", i, parent);
					}

					MYLOG("Xsens Tpose pos (%f,%f,%f), parent Tpose pos (%f,%f,%f)",
						BoneTransform.GetScale3D()[0], BoneTransform.GetScale3D()[1], BoneTransform.GetScale3D()[2],
						InFrameData->Transforms[parent].GetScale3D()[0], InFrameData->Transforms[parent].GetScale3D()[1], InFrameData->Transforms[parent].GetScale3D()[2]);

					//combine the tpose and mvn rotation
					FQuat drot = BoneTransform.GetRotation() * m_mvnToUnrealTpose[i].GetRotation();
					SegData[i].SetRotation(drot);
					// remove the rotation of the parent from the current segment rotation before applying
					drot = SegData[parent].GetRotation().Inverse() * drot;

					BoneTransform.SetRotation(drot);
					BoneTransform.SetTranslation(m_mvnToUnrealTpose[i].GetTranslation());
					BoneTransform.SetScale3D(OutPose.GetBoneContainer().GetReferenceSkeleton().GetRefBonePose()[boneIndex].GetScale3D());
				}
				else
				{
					MYLOG("%d Bone index = %d, parent = %d", i, boneIndex, parent);
				}
			}
		}
		else
		{
			auto boneIndex = OutPose.GetBoneContainer().GetReferenceSkeleton().FindBoneIndex(BoneName);
			MYLOG("%d Bone index = %d", i, boneIndex);
			if(boneIndex >= 0 && m_tposeWorld.Num() > 0)
			{
				MYLOG("Tpose pos (%f,%f,%f) ori (%f,%f,%f,%f)",
					m_tposeWorld[boneIndex].GetTranslation()[0], m_tposeWorld[boneIndex].GetTranslation()[1], m_tposeWorld[boneIndex].GetTranslation()[2],
					m_tposeWorld[boneIndex].GetRotation().W, m_tposeWorld[boneIndex].GetRotation().X, m_tposeWorld[boneIndex].GetRotation().Y, m_tposeWorld[boneIndex].GetRotation().Z);
			}
		}

		int32 MeshIndex = OutPose.GetBoneContainer().GetPoseBoneIndexForBoneName(BoneName);

		if(MeshIndex != INDEX_NONE)
		{
			FCompactPoseBoneIndex CPIndex = OutPose.GetBoneContainer().MakeCompactPoseIndex(FMeshPoseBoneIndex(MeshIndex));
			if(CPIndex != INDEX_NONE)
			{
				OutPose[CPIndex] = BoneTransform;
			}
		}
	}

	//todo: the following code does absolutelynothing, CurveNameMap is useless
	const TArray<FName>& SourceCurveNames = InSkeletonData->PropertyNames;
	TArray<FName, TMemStackAllocator<>> TransformedCurveNames;
	TransformedCurveNames.Reserve(SourceCurveNames.Num());

	for(const FName& SrcCurveName : SourceCurveNames)
	{
		FName* TargetCurveName = CurveNameMap.Find(SrcCurveName);
		if(TargetCurveName == nullptr)
		{
			FName NewName = SrcCurveName;
			TransformedCurveNames.Add(NewName);
			CurveNameMap.Add(SrcCurveName, NewName);
		}
		else
		{
			TransformedCurveNames.Add(*TargetCurveName);
		}
	}

	if(logValid)
		m_doLog = false;
}

void ULiveLinkMvnRetargetAsset::BuildPoseAndCurveFromBaseData(float DeltaTime, const FLiveLinkBaseStaticData* InBaseStaticData, const FLiveLinkBaseFrameData* InBaseFrameData, FCompactPose& OutPose, FBlendedCurve& OutCurve)
{
	TMap<FName, float> BPCurveValues;
	BuildCurveData(BPCurveValues, OutPose, OutCurve);
}

void ULiveLinkMvnRetargetAsset::FetchSkeletonBoneNames()
{
	if (m_skeleton != nullptr)
	{
		const FReferenceSkeleton& RefSkeleton = m_skeleton->GetReferenceSkeleton();

		m_skeleton_bone_names.Empty(RefSkeleton.GetNum());
		for(int32 BoneIndex = 0; BoneIndex < RefSkeleton.GetNum(); BoneIndex++)
		{
			m_skeleton_bone_names.Add(RefSkeleton.GetBoneName(BoneIndex));
		}
	}
	else
	{
		m_skeleton_bone_names.Empty();
	}
}

const std::map<EXsensMapping, FName>& ULiveLinkMvnRetargetAsset::GetNamingConventionBoneMap( EXsensRetargetNamingConvention namingConvention )
{
	if ( namingConvention == EXsensRetargetNamingConvention::Maya )
	{
		return s_bones_map_maya;
	}
	if ( namingConvention == EXsensRetargetNamingConvention::XSens )
	{
		return s_bones_map_xsens;
	}
	return s_bones_map_default;
}

FName ULiveLinkMvnRetargetAsset::GetRemappedBoneNameByConvention( EXsensMapping Bone, EXsensRetargetNamingConvention Convention ) const
{
	FName lookupName = NAME_None;

	if ( Convention == EXsensRetargetNamingConvention::Custom )
	{
		// Execute overridable Blueprint function
		lookupName = GetCustomRemappedBoneName(Bone);
	}
	else
	{ 
		auto& namedBonesMap = GetNamingConventionBoneMap(Convention);
		// if naming convention contains a matching name
		auto found =namedBonesMap.find(Bone);
		if (found != namedBonesMap.end())
		{
			lookupName = found->second;
		}
	}

	// Check if bone with this name exists in skeleton
	if (lookupName != NAME_None && m_skeleton_bone_names.Contains(lookupName))
	{
		return lookupName;
	}
	return NAME_None;
}

// Default implementation, overridable in Blueprint
FName ULiveLinkMvnRetargetAsset::GetCustomRemappedBoneName_Implementation( EXsensMapping Bone ) const
{
	if ( s_bones_map_default.find ( Bone ) != s_bones_map_default.end () )
	{
		return s_bones_map_default[Bone];
	}
	return NAME_None;
}
