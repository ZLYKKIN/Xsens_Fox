#include "LiveLinkMvnRemapper.h"
#include "Animation/AnimCurveTypes.h"

std::map<FName, EXsensMapping, FNameFastLess> ULiveLinkMvnRemapper::s_remap_bones_names{ {"Root", EXsensMapping::Root}, {"Pelvis", EXsensMapping::Pelvis}, {"L5", EXsensMapping::L5}, {"L3", EXsensMapping::L3}, {"T12", EXsensMapping::T12}, {"T8", EXsensMapping::T8}, {"Neck", EXsensMapping::Neck}, {"Head", EXsensMapping::Head},
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


std::map<EXsensMapping, FName> ULiveLinkMvnRemapper::s_bones_map_default{ {EXsensMapping::Root, "root"},
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

std::map<EXsensMapping, FName> ULiveLinkMvnRemapper::s_bones_map_maya{ {EXsensMapping::Root, "Reference"},
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
std::map<EXsensMapping, FName> ULiveLinkMvnRemapper::s_bones_map_xsens{ {EXsensMapping::Root, "Reference"},
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

ULiveLinkMvnRemapper::ULiveLinkMvnRemapper()
{
}

ULiveLinkMvnRemapper::~ULiveLinkMvnRemapper()
{
}

void ULiveLinkMvnRemapper::Initialize(const FLiveLinkSubjectKey& SubjectKey)
{
	UpdateBoneMap();
}

#if WITH_EDITOR
void ULiveLinkMvnRemapper::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	if (PropertyChangedEvent.Property != nullptr)
	{
		FName PropertyName = PropertyChangedEvent.GetPropertyName();
		FName MemberPropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;

		if (PropertyName == GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRemapper, ReferenceSkeleton))
		{
			UpdateBoneMap();
		}
		else if (PropertyName == GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRemapper, MappingConvention))
		{
			UpdateBoneMap();
		}
		else if (MemberPropertyName == GET_MEMBER_NAME_CHECKED(ULiveLinkMvnRemapper, BoneNameMap))
		{
			// When user changes any row mapping, switch to Manual mapping type, so these changes won't be lost
			MappingConvention = EXsensRetargetNamingConvention::Manual;
		}
	}
}
#endif

void ULiveLinkMvnRemapper::UpdateBoneMap()
{
	BoneNameMap.Empty();

	for (int32 i = 0; i < static_cast<int>(EXsensMapping::Count); i++)
	{
		EXsensMapping CurrBone = EXsensMapping(i);
		auto BoneNameIt = std::find_if(std::begin(s_remap_bones_names), std::end(s_remap_bones_names), [&](const std::pair<FName, EXsensMapping>& pair)
			{
				return pair.second == CurrBone;
			});
		if(BoneNameIt != s_remap_bones_names.end())
		{
			BoneNameMap.Add(BoneNameIt->first, GetRemappedBoneNameByConvention(CurrBone, MappingConvention));
		}
	}
}

const std::map<EXsensMapping, FName>& ULiveLinkMvnRemapper::GetNamingConventionBoneMap(EXsensRetargetNamingConvention namingConvention)
{
	if (namingConvention == EXsensRetargetNamingConvention::Maya)
	{
		return s_bones_map_maya;
	}
	if (namingConvention == EXsensRetargetNamingConvention::XSens)
	{
		return s_bones_map_xsens;
	}
	return s_bones_map_default;
}

FName ULiveLinkMvnRemapper::GetRemappedBoneNameByConvention(EXsensMapping Bone, EXsensRetargetNamingConvention Convention) const
{
	FName lookupName = NAME_None;

	if (Convention == EXsensRetargetNamingConvention::Custom)
	{
		// Execute overridable Blueprint function
		lookupName = GetCustomRemappedBoneName(Bone);
	}
	else
	{
		auto& namedBonesMap = GetNamingConventionBoneMap(Convention);
		// if naming convention contains a matching name
		auto found = namedBonesMap.find(Bone);
		if (found != namedBonesMap.end())
		{
			lookupName = found->second;
		}
	}

	return lookupName;
}

// Default implementation, overridable in Blueprint
FName ULiveLinkMvnRemapper::GetCustomRemappedBoneName_Implementation(EXsensMapping Bone) const
{
	if (s_bones_map_default.find(Bone) != s_bones_map_default.end())
	{
		return s_bones_map_default[Bone];
	}
	return NAME_None;
}

void FLiveLinkMvnRemapperWorker::RemapSkeletonStaticData(FLiveLinkSkeletonStaticData& InOutSkeletonData)
{
	if (!ReferenceSkeleton)
	{
		return;
	}

	const FReferenceSkeleton& RefSkeleton = ReferenceSkeleton->GetRefSkeleton();
	auto TargetBoneNames = PopulateBoneNames(&InOutSkeletonData);

	InOutSkeletonData.SetBoneNames(TargetBoneNames);
}

TArray<FName> FLiveLinkMvnRemapperWorker::PopulateBoneNames(const FLiveLinkSkeletonStaticData* InSkeletonData)
{
	const auto& source_bone_names = InSkeletonData->GetBoneNames();

	TArray<FName> transformed_bone_names;
	transformed_bone_names.Reserve(source_bone_names.Num() + 4);

	//todo: BoneNameMap is never emptied
	for (const auto& bone_name : source_bone_names)
	{
		FName* TargetBoneName = BoneNameMap.Find(bone_name);
		if (TargetBoneName == nullptr)
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

void FLiveLinkMvnRemapperWorker::calculateTposeValues(const FLiveLinkSkeletonStaticData& InSkeletonData, const FLiveLinkAnimationFrameData& InFrameData, FBlendedCurve& OutCurve)
{
	bool logValid = false;
	std::map<int, int> parentOverride;

	auto TransformedBoneNames = PopulateBoneNames(&InSkeletonData);

	//get reference pose values
	TArray<FTransform> TPose = ReferenceSkeleton->GetRefSkeleton().GetRefBonePose();
	
	FVector uniformScale = ReferenceSkeleton->GetRefSkeleton().GetRefBonePose()[0].GetScale3D();
	//calculate the character tpose rotation and position in world space
	if (m_tposeWorld.Num() != 0 && m_tposeWorld.Num() != TPose.Num())
		return;
	m_tposeWorld = TPose;
	for (int32 i = 0; i < TPose.Num(); ++i)
	{
		if (m_tposeWorld.IsValidIndex(i))
		{
			int parentBoneIndex = ReferenceSkeleton->GetRefSkeleton().GetParentIndex(i);
			if (m_tposeWorld.IsValidIndex(parentBoneIndex))
			{
				m_tposeWorld[i].SetRotation(m_tposeWorld[parentBoneIndex].GetRotation() * TPose[i].GetRotation());
				m_tposeWorld[i].SetTranslation(m_tposeWorld[parentBoneIndex].GetRotation().RotateVector(TPose[i].GetTranslation() * uniformScale) + m_tposeWorld[parentBoneIndex].GetTranslation());
			}
		}
	}

	m_mvnToUnrealTpose.SetNum(InFrameData.Transforms.Num());
	for (int32 i = 0; i < InFrameData.Transforms.Num(); ++i)
	{
		FName BoneName = TransformedBoneNames[i];
		auto boneIndex = ReferenceSkeleton->GetRefSkeleton().FindBoneIndex(BoneName);
		if (m_tposeWorld.IsValidIndex(boneIndex))
		{
			auto parentBoneIndex = ReferenceSkeleton->GetRefSkeleton().GetParentIndex(boneIndex);
			m_mvnToUnrealTpose[i].SetRotation(m_tposeWorld[boneIndex].GetRotation());

			FVector tposeVec;

			if (i > 0 && m_tposeWorld.IsValidIndex(parentBoneIndex))
			{
				m_mvnToUnrealTpose[i].SetTranslation(TPose[boneIndex].GetTranslation());
			}
			else
			{
				tposeVec = m_tposeWorld[boneIndex].GetTranslation();
				m_mvnToUnrealTpose[i].SetTranslation(tposeVec);
			}
		}
	}
}

float FLiveLinkMvnRemapperWorker::calculateVectorScale(FVector xsensVec, FVector unrealVec)
{
	float xsensLength = xsensVec.Size();
	float unrealLength = unrealVec.Size();

	return (unrealLength / xsensLength);
}

void FLiveLinkMvnRemapperWorker::RemapSkeletonFrameData(const FLiveLinkSkeletonStaticData& InOutSkeletonData, FLiveLinkAnimationFrameData& InOutFrameData)
{
	if (!ReferenceSkeleton)
	{
		return;
	}

	bool logValid = false;
	std::map<int, int> parentOverride;

	auto TransformedBoneNames = PopulateBoneNames(&InOutSkeletonData);

	FVector uniformScale = ReferenceSkeleton->GetRefSkeleton().GetRefBonePose()[0].GetScale3D();

	if (m_retarget++ >= 100 || (InOutFrameData.Transforms.Num() != m_mvnToUnrealTpose.Num()))
	{
		FBlendedCurve OutCurve;
		calculateTposeValues(InOutSkeletonData, InOutFrameData, OutCurve);
		m_retarget = 0;
	}

	TArray<FTransform> SegData;
	for (int32 i = 0; i < InOutFrameData.Transforms.Num(); ++i)
	{
		FName BoneName = TransformedBoneNames[i];

		FTransform BoneTransform = FTransform::Identity;
		if (!IgnorableBones.Contains(i))
		{
			BoneTransform = InOutFrameData.Transforms[i];
		}

		int parent = 0;
		SegData.Add(BoneTransform);

		float fFwdFixAngle = IsForwardY ? -PI / 2.0f : 0;
		FQuat fwdYRotation(FVector::UpVector, fFwdFixAngle);
		FQuat currRot = BoneTransform.GetRotation();
		currRot = currRot * fwdYRotation;
		BoneTransform.SetRotation(currRot);

		const FName MapBoneName = *BoneNameMap.FindKey(BoneName);

		//set translation and rotation for the pelvis
		if (MapBoneName == "Pelvis")
		{
			auto boneIndex = ReferenceSkeleton->GetRefSkeleton().FindBoneIndex(BoneName);
			if (boneIndex >= 0)
			{
				//scale the pelvis to the correct height with the Xsens data and Unreal skeleton
				float scale = calculateVectorScale(BoneTransform.GetScale3D(), m_tposeWorld[boneIndex].GetTranslation());
				if (isinf(scale))
					scale = calculateVectorScale(ReferenceSkeleton->GetRefSkeleton().GetRefBonePose()[boneIndex].GetTranslation(), m_tposeWorld[boneIndex].GetTranslation());
				//Calculate the pelvis rotation using the mvn and tpose rotation
				SegData[i].SetRotation(BoneTransform.GetRotation() * m_mvnToUnrealTpose[i].GetRotation());
				//scale the position so the pelvis is at the correct height
				SegData[i].SetTranslation(BoneTransform.GetTranslation() * scale);
				SegData[i].SetScale3D(uniformScale);
				BoneTransform = SegData[i];
				InOutFrameData.Transforms[i] = BoneTransform;
			}
		}
		else if (i > 23 && i < 29 && MapBoneName.ToString().Contains("Prop"))
		{
			//for all props
			auto boneIndex = ReferenceSkeleton->GetRefSkeleton().FindBoneIndex(BoneName);
			if (boneIndex >= 0)
			{
				//find the parent bone
				auto parentBoneIndex = ReferenceSkeleton->GetRefSkeleton().GetParentIndex(boneIndex);
				FName parentBoneName = ReferenceSkeleton->GetRefSkeleton().GetBoneName(parentBoneIndex);

				const FName* TargetBoneName = BoneNameMap.FindKey(parentBoneName);
				// if the parent is not in the BoneNameMap (retargeted) find the parent of the parent
				while (!TargetBoneName && parentBoneIndex >= 0)
				{
					parentBoneIndex = ReferenceSkeleton->GetRefSkeleton().GetParentIndex(parentBoneIndex);
					if (parentBoneIndex >= 0)
					{
						parentBoneName = ReferenceSkeleton->GetRefSkeleton().GetBoneName(parentBoneIndex);
						TargetBoneName = BoneNameMap.FindKey(parentBoneName);
					}
				}
				if (parentBoneIndex >= 0)
				{
					parent = SegmentInformation::SegmentBoneNames.Find(*TargetBoneName);
					FQuat drot = BoneTransform.GetRotation() * m_mvnToUnrealTpose[i].GetRotation();
					drot = SegData[parent].GetRotation().Inverse() * drot;
					BoneTransform.SetRotation(drot);
					BoneTransform.SetTranslation(m_mvnToUnrealTpose[i].GetTranslation());
					BoneTransform.SetScale3D(ReferenceSkeleton->GetRefSkeleton().GetRefBonePose()[boneIndex].GetScale3D());
					InOutFrameData.Transforms[i] = BoneTransform;
				}
			}
		}
		else if (i > 0)
		{
			//if an Xsens bone is not mapped to a bone from the current character find a parent that is for the child to use
			if (BoneName == "None")
			{
				parent = SegmentInformation::parentIndex[i];
				int cur = parent;
				FName parentBoneName = *BoneNameMap.Find(SegmentInformation::SegmentBoneNames[parent]);
				while (parentBoneName == "None")
				{
					cur = parent;
					parent = SegmentInformation::parentIndex[parent];
					if (parent < 0)
					{
						break;
					}
					parentBoneName = *BoneNameMap.Find(SegmentInformation::SegmentBoneNames[parent]);
				}
				auto parentBoneIndex = ReferenceSkeleton->GetRefSkeleton().FindBoneIndex(parentBoneName);

				parentOverride[i] = cur;
			}
			else if (BoneName != "p1" && BoneName != "p2" && BoneName != "p3" && BoneName != "p4" && m_mvnToUnrealTpose.Num() > i)
			{
				//xsens parent
				parent = SegmentInformation::parentIndex[i];// -1] + 1;
				//character bone index
				auto boneIndex = ReferenceSkeleton->GetRefSkeleton().FindBoneIndex(BoneName);
				if (boneIndex >= 0)
				{
					//character parent bone index
					auto parentBoneIndex = ReferenceSkeleton->GetRefSkeleton().GetParentIndex(boneIndex);

					//if parent is not mapped override it with a parent that is
					auto it = parentOverride.find(parent);
					if (it != parentOverride.end())
					{
						parent = it->second;
					}

					//combine the tpose and mvn rotation
					FQuat drot = BoneTransform.GetRotation() * m_mvnToUnrealTpose[i].GetRotation();
					SegData[i].SetRotation(drot);
					// remove the rotation of the parent from the current segment rotation before applying
					drot = SegData[parent].GetRotation().Inverse() * drot;

					BoneTransform.SetRotation(drot);
					BoneTransform.SetTranslation(m_mvnToUnrealTpose[i].GetTranslation());
					BoneTransform.SetScale3D(ReferenceSkeleton->GetRefSkeleton().GetRefBonePose()[boneIndex].GetScale3D());
					InOutFrameData.Transforms[i] = BoneTransform;
				}
			}
		}
	}
}

