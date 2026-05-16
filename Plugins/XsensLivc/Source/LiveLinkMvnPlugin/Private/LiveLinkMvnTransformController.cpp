// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnTransformController.h"

#include "LiveLinkComponentController.h"
#include "Roles/LiveLinkAnimationRole.h"
#include "Roles/LiveLinkAnimationTypes.h"

bool ULiveLinkMvnTransformController::IsRoleSupported(const TSubclassOf<ULiveLinkRole>& RoleToSupport)
{
	return RoleToSupport ==  ULiveLinkAnimationRole::StaticClass();
}

TSubclassOf<UActorComponent> ULiveLinkMvnTransformController::GetDesiredComponentClass() const
{
	return USceneComponent::StaticClass();
}

void ULiveLinkMvnTransformController::Tick(float DeltaTime, const FLiveLinkSubjectFrameData& SubjectData)
{
	const FLiveLinkSkeletonStaticData* StaticData = SubjectData.StaticData.Cast<FLiveLinkSkeletonStaticData>();
	const FLiveLinkAnimationFrameData* FrameData = SubjectData.FrameData.Cast<FLiveLinkAnimationFrameData>();

	if (StaticData && FrameData)
	{
		if (USceneComponent* SceneComponent = Cast<USceneComponent>(GetAttachedComponent()))
		{
			if (SegmentIndex >= 0 && SegmentIndex < FrameData->Transforms.Num()) {
				FTransform xfm = FrameData->Transforms[SegmentIndex];
				xfm.SetScale3D(SceneComponent->GetRelativeScale3D());

				// Objects should have an additional transform
				FString SubjectName = GetSelectedSubject().Subject.Name.ToString();
				bool bShouldFixTransform = SubjectName.EndsWith(TEXT("-Objects"));

				if (bShouldFixTransform)
				{
					FQuat rot = xfm.GetRotation();
//					rot = (FQuat(rot.Y, -rot.X, rot.Z, rot.W) * FQuat::MakeFromEuler(FVector(0.0f, 0.0f, 180.0f))).GetNormalized();
					xfm.SetRotation(FQuat(-rot.X, -rot.Y, rot.W, -rot.Z));
				}

				SceneComponent->SetRelativeTransform(xfm, false, nullptr, ETeleportType::TeleportPhysics);
			}
		}
	}
}