// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once
#include "Settings/EditorLoadingSavingSettings.h"
#include "AssetTypeActions_Base.h"
#include "Animation/Skeleton.h"
#include "EditorAnimUtils.h"
#include "IAssetTypeActions.h"
#include "SLiveLinkMvnSubjectRepresentationPicker.h"
#include "LiveLinkComponentController.h"

class FLiveLinkMvnEditorUtilities
{
public:
	static void AddLiveLinkSourcePicker(UToolMenu* Menu);
	static void OnAttachToSource(SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment NewValue);
	static SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment GetCurrentAttachedTo();
	static void OnDetachFromSource();

	static void AttachActionApply(AActor* Actor, TWeakObjectPtr<ULiveLinkComponentController> Component,
		SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment NewSsrs);
	static void AttachActionRevert(AActor* Actor, bool HadComponent, FLiveLinkSubjectRepresentation OrignalSubjectRepresentation,
		int32 OriginalSegmentIndex);

	static void DetachActionApply(AActor* Actor);
	static void DetachActionRevert(AActor* Actor, FLiveLinkSubjectRepresentation OrignalSubjectRepresentation,
		int32 OriginalSegmentIndex);
};