// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "Customizations/LiveLinkMvnEditorUtilities.h"

#include "UnrealEd.h"
#include "Engine/Selection.h"
#include "ToolMenus.h"
#include "ToolMenuEntry.h"
#include "Misc/Change.h"

#include "Roles/LiveLinkAnimationRole.h"

#include "LiveLinkMvnTransformController.h"

#define LOCTEXT_NAMESPACE "FLiveLinkMvnObjectAttachAction"


class AttachToSourceChange : public FCommandChange
{
public:

	AttachToSourceChange(ULiveLinkComponentController* OriginalComponent,
		FLiveLinkSubjectRepresentation OrignalSubjectRepresentation,
		int32 OriginalSegmentIndex,
		SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment NewSsrc)
	{
		this->OriginalComponent = OriginalComponent;
		this->OrignalSubjectRepresentation = OrignalSubjectRepresentation;
		this->OriginalSegmentIndex = OriginalSegmentIndex;
		this->NewSsrc = NewSsrc;
	}

	virtual void Apply(UObject* Object) override
	{
		AActor* Actor = Cast<AActor>(Object);
		FLiveLinkMvnEditorUtilities::AttachActionApply(Actor, OriginalComponent, NewSsrc);
	}

	virtual void Revert(UObject* Object) override
	{
		AActor* Actor = Cast<AActor>(Object);
		FLiveLinkMvnEditorUtilities::AttachActionRevert(Actor, OriginalComponent != nullptr, 
			OrignalSubjectRepresentation, OriginalSegmentIndex);
	}

	virtual FString ToString() const override
	{
		return TEXT("ULiveLinkComponentController creation");
	}

protected:

	TWeakObjectPtr<ULiveLinkComponentController> OriginalComponent;
	FLiveLinkSubjectRepresentation OrignalSubjectRepresentation;
	int32 OriginalSegmentIndex;
	SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment NewSsrc;
};

class DetachFromSourceChange : public FCommandChange
{
public:

	DetachFromSourceChange(ULiveLinkComponentController* OriginalComponent,
		FLiveLinkSubjectRepresentation OrignalSubjectRepresentation,
		int32 OriginalSegmentIndex)
	{
		this->OriginalComponent = OriginalComponent;
		this->OrignalSubjectRepresentation = OrignalSubjectRepresentation;
		this->OriginalSegmentIndex = OriginalSegmentIndex;
	}

	virtual void Apply(UObject* Object) override
	{
		AActor* Actor = Cast<AActor>(Object);
		FLiveLinkMvnEditorUtilities::DetachActionApply(Actor);
	}

	virtual void Revert(UObject* Object) override
	{
		AActor* Actor = Cast<AActor>(Object);
		FLiveLinkMvnEditorUtilities::DetachActionRevert(Actor, OrignalSubjectRepresentation, OriginalSegmentIndex);
	}

	virtual FString ToString() const override
	{
		return TEXT("ULiveLinkComponentController deletion");
	}

protected:

	TWeakObjectPtr<ULiveLinkComponentController> OriginalComponent;
	FLiveLinkSubjectRepresentation OrignalSubjectRepresentation;
	int32 OriginalSegmentIndex;
};


void FLiveLinkMvnEditorUtilities::OnAttachToSource(SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment NewValue)
{
	TArray<AActor*> SelectedActors;
	GEditor->GetSelectedActors()->GetSelectedObjects<AActor>(SelectedActors);

	if (SelectedActors.Num() == 1) {
		AActor* Actor = SelectedActors[0];

		UEditorEngine* EditorEngine = Cast<UEditorEngine>(GEngine);
		FText transactionDescription = FText::FromString(FString("Attach Actor to LiveLink MVN Source"));
		int32 transaction = EditorEngine->BeginTransaction(transactionDescription);

		SnapshotTransactionBuffer(Actor);
		Actor->Modify();

		ULiveLinkComponentController* LiveLinkComponent = Cast<ULiveLinkComponentController>(Actor->GetComponentByClass(ULiveLinkComponentController::StaticClass()));

		if (GUndo) {
			FLiveLinkSubjectRepresentation SubjectRepresentation = LiveLinkComponent != nullptr
				? LiveLinkComponent->SubjectRepresentation
				: FLiveLinkSubjectRepresentation();
			int32 SegmentIndex = -1;
			if (LiveLinkComponent != nullptr && LiveLinkComponent->ControllerMap.Contains(ULiveLinkAnimationRole::StaticClass()))
			{
				ULiveLinkMvnTransformController* MvnTransformController
					= Cast<ULiveLinkMvnTransformController>(LiveLinkComponent->ControllerMap[ULiveLinkAnimationRole::StaticClass()]);
				if (MvnTransformController)
				{
					SegmentIndex = MvnTransformController->SegmentIndex;
				}
			}

			GUndo->StoreUndo(Actor, MakeUnique<AttachToSourceChange>(LiveLinkComponent, SubjectRepresentation, SegmentIndex, NewValue));
		}

		AttachActionApply(Actor, LiveLinkComponent, NewValue);

		Actor->PostEditChange();
		GUnrealEd->UpdateFloatingPropertyWindows(true);

		EditorEngine->EndTransaction();
	}
}

void FLiveLinkMvnEditorUtilities::OnDetachFromSource()
{
	TArray<AActor*> SelectedActors;
	GEditor->GetSelectedActors()->GetSelectedObjects<AActor>(SelectedActors);

	if (SelectedActors.Num() == 1) {
		AActor* Actor = SelectedActors[0];

		ULiveLinkMvnTransformController* MvnTransformController = nullptr;
		ULiveLinkComponentController* LiveLinkComponent = Cast<ULiveLinkComponentController>(Actor->GetComponentByClass(ULiveLinkComponentController::StaticClass()));
		if (LiveLinkComponent == nullptr || !LiveLinkComponent->ControllerMap.Contains(ULiveLinkAnimationRole::StaticClass()))
		{
			return;
		}
		MvnTransformController = Cast<ULiveLinkMvnTransformController>(LiveLinkComponent->ControllerMap[ULiveLinkAnimationRole::StaticClass()]);
		if (MvnTransformController == nullptr)
		{
			return;
		}


		UEditorEngine* EditorEngine = Cast<UEditorEngine>(GEngine);
		FText transactionDescription = FText::FromString(FString("Attach Actor to LiveLink MVN Source"));
		int32 transaction = EditorEngine->BeginTransaction(transactionDescription);

		SnapshotTransactionBuffer(Actor);
		Actor->Modify();

		if (GUndo) {
			FLiveLinkSubjectRepresentation SubjectRepresentation = LiveLinkComponent->SubjectRepresentation;
			int32 SegmentIndex = MvnTransformController->SegmentIndex;
			GUndo->StoreUndo(Actor, MakeUnique<DetachFromSourceChange>(LiveLinkComponent, SubjectRepresentation, SegmentIndex));
		}

		DetachActionApply(Actor);

		Actor->PostEditChange();
		GUnrealEd->UpdateFloatingPropertyWindows(true);

		EditorEngine->EndTransaction();
	}
}

void FLiveLinkMvnEditorUtilities::AttachActionApply(AActor* Actor, TWeakObjectPtr<ULiveLinkComponentController> Component,
	SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment NewSsrs)
{
	if (Actor == nullptr)
	{
		return;
	}

	if (!Component.IsValid())
	{
		ULiveLinkComponentController* NewComponent = NewObject<ULiveLinkComponentController>(Actor, "LiveLinkController", RF_Public);
		NewComponent->RegisterComponent();
		Actor->AddInstanceComponent(NewComponent);
		Component = NewComponent;
	}

	Component->SubjectRepresentation = NewSsrs.ToSubjectRepresentation();
	Component->OnSubjectRoleChanged();

	if (Component->ControllerMap.Contains(ULiveLinkAnimationRole::StaticClass()))
	{
		ULiveLinkMvnTransformController* MvnTransformController
			= Cast<ULiveLinkMvnTransformController>(Component->ControllerMap[ULiveLinkAnimationRole::StaticClass()]);
		if (MvnTransformController)
		{
			MvnTransformController->SegmentIndex = NewSsrs.SegmentIndex;
		}
	}
}

void FLiveLinkMvnEditorUtilities::AttachActionRevert(AActor* Actor, bool hadComponent,
	FLiveLinkSubjectRepresentation OrignalSubjectRepresentation, int32 OriginalSegmentIndex)
{
	if (Actor == nullptr)
	{
		return;
	}

	ULiveLinkComponentController* LiveLinkComponent = Cast<ULiveLinkComponentController>(Actor->GetComponentByClass(ULiveLinkComponentController::StaticClass()));
	if (!hadComponent)
	{
		if (LiveLinkComponent != nullptr)
		{
			LiveLinkComponent->DestroyComponent();
		}
	}
	else
	{
		LiveLinkComponent->SubjectRepresentation = OrignalSubjectRepresentation;
		LiveLinkComponent->OnSubjectRoleChanged();
		if (LiveLinkComponent->ControllerMap.Contains(ULiveLinkAnimationRole::StaticClass()))
		{
			ULiveLinkMvnTransformController* MvnTransformController
				= Cast<ULiveLinkMvnTransformController>(LiveLinkComponent->ControllerMap[ULiveLinkAnimationRole::StaticClass()]);
			if (MvnTransformController)
			{
				MvnTransformController->SegmentIndex = OriginalSegmentIndex;
			}
		}
	}
}

void FLiveLinkMvnEditorUtilities::DetachActionApply(AActor* Actor)
{
	if (Actor == nullptr)
	{
		return;
	}

	ULiveLinkComponentController* LiveLinkComponent = Cast<ULiveLinkComponentController>(Actor->GetComponentByClass(ULiveLinkComponentController::StaticClass()));
	if (LiveLinkComponent != nullptr)
	{
		LiveLinkComponent->DestroyComponent();
	}
}

void FLiveLinkMvnEditorUtilities::DetachActionRevert(AActor* Actor, FLiveLinkSubjectRepresentation OrignalSubjectRepresentation,
	int32 OriginalSegmentIndex)
{
	if (Actor == nullptr)
	{
		return;
	}

	ULiveLinkComponentController* NewComponent = NewObject<ULiveLinkComponentController>(Actor, "LiveLinkController", RF_Public);
	NewComponent->RegisterComponent();
	Actor->AddInstanceComponent(NewComponent);

	NewComponent->SubjectRepresentation = OrignalSubjectRepresentation;
	NewComponent->OnSubjectRoleChanged();

	if (NewComponent->ControllerMap.Contains(ULiveLinkAnimationRole::StaticClass()))
	{
		ULiveLinkMvnTransformController* MvnTransformController
			= Cast<ULiveLinkMvnTransformController>(NewComponent->ControllerMap[ULiveLinkAnimationRole::StaticClass()]);
		if (MvnTransformController)
		{
			MvnTransformController->SegmentIndex = OriginalSegmentIndex;
		}
	}
}

void FLiveLinkMvnEditorUtilities::AddLiveLinkSourcePicker(UToolMenu* Menu)
{
	SLiveLinkMvnSubjectRepresentationPicker::FOnValueChanged OnValueChange;
	OnValueChange.BindStatic(&FLiveLinkMvnEditorUtilities::OnAttachToSource);

	SLiveLinkMvnSubjectRepresentationPicker::FOnDetach OnDetach;
	OnDetach.BindStatic(&FLiveLinkMvnEditorUtilities::OnDetachFromSource);

	SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment CurrentValue = GetCurrentAttachedTo();

	TSharedRef<SWidget> PickerWidget = SNew(SLiveLinkMvnSubjectRepresentationPicker)
		.Value(CurrentValue)
		.OnValueChanged(OnValueChange)
		.OnDetach(OnDetach)
	;


	FToolMenuSection& Section = Menu->AddSection("LiveLink");
	Section.AddEntry(FToolMenuEntry::InitWidget("PickParentActor", PickerWidget, FText::GetEmpty(), false));
}

SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment FLiveLinkMvnEditorUtilities::GetCurrentAttachedTo()
{
	TArray<AActor*> SelectedActors;
	GEditor->GetSelectedActors()->GetSelectedObjects<AActor>(SelectedActors);

	if (SelectedActors.Num() == 1) {
		AActor* Actor = SelectedActors[0];

		ULiveLinkComponentController* LiveLinkComponent = Cast<ULiveLinkComponentController>(Actor->GetComponentByClass(ULiveLinkComponentController::StaticClass()));
		if (LiveLinkComponent)
		{
			int32 SegmentIndex = 0;
			if (LiveLinkComponent->ControllerMap.Contains(ULiveLinkAnimationRole::StaticClass()))
			{
				ULiveLinkMvnTransformController* MvnTransformController
					= Cast<ULiveLinkMvnTransformController>(LiveLinkComponent->ControllerMap[ULiveLinkAnimationRole::StaticClass()]);
				if (MvnTransformController)
				{
					SegmentIndex = MvnTransformController->SegmentIndex;
				}
			}
			return SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment(LiveLinkComponent->SubjectRepresentation, SegmentIndex);
		}
	}

	return SLiveLinkMvnSubjectRepresentationPicker::FLiveLinkMvnSourceSubjectRoleSegment(FLiveLinkSubjectRepresentation(), -1);
}

#undef LOCTEXT_NAMESPACE
