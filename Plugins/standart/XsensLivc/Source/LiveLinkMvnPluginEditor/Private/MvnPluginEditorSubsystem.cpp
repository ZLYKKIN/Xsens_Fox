// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnPluginEditorSubsystem.h"
#include "LiveLinkComponent.h"
#include <Editor.h>


UMvnPluginEditorSubsystem::UMvnPluginEditorSubsystem()
{
}

UMvnPluginEditorSubsystem::~UMvnPluginEditorSubsystem()
{
}

void UMvnPluginEditorSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	if (GEditor)
	{
		FEditorDelegates::OnMapOpened.AddLambda([this](const FString& MapName, bool Unused)
		{
			// XUU-186: Force actor reconstruction so animation is updated immediately when switching
			// live link client subjects and recompiling the animation blueprint
			if (!BlueprintCompiledHandle.IsValid())
			{
				BlueprintCompiledHandle = GEditor->OnBlueprintCompiled().AddUObject(this, &UMvnPluginEditorSubsystem::ReconstructActors);
			}
		});
	}
}

void UMvnPluginEditorSubsystem::Deinitialize()
{

	if (GEditor && BlueprintCompiledHandle.IsValid())
	{
		GEditor->OnBlueprintCompiled().Remove(BlueprintCompiledHandle);
		BlueprintCompiledHandle.Reset();
	}
}

void UMvnPluginEditorSubsystem::ReconstructActors()
{
	if (!GEditor || !GEditor->GetWorld())
	{
		return;
	}

	TArray<UObject*> Results;
	TSet<AActor*> OwningActors;
	GetObjectsOfClass(ULiveLinkComponent::StaticClass(), Results);
	for (UObject* Component : Results)
	{
		ULiveLinkComponent* LiveLinkComponent = Cast<ULiveLinkComponent>(Component);
		if (LiveLinkComponent)
		{
			AActor* SkeletonActor = LiveLinkComponent->GetOwner();
			if (SkeletonActor)
			{
				OwningActors.Add(SkeletonActor);
			}
		}
	}

	for (AActor* SkeletonActor : OwningActors)
	{
		SkeletonActor->RerunConstructionScripts();
	}
}
