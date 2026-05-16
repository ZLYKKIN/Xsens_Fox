// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/UnrealEditorSubsystem.h"
#include "MvnPluginEditorSubsystem.generated.h"
/**
 * 
 */

UCLASS()
class LIVELINKMVNPLUGINEDITOR_API UMvnPluginEditorSubsystem : public UUnrealEditorSubsystem
{

	GENERATED_BODY()
public:
	UMvnPluginEditorSubsystem();
	~UMvnPluginEditorSubsystem();

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

	void ReconstructActors();

private:
	FDelegateHandle BlueprintCompiledHandle;
};
