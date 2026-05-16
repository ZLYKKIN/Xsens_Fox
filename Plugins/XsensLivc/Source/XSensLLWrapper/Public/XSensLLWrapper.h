// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FXSensLLWrapperModule : public IModuleInterface
{
public:
	FXSensLLWrapperModule();

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static FXSensLLWrapperModule& Get()
	{
		return FModuleManager::LoadModuleChecked<FXSensLLWrapperModule>("XSensLLWrapper");
	}

};
