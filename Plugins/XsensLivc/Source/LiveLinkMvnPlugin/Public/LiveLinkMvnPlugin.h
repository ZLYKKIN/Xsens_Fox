// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FMVNLiveLinkPresenceDetector;

class FLiveLinkMvnPluginModule : public IModuleInterface
{
public:
	FLiveLinkMvnPluginModule();

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static FLiveLinkMvnPluginModule& Get()
	{
		return FModuleManager::LoadModuleChecked<FLiveLinkMvnPluginModule>("LiveLinkMvnPlugin");
	}

	virtual FMVNLiveLinkPresenceDetector& GetPresenceDetector() { return *PresenceDetector; }

private:
	TUniquePtr<FMVNLiveLinkPresenceDetector> PresenceDetector;
};
