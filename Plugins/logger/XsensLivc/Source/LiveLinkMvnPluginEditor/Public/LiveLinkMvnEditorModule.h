// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"
#include "Modules/ModuleManager.h"
#include "IAssetTypeActions.h"

class MvnRemoteControlManager;

class FLiveLinkMvnPluginEditorModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

private:
	TArray<TSharedPtr<IAssetTypeActions>> CreatedAssetTypeActions;

    // Track whether remote control tab spawner is registered
    bool bHasRegisteredRemoteControlTabSpawner = false;

    /// register remote control tab spawner in the editor ui
    void RegisterRemoteControlTabSpawner();

    /// unregister remote control tab spawner in the editor ui
    void UnregisterRemoteControlTabSpawner();

    void RegisterSettings();
    void UnregisterSettings();

    /// remote control manager
    MvnRemoteControlManager* RemoteControlManager = nullptr;

};