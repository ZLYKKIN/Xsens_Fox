// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnPlugin.h"
#include "MvnLiveLinkPresenceDetector.h"
#include "MvnRemoteControlSession.h"

#define LOCTEXT_NAMESPACE "FLiveLinkMvnPluginModule"

FLiveLinkMvnPluginModule::FLiveLinkMvnPluginModule()
	: PresenceDetector(MakeUnique<FMVNLiveLinkPresenceDetector>())
{
}

void FLiveLinkMvnPluginModule::StartupModule()
{
}

void FLiveLinkMvnPluginModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FLiveLinkMvnPluginModule, LiveLinkMvnPlugin)