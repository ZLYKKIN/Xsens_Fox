// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnPlugin.h"
#include "MvnLiveLinkPresenceDetector.h"
#include "MvnRemoteControlSession.h"
#include "FoxLog.h"

#define LOCTEXT_NAMESPACE "FLiveLinkMvnPluginModule"

FLiveLinkMvnPluginModule::FLiveLinkMvnPluginModule()
	: PresenceDetector(MakeUnique<FMVNLiveLinkPresenceDetector>())
{
}

void FLiveLinkMvnPluginModule::StartupModule()
{
	// ue_log.log is (re)created when a stream actually starts (see FLiveLinkMvnSource::InitializeSettings);
	// here we only announce the module to the UE Output Log under the LogFoxStream category.
	UE_LOG(LogFoxStream, Log, TEXT("LiveLinkMvnPlugin module started"));
}

void FLiveLinkMvnPluginModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	FFoxLog::Get().Close(TEXT("reason=module-shutdown"));
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FLiveLinkMvnPluginModule, LiveLinkMvnPlugin)