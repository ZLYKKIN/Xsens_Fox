// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "XSensLLWrapper.h"
#include "MvnWrapper.h"

#define LOCTEXT_NAMESPACE "FLiveLinkMvnPluginModule"

FXSensLLWrapperModule::FXSensLLWrapperModule()
{
}

void FXSensLLWrapperModule::StartupModule()
{
	MvnWrapper::Start();
}

void FXSensLLWrapperModule::ShutdownModule()
{
	MvnWrapper::ShutDown();
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FXSensLLWrapperModule, XSensLLWrapper)