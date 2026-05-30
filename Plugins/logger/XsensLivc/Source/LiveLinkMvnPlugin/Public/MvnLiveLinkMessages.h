// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "LiveLinkMessages.h"
#include "UObject/Object.h"
#include "MvnLiveLinkMessages.generated.h"

USTRUCT()
struct LIVELINKMVNPLUGIN_API FMvnLiveLinkPingMessage : public FLiveLinkPingMessage
{
	GENERATED_BODY()

	UPROPERTY()
	FString XSensVersion = "v23.2.0";

	UPROPERTY()
	FString UnrealVersion = "5.3";

	FMvnLiveLinkPingMessage();

	FMvnLiveLinkPingMessage(const FGuid& InPollRequest, int32 InLiveLinkVersion);
};

USTRUCT()
struct LIVELINKMVNPLUGIN_API FMvnLiveLinkPongMessage : public FLiveLinkPongMessage
{

	GENERATED_BODY()

	UPROPERTY()
	FString XSensVersion = "v23.2.0";

	UPROPERTY()
	FString UnrealVersion = "5.3";

	FMvnLiveLinkPongMessage();

	FMvnLiveLinkPongMessage(const FString& InProviderName, const FString& InMachineName, const FGuid& InPollRequest, int32 InLiveLinkVersion);
};
