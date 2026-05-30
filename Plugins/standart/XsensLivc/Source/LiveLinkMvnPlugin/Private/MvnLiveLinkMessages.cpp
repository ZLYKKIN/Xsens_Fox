// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnLiveLinkMessages.h"

FMvnLiveLinkPingMessage::FMvnLiveLinkPingMessage(const FGuid& InPollRequest, int32 InLiveLinkVersion)
	: FLiveLinkPingMessage(InPollRequest, InLiveLinkVersion)
{
}

FMvnLiveLinkPingMessage::FMvnLiveLinkPingMessage()
{

}

FMvnLiveLinkPongMessage::FMvnLiveLinkPongMessage(const FString& InProviderName, const FString& InMachineName, const FGuid& InPollRequest, int32 InLiveLinkVersion)
	: FLiveLinkPongMessage(InProviderName, InMachineName, InPollRequest, InLiveLinkVersion)
{
}

FMvnLiveLinkPongMessage::FMvnLiveLinkPongMessage()
{

}
