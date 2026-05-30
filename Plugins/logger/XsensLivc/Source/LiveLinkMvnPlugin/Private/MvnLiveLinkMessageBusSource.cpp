// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnLiveLinkMessageBusSource.h"
#include "LiveLinkMessages.h"
#include "LiveLinkMvnPlugin.h"
#include "MvnLiveLinkPresenceDetector.h"
#include "FoxLog.h"

FMVNLiveLinkMessageBusSource::FMVNLiveLinkMessageBusSource(const FText& InSourceType, const FText& InSourceMachineName, const FMessageAddress& InConnectionAddress, double InMachineTimeOffset)
	: FLiveLinkMvnSource(InSourceType, InSourceMachineName, InConnectionAddress, InMachineTimeOffset)
{
}

void FMVNLiveLinkMessageBusSource::ReceiveClient(ILiveLinkClient* InClient, FGuid InSourceGuid)
{
	FLiveLinkMvnSource::ReceiveClient(InClient, InSourceGuid);
	FFoxLog::Get().Log(TEXT("boot"), TEXT("message-bus client attached; presence published"));
	if (FLiveLinkMvnPluginModule::Get().GetPresenceDetector().GetMessageEndpoint().IsValid())
	{
		FLiveLinkMvnPluginModule::Get().GetPresenceDetector().GetMessageEndpoint()->Publish<FLiveLinkConnectMessage>(FMessageEndpoint::MakeMessage<FLiveLinkConnectMessage>());
	}
}

void FMVNLiveLinkMessageBusSource::InitializeMessageEndpoint(FMessageEndpointBuilder& EndpointBuilder)
{
	FLiveLinkMvnSource::InitializeMessageEndpoint(EndpointBuilder);
}

void FMVNLiveLinkMessageBusSource::InitializeAndPushStaticData_AnyThread(FName SubjectName,	TSubclassOf<ULiveLinkRole> SubjectRole,	const FLiveLinkSubjectKey& SubjectKey, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context, UScriptStruct* MessageTypeInfo)
{
	const FLiveLinkBaseStaticData* Message = reinterpret_cast<const FLiveLinkBaseStaticData*>(Context->GetMessage());

	FLiveLinkMvnSource::InitializeAndPushStaticData_AnyThread(SubjectName, SubjectRole, SubjectKey, Context, MessageTypeInfo);
}