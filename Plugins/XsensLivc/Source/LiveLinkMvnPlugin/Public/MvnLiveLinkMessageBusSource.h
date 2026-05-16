// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include <LiveLinkMessageBusSource.h>
#include "LiveLinkMvnSource.h"

class LIVELINKMVNPLUGIN_API FMVNLiveLinkMessageBusSource : public FLiveLinkMvnSource
{
public:
	FMVNLiveLinkMessageBusSource(const FText& InSourceType, const FText& InSourceMachineName, const FMessageAddress& InConnectionAddress, double InMachineTimeOffset);
	virtual void ReceiveClient(ILiveLinkClient* InClient, FGuid InSourceGuid) override;

protected:

	virtual const FName& GetSourceName() const override { static FName Name(TEXT("MVNLiveLinkMessageBusSource")); return Name; }
	virtual void InitializeMessageEndpoint(FMessageEndpointBuilder& EndpointBuilder) override;
	void InitializeAndPushStaticData_AnyThread(FName SubjectName, TSubclassOf<ULiveLinkRole> SubjectRole, const FLiveLinkSubjectKey& SubjectKey, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context, UScriptStruct* MessageTypeInfo);

};