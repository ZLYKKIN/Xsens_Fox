// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include <LiveLinkProvider.h>
#include <Common/UdpSocketReceiver.h>
#include "ParserManager.h"

struct MvnVector
{
	double x;
	double y;
	double z;
};


class XSENSLLWRAPPER_API MvnWrapper : public FRunnable
{

private:
	const int32 port = 9763;
	const FTimespan WaitTime = FTimespan::FromSeconds(1);

	static MvnWrapper* s_MvnWrapperPtr;
	static FRunnableThread* s_Thread;

	FGuid PingId;
	bool isRunning = false;
	FSocket* Socket = nullptr;
	ISocketSubsystem* SocketSubsystem;
	TSharedPtr<FMessageEndpoint, ESPMode::ThreadSafe> MessageEndpoint;
	std::unique_ptr<ParserManager> Parser;
	
	TArray<FGuid> MvnLiveLinkSources;
	TMap<FString, TSharedPtr<FInternetAddr>> SubjectCache;
	TMap<FString, FDateTime> SubjectLastSeen;

	FString DecipherSubjectName(const FArrayReaderPtr& ArrayReaderPtr);
	void HandlePingMessage(const struct FMvnLiveLinkPingMessage& Message, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context);
	void HandleConnectionStatusChanged(const struct FLiveLinkConnectMessage& Message, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context);

	void StopListening();
	void StartListening();

	void BroadcastSubject();
	void RefreshCache();

public:
	MvnWrapper();
	static void Start();
	static void ShutDown();

	void AddSourceEntry(const FString& SubjectName, TSharedPtr<FInternetAddr> InternetAddress);

	void OnConnectionStatusChanged();

	bool Init();

	TSharedPtr<ILiveLinkProvider> LiveLinkProvider;
	FDelegateHandle ConnectionStatusChangedHandle;

	uint32 Run() override;

	void Stop() override;

	void Exit() override;
};


