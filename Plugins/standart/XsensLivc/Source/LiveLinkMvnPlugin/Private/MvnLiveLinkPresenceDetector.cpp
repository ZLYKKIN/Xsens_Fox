// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnLiveLinkPresenceDetector.h"
#include "MessageEndpointBuilder.h"
#include <LiveLinkSettings.h>
#include <ILiveLinkClient.h>
#include "MvnLiveLinkMessages.h"


FMVNLiveLinkPresenceDetector::FMVNLiveLinkPresenceDetector()
	: bIsRunning(true)
	, Thread(nullptr)
	, NumberRequests(0)
{
	PingRequestFrequency = GetDefault<ULiveLinkSettings>()->GetMessageBusPingRequestFrequency();

	MessageEndpoint = FMessageEndpoint::Builder(TEXT("MVNLiveLinkPresenceDetector"))
		.Handling<FMvnLiveLinkPongMessage>(this, &FMVNLiveLinkPresenceDetector::HandlePongMessage);


	bIsRunning = MessageEndpoint.IsValid();
	if (bIsRunning)
	{
		MessageEndpoint->Subscribe<FMvnLiveLinkPongMessage>();
		// Create the runnable thread
		Thread = FRunnableThread::Create(this, TEXT("MVNLinkPresenceDetector"));
	}
}

FMVNLiveLinkPresenceDetector::~FMVNLiveLinkPresenceDetector()
{
	{
		FScopeLock Lock(&CriticalSection);

		if (MessageEndpoint)
		{
			MessageEndpoint->Disable();
			MessageEndpoint.Reset();
		}
	}

	Stop();

	if (Thread)
	{
		Thread->Kill(true);
		Thread = nullptr;
	}
}

uint32 FMVNLiveLinkPresenceDetector::Run()
{
	while (bIsRunning)
	{
		{
			FScopeLock Lock(&CriticalSection);

			if (NumberRequests > 0)
			{
				PollResults.Reset();
				PingId = FGuid::NewGuid();

				MessageEndpoint->Publish(FMessageEndpoint::MakeMessage<FMvnLiveLinkPingMessage>(PingId, ILiveLinkClient::LIVELINK_VERSION));
			}
		}

		// Wait before sending another message
		FPlatformProcess::Sleep(PingRequestFrequency);
	}
	return 0;
}

void FMVNLiveLinkPresenceDetector::Stop()
{
	bIsRunning = false;
}

void FMVNLiveLinkPresenceDetector::AddPresenceRequest()
{
	FScopeLock Lock(&CriticalSection);
	if (++NumberRequests == 1)
	{
		PollResults.Reset();
	}
}

void FMVNLiveLinkPresenceDetector::RemovePresenceRequest()
{
	--NumberRequests;
}

TSharedPtr<FMessageEndpoint, ESPMode::ThreadSafe> FMVNLiveLinkPresenceDetector::GetMessageEndpoint()
{
	return MessageEndpoint;
}

void FMVNLiveLinkPresenceDetector::GetResults(TArray<FProviderPollResultPtr>& Results)
{
	FScopeLock Lock(&CriticalSection);
	Results = PollResults;
}

void FMVNLiveLinkPresenceDetector::HandlePongMessage(const struct FMvnLiveLinkPongMessage& Message, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context)
{
	FScopeLock Lock(&CriticalSection);

	if (Message.PollRequest == PingId)
	{
		PollResults.Emplace(MakeShared<FProviderPollResult, ESPMode::ThreadSafe>(Context->GetSender(),
			Message.ProviderName,
			Message.MachineName,
			0,
			true));
	}
}
