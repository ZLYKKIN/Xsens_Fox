// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Templates/Atomic.h"
#include "HAL/ThreadSafeBool.h"
#include "LiveLinkMessageBusFinder.h"
#include <LiveLinkMessages.h>


class LIVELINKMVNPLUGIN_API FMVNLiveLinkPresenceDetector : public FRunnable
{
public:
	FMVNLiveLinkPresenceDetector();
	virtual ~FMVNLiveLinkPresenceDetector();
	bool IsRunning() const { return bIsRunning; }
	void GetResults(TArray<FProviderPollResultPtr>& Results);

	virtual void Stop() override;
	virtual uint32 Run() override;

	void AddPresenceRequest();
	void RemovePresenceRequest();
	TSharedPtr<FMessageEndpoint, ESPMode::ThreadSafe> GetMessageEndpoint();

private:

	void HandlePongMessage(const struct FMvnLiveLinkPongMessage& Message, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context);

private:
	FThreadSafeBool bIsRunning;
	double PingRequestFrequency;
	FGuid PingId;

	TSharedPtr<FMessageEndpoint, ESPMode::ThreadSafe> MessageEndpoint;
	class FRunnableThread* Thread;

	FCriticalSection CriticalSection;

	TArray<FProviderPollResultPtr> PollResults;
	TAtomic<int32> NumberRequests;
};