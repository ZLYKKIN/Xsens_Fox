// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnWrapper.h"
#include <Interfaces/IPv4/IPv4Endpoint.h>
#include <Common/UdpSocketBuilder.h>
#include <Common/UdpSocketReceiver.h>
#include <Misc/EngineVersionComparison.h>
#include "Datagram.h"
#include "MetaDatagram.h"
#include <Roles/LiveLinkTransformRole.h>
#include <Roles/LiveLinkTransformTypes.h>
#include <../Plugins/Animation/LiveLink/Source/LiveLink/Public/LiveLinkClient.h>
#include "LiveLinkMvnSource.h"
#include <MessageEndpoint.h>
#include <LiveLinkMessages.h>
#include "MvnLiveLinkMessages.h"
#include "Features/IModularFeatures.h"
#include <ws2tcpip.h>

MvnWrapper* MvnWrapper::s_MvnWrapperPtr = nullptr;
FRunnableThread* MvnWrapper::s_Thread = nullptr;

MvnWrapper::MvnWrapper()
{
}

void MvnWrapper::Start()
{
	if (!s_MvnWrapperPtr)
		s_MvnWrapperPtr = new MvnWrapper();
	else
		return;


	// Create a thread and give it your FRunnable object
	s_Thread = FRunnableThread::Create(s_MvnWrapperPtr, TEXT("Wrapper Thread"));
}

bool MvnWrapper::Init()
{
	isRunning = true;

	StartListening();

	SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);

	/* For Passing to Live Link using Message Bus */
	MessageEndpoint = FMessageEndpoint::Builder(TEXT("MVNLiveLinkPresenceDetector"))
		.Handling<FMvnLiveLinkPingMessage>(this, &MvnWrapper::HandlePingMessage)
		.Handling<FLiveLinkConnectMessage>(this, &MvnWrapper::HandleConnectionStatusChanged);

	if (MessageEndpoint.IsValid())
	{
		MessageEndpoint->Subscribe<FMvnLiveLinkPingMessage>();
		MessageEndpoint->Subscribe<FLiveLinkConnectMessage>();
	}

	IModularFeatures& ModularFeatures = IModularFeatures::Get();
	if (ModularFeatures.IsModularFeatureAvailable(FLiveLinkClient::ModularFeatureName))
	{
		FLiveLinkClient* Client = &IModularFeatures::Get().GetModularFeature<FLiveLinkClient>(FLiveLinkClient::ModularFeatureName);

		Client->OnLiveLinkSourceAdded().AddLambda([this, Client](const FGuid& LiveSourceGuid)
		{
			FText SourceType = Client->GetSourceType(LiveSourceGuid);
			if (SourceType.ToString().Contains("MVN"))
				MvnLiveLinkSources.Add(LiveSourceGuid);
		});
		Client->OnLiveLinkSourceRemoved().AddLambda([this](const FGuid& LinkSourceGuid)
		{
			if (MvnLiveLinkSources.Contains(LinkSourceGuid))
			{
				MvnLiveLinkSources.Remove(LinkSourceGuid);
				StartListening();
			}
		});
	}

#ifdef CUSTOM_LIVELINK_PROVIDER
	/*In case custom Live Link provider is required*/
	if (LiveLinkProvider.IsValid())
	{
		FPlatformMisc::LowLevelOutputDebugString(TEXT("Live Link Provider already started!\n"));
		return true;
	}

	LiveLinkProvider = ILiveLinkProvider::CreateLiveLinkProvider(TEXT("Mvn Live Link"));
#endif

	return true;
}

void MvnWrapper::ShutDown()
{
	if (s_MvnWrapperPtr)
	{
		s_MvnWrapperPtr->Stop();
		
		s_Thread->WaitForCompletion();

		delete s_Thread;
		s_Thread = nullptr;

		delete s_MvnWrapperPtr;
		s_MvnWrapperPtr = nullptr;
	}
}


void MvnWrapper::HandlePingMessage(const struct FMvnLiveLinkPingMessage& Message, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context)
{
	PingId = Message.PollRequest;
}

uint32 MvnWrapper::Run()
{
	UE_LOG(LogTemp, Warning, TEXT("Begin Search"));

	TSharedRef<FInternetAddr> Sender = SocketSubsystem->CreateInternetAddr();


	while (isRunning)
	{
		uint32 Size;

		while (Socket->HasPendingData(Size))
		{
			FArrayReaderPtr Reader = MakeShareable(new FArrayReader(true));
			Reader->SetNumUninitialized(FMath::Min(Size, 65507u));

			int32 Read = 0;

			if (Socket->RecvFrom(Reader->GetData(), Reader->Num(), Read, *Sender))
			{
				#if UE_VERSION_OLDER_THAN(5, 5, 0)
					Reader->RemoveAt(Read, Reader->Num() - Read, false);
				#else 
					Reader->RemoveAt(Read, Reader->Num() - Read, EAllowShrinking::No);
				#endif
				
				FString SubjectName = DecipherSubjectName(Reader);

				if (!SubjectName.IsEmpty())
				{
					AddSourceEntry(SubjectName, Sender.ToSharedPtr());
				}
			}
		}

		RefreshCache();

		if (!SubjectCache.IsEmpty())
		{
			BroadcastSubject();
		}
	}
	
	UE_LOG(LogTemp, Warning, TEXT("End Search"));
	return -1;
}

FString MvnWrapper::DecipherSubjectName(const FArrayReaderPtr& ArrayReaderPtr)
{
	TArray<uint8_t> RawData;
	uint32 DataReceivedNum = ArrayReaderPtr->Num();

	RawData.Append(ArrayReaderPtr->GetData(), DataReceivedNum);

	if (RawData.Num() > 0)
	{
		std::unique_ptr<Datagram> d(Parser->readDatagram(RawData));

		if (d != nullptr)
		{
			StreamingProtocol proto = (StreamingProtocol)d->messageType();

			if (proto == SPMetaMoreMeta)
			{
				MetaDatagram* q = static_cast<MetaDatagram*>(d.get());
				if (!q->m_Name.empty())
				{
					bool IsNewSubject = false;
					FString NewSubjectName = FString(q->m_Name.c_str());

					return NewSubjectName;
				}
			}
		}
	}

	return "";
}

FString GetHostnameFromIP(TSharedPtr<FInternetAddr> InternetAddress)
{
	const char* ip_address = TCHAR_TO_UTF8(*(InternetAddress->ToString(false))); // Replace with the actual IP address

	struct sockaddr_in sa;
	sa.sin_family = AF_INET;
	inet_pton(AF_INET, ip_address, &sa.sin_addr);

	char host[NI_MAXHOST];

	int res = getnameinfo((struct sockaddr*)&sa, sizeof(sa), host, NI_MAXHOST, nullptr, 0, 0);
	if (res == 0) {
		return FString(host);
	}
	else {
		return "";
	}
}

void MvnWrapper::AddSourceEntry(const FString& SubjectName, TSharedPtr<FInternetAddr> InternetAddress)
{
	// Find if the key already exists in the map
	TSharedPtr<FInternetAddr>* ExistingAddress = SubjectCache.Find(SubjectName);

	// If the key exists, update the value
	if (ExistingAddress)
	{
		*ExistingAddress = InternetAddress;
	}
	// If the key doesn't exist, add a new key-value pair
	else
	{
		SubjectCache.Add(SubjectName, InternetAddress);
	}

	FDateTime* ExistingTime = SubjectLastSeen.Find(SubjectName);

	// If the key exists, update the value
	if (ExistingTime)
	{
		*ExistingTime = FDateTime::Now();
	}
	else
	{
		SubjectLastSeen.Add(SubjectName, FDateTime::Now());
	}

}

void MvnWrapper::HandleConnectionStatusChanged(const struct FLiveLinkConnectMessage& Message, const TSharedRef<IMessageContext, ESPMode::ThreadSafe>& Context)
{
	StopListening();
}

void MvnWrapper::StartListening()
{
	if (!isRunning)
		return;

	FIPv4Endpoint Endpoint(FIPv4Address::Any, port);
	int32 BufferSize = 1024 * 1024;

	/* Socket for Listening to UDP Message and Subject */
	Socket = FUdpSocketBuilder(TEXT("MVNSOCKET"))
		.AsNonBlocking()
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(BufferSize);
}

void MvnWrapper::StopListening()
{
	Socket->Close();
}

void MvnWrapper::BroadcastSubject()
{
	for (auto elem : SubjectCache)
	{
		FString HostName = GetHostnameFromIP(elem.Value);
		MessageEndpoint->Publish<FMvnLiveLinkPongMessage>(FMessageEndpoint::MakeMessage<FMvnLiveLinkPongMessage>("MVN Live", HostName, PingId, ILiveLinkClient::LIVELINK_VERSION));

#ifdef CUSTOM_LIVELINK_PROVIDER
		//To make Custom Entry 

		isRunning = false;
		Socket->Close();
		FName SubjectFName = *elem.Key;

		FLiveLinkStaticDataStruct StaticData(FLiveLinkBaseStaticData::StaticStruct());
		FLiveLinkBaseStaticData& BaseData = *StaticData.Cast<FLiveLinkBaseStaticData>();
		BaseData.PropertyNames.Add(SubjectFName);

		LiveLinkProvider->UpdateSubjectStaticData(SubjectFName, ULiveLinkBasicRole::StaticClass(), MoveTemp(StaticData));

		ConnectionStatusChangedHandle = LiveLinkProvider->RegisterConnStatusChangedHandle(FLiveLinkProviderConnectionStatusChanged::FDelegate::CreateRaw(this, &MvnWrapper::OnConnectionStatusChanged));

		FPlatformMisc::LowLevelOutputDebugString(TEXT("Live Link Provider started!\n"));
#endif
	}
}

void MvnWrapper::RefreshCache()
{

	for (auto It = SubjectLastSeen.CreateIterator(); It; ++It)
	{
		FTimespan TimeSinceLastSeen = FDateTime::Now() - It.Value();
		if (TimeSinceLastSeen >= WaitTime)
		{
			SubjectCache.Remove(It.Key());
			It.RemoveCurrent();
		}
	}
}

void MvnWrapper::OnConnectionStatusChanged()
{
#ifdef CUSTOM_LIVELINK_PROVIDER
	if (LiveLinkProvider.IsValid())
	{
		isRunning = false;
		Socket->Close();
		AsyncTask(ENamedThreads::GameThread, [this]() 
		{
			FLiveLinkClient* Client = &IModularFeatures::Get().GetModularFeature<FLiveLinkClient>(FLiveLinkClient::ModularFeatureName);
			Client->RemoveAllSources();
			Client->AddSource(MakeShared<FLiveLinkMvnSource>(port, true));
		});
	}
#endif
}

void MvnWrapper::Stop()
{
	StopListening();
	isRunning = false;
}

void MvnWrapper::Exit()
{
	StopListening();
	isRunning = false;
}