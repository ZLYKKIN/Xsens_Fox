// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnSource.h"
#include "LiveLinkMessages.h"
#include "LiveLinkMvnSettings.h"
#include "ILiveLinkClient.h"
#include "Async/Async.h"
#include "SegmentInformation.h"
#include "Roles/LiveLinkAnimationRole.h"

#include "Features/IModularFeatures.h"
#include "LiveLinkClient.h"
#include <Misc/EngineVersionComparison.h>
#include "MvnRemoteControlManager.h"
#include "LiveLinkMvnMetadataService.h"
#include "LiveLinkSourceSettings.h"
#include "LiveLinkMvnSourceFactory.h"


#pragma optimize("", off)



// ctor setup the socket for the given port and start the listener thread if isRunning is true

FLiveLinkMvnSource::FLiveLinkMvnSource(const FText& InSourceType, const FText& InSourceMachineName, const FMessageAddress& InConnectionAddress, double InMachineTimeOffset)
 :  FLiveLinkMessageBusSource(InSourceType, InSourceMachineName, InConnectionAddress, InMachineTimeOffset)
{
	FMemory::Memset(CurrentSkeletonSegmentCount, 0, sizeof(CurrentSkeletonSegmentCount));

	// Size our LastPoses array
	m_lastPoses.SetNum(XS_MAX_ACTORS_NUM);
	port = 9763;
	Stopping = false;
	WaitTime = FTimespan::FromMilliseconds(100);
}

FLiveLinkMvnSource::~FLiveLinkMvnSource()
{
	UE_LOG(LogTemp, Warning, TEXT("FLiveLinkMvnSource dtor"));

	Stop();
	if (Thread != nullptr)
	{
		//todo fix livelink
		Thread->WaitForCompletion();
		delete Thread;
		Thread = nullptr;
	}

	if (Socket != nullptr)
	{
		Socket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
	}

	if ( MvnRemoteControlManager::GetInstance() )
	{
		MvnRemoteControlManager::GetInstance()->RemoveReservedPort( port );
	}
	//remove client
}

FName FLiveLinkMvnSource::GetSubjectName(int AvatarId, int PortNum) const
{
	FString AvatarName = FString("");
	if (m_AvatarNames.Contains(AvatarId))
	{
		AvatarName = m_AvatarNames[AvatarId];
	}
	if (AvatarName.IsEmpty())
	{
		AvatarName = FString::FromInt(1 + AvatarId);
	}
	return FName(*(FString::FromInt(PortNum) + FString("-") + AvatarName));
}

void FLiveLinkMvnSource::ReceiveClient(ILiveLinkClient* InClient, FGuid InSourceGuid)
{
	ensureMsgf(InClient != nullptr, TEXT("Client shouldn't be null"));

	Client = InClient;
	SourceGuid = InSourceGuid;
	FLiveLinkMessageBusSource::ReceiveClient(InClient, InSourceGuid);
}

double FLiveLinkMvnSource::GetDeadSourceTimeout() const
{
	return GetDefault<ULiveLinkMvnSettings>()->GetMessageBusTimeBeforeRemovingDeadSourceOverride();
}

bool FLiveLinkMvnSource::IsSourceStillValid() const
{
	// Source is valid if we have a valid thread and socket
	bool bIsSourceValid = Thread != nullptr && Socket != nullptr;
	return bIsSourceValid;
}

bool FLiveLinkMvnSource::RequestSourceShutdown()
{
	Stop();

	return true;
}

// Release client, clear subjects
void FLiveLinkMvnSource::ClearSubject()
{
	//release client
	for (int avatarId=0; avatarId < XS_MAX_ACTORS_NUM; ++avatarId)
	{
		m_critical.Lock();
		if (m_lastPoses[avatarId].IsValid())
		{
			m_critical.Unlock();
			FLiveLinkSubjectKey SubjectKey;
			SubjectKey.Source = SourceGuid;
			SubjectKey.SubjectName = GetSubjectName(avatarId, port);
			Client->RemoveSubject_AnyThread(SubjectKey);
		}
		else
			m_critical.Unlock();
	}
}

void FLiveLinkMvnSource::updateRefSkeleton(int avatarId, int numOfSegments)
{
	const int32 RequiredBoneCount = numOfSegments + 1;
	if (CurrentSkeletonSegmentCount[avatarId] != RequiredBoneCount && SourceGuid.IsValid())
	{
		FLiveLinkStaticDataStruct StaticData(FLiveLinkSkeletonStaticData::StaticStruct());;
		FLiveLinkSkeletonStaticData& NewSkeleton = *StaticData.Cast<FLiveLinkSkeletonStaticData>();
		NewSkeleton.BoneNames.Reserve(RequiredBoneCount);
		NewSkeleton.BoneParents.Reserve(RequiredBoneCount);
		for (int32 i = 0; i < RequiredBoneCount; ++i)
		{
			NewSkeleton.BoneNames.Add(SegmentInformation::SegmentBoneNames[i]);
			NewSkeleton.BoneParents.Add(SegmentInformation::parentIndex[i]);
		}

		CurrentSkeletonSegmentCount[avatarId] = RequiredBoneCount;

		const FName SubjectName = GetSubjectName(avatarId, port);
		if (SourceGuid.IsValid() && m_AvatarNames.Contains(avatarId))
		{
			FLiveLinkSubjectKey SubjectKey(SourceGuid, SubjectName);
			// fix for duplicated sources issue XUU-90
			//if ( Client->GetSubjectSettings( SubjectKey ) == nullptr )
			{
				Client->PushSubjectStaticData_AnyThread( SubjectKey, ULiveLinkAnimationRole::StaticClass(), MoveTemp( StaticData ) );
			}
			//else
			{
				// fix for duplicated sources issue XUU-90
				//UE_LOG( LogTemp, Warning, TEXT( "detected reading source %s " ), *SubjectKey.SubjectName.ToString() );
			}
		}
	}
}

// FRunnable interface
void FLiveLinkMvnSource::Start()
{
	ThreadName = "MVN UDP Receiver ";
	ThreadName.AppendInt(FAsyncThreadIndex::GetNext());

	Thread = FRunnableThread::Create(this, *ThreadName, 128 * 1024, TPri_AboveNormal, FPlatformAffinity::GetPoolThreadMask());
}

void FLiveLinkMvnSource::Stop()
{
	Stopping = true;
}

uint32 FLiveLinkMvnSource::Run()
{
	ensureMsgf(Client != nullptr, TEXT("Client should be initialized before running source"));

	TSharedRef<FInternetAddr> Sender = SocketSubsystem->CreateInternetAddr();

	while (!Stopping)
	{
		if (!Socket->Wait(ESocketWaitConditions::WaitForRead, WaitTime))
		{
			continue;
		}

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
				
				Recv(Reader, FIPv4Endpoint(Sender));
			}
		}
	}
	return 0;
}

// Receiver thread runs until Stop() is called or source removed
void FLiveLinkMvnSource::Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt)
{
	TArray<uint8_t> RawData;
	uint32 DataReceivedNum = ArrayReaderPtr->Num();

	RawData.Append(ArrayReaderPtr->GetData(), DataReceivedNum);

	if (RawData.Num() > 0)
	{
		std::unique_ptr<Datagram> d(Parser->readDatagram(RawData));

		if (d != nullptr)
		{
			unsigned int avatarId = d->avatarId();
			unsigned int segmentCount = d->bodySegmentCount();
			unsigned int propCount = d->propCount();
			unsigned int fingerSegCount = d->fingerTrackingSegmentCount();

			StreamingProtocol proto = (StreamingProtocol)d->messageType();
			MvnTPose& tPose = m_atposesPos[avatarId];
			if (proto == StreamingProtocol::SPPoseQuaternion)
			{
				TSharedPtr<Pose, ESPMode::ThreadSafe> NewPose(new Pose());

				QuaternionDatagram *q = static_cast<QuaternionDatagram *>(d.get());

				bool haveFingers = (fingerSegCount > 0);
				int firstFinger = (haveFingers ? SegmentIndexes::Prop1 + propCount : q->m_data.size());

				if (haveFingers)
					NewPose->Segments.resize(SegmentIndexes::XS_SEG_NUM_FINGERS);
				else
					NewPose->Segments.resize(q->m_data.size());

				NewPose->AvatarId = avatarId;
				NewPose->FrameId = FrameCounter++;

				for (int s = 0; s < (int)q->m_data.size(); s++)
				{
					int segmentId = q->m_data[s].segmentId;
					int segmentIndex = segmentId - 1;
					SegData* seg = &NewPose->Segments[segmentIndex];

					if (s >= firstFinger)
					{
						if (s == firstFinger)
						{
							// fill in missing props
							QuaternionDatagram::Kinematics kin = QuaternionDatagram::Kinematics();
							for (int t = s; t < SegmentIndexes::XS_SEG_NUM; ++t)
								NewPose->Segments[segmentIndex].Set(kin, t);
						}
						segmentIndex = s + SegmentIndexes::XS_SEG_NUM - firstFinger;
						seg = &NewPose->Segments[segmentIndex];
					}

					QuaternionDatagram::Kinematics kin = q->m_data[s];
					if (tPose.Num() > s)
						seg->Set(kin, segmentIndex, tPose[s]);
					else
						seg->Set(kin, segmentIndex);
				}

				FLiveLinkSubjectName SubjectName = FLiveLinkSubjectName(GetSubjectName(avatarId, port));
				FLiveLinkMvnMetadataService::getInstance().SetSegmentCount(SourceGuid, SubjectName, NewPose->Segments.size());

				updateRefSkeleton(avatarId, NewPose->Segments.size());
				m_critical.Lock();
				// Pointer swap
				m_lastPoses[avatarId] = NewPose;
				m_critical.Unlock();
			}
			else if (proto == SPMetaScaling)
			{
				ScaleDatagram *q = static_cast<ScaleDatagram *>(d.get());
				if (!q->m_data.empty())
				{
					TArray<FString> segmentNames;

					if (tPose.Num() != q->m_data.size())
					{
						tPose.SetNum((int)q->m_data.size());
					}
					for (int s = 0; s < (int)q->m_data.size(); s++)
					{
						tPose[s] = FVector(q->m_data[s].segmentOriginPos[0] * 100, -q->m_data[s].segmentOriginPos[1] * 100, q->m_data[s].segmentOriginPos[2] * 100);
						segmentNames.Add(FString(q->m_data[s].segmentName.c_str()));
					}

					FLiveLinkSubjectName SubjectName = FLiveLinkSubjectName(GetSubjectName(avatarId, port));
					FLiveLinkMvnMetadataService::getInstance().SetSegmentNames(SourceGuid, SubjectName, segmentNames);
				}
			}
			else if (proto == SPMetaMoreMeta)
			{
				MetaDatagram* q = static_cast<MetaDatagram*>(d.get());
				if (!q->m_Name.empty())
				{
					bool IsNewSubject = false;
					FString NewSubjectName = FString(q->m_Name.c_str());
					if (!m_AvatarNames.Contains(avatarId))
					{
						IsNewSubject = true;
					}
					else
					{
						IsNewSubject = m_AvatarNames[avatarId] != NewSubjectName;
					}
					if (!Stopping && IsNewSubject)
					{
						FLiveLinkSubjectKey SubjectKey;
						SubjectKey.Source = SourceGuid;
						SubjectKey.SubjectName = GetSubjectName(avatarId, port);
						if (Client->GetSubjectSettings(SubjectKey) != nullptr)
						{
							Client->RemoveSubject_AnyThread(SubjectKey);
						}
						else
						{
							m_SubjectsToDelete.Add(SubjectKey);
						}
						CurrentSkeletonSegmentCount[avatarId] = 0; // to force subject update

						m_AvatarNames.Add(avatarId, NewSubjectName);
					}
				}
			}
			
			DeleteDelayedSubjects();
			AsyncTask(ENamedThreads::GameThread, [this, avatarId]() { Send(avatarId); });
		}
	}
}

void FLiveLinkMvnSource::Send(int avatarId)
{
	// Early exit if no Client or not running
	if (Stopping || Client == nullptr)
		return;

	//build up Subject data
	const FName SubjectName = GetSubjectName(avatarId, port);
	FLiveLinkSubjectKey SubjectKey(SourceGuid, SubjectName);

	FLiveLinkFrameDataStruct FrameData(FLiveLinkAnimationFrameData::StaticStruct());
	FLiveLinkAnimationFrameData& AnimFrameData = *FrameData.Cast<FLiveLinkAnimationFrameData>();
	TArray<FTransform>& Transforms = AnimFrameData.Transforms;

	// Reserve space for all segments + root
	Transforms.Reserve(1 + SegmentIndexes::XS_SEG_NUM);

	// add Root first
	Transforms.Add(FTransform::Identity);

	m_critical.Lock();
	TSharedPtr<Pose, ESPMode::ThreadSafe> LatestPose = m_lastPoses[avatarId];
	m_critical.Unlock();

	if (!LatestPose.IsValid())
		return;

	for(auto & Segment : LatestPose->Segments)
	{
		SegData* seg = &Segment;
		Transforms.Add(seg->Transform);
	}

	//direct communication with client
	// if subject is missing, force its static data update (can happen after multiple source sending to the same port)
	if (Client->GetSubjectSettings(SubjectKey) == nullptr)
	{
		CurrentSkeletonSegmentCount[avatarId] = 0;
	}
	Client->PushSubjectFrameData_AnyThread(SubjectKey, MoveTemp(FrameData));
}

void FLiveLinkMvnSource::DeleteDelayedSubjects()
{
	if (Stopping)
		return;

	for (int32 i = 0; i < m_SubjectsToDelete.Num(); ++i)
	{
		FLiveLinkSubjectKey SubjectKey = m_SubjectsToDelete[i];
		if (Client->GetSubjectSettings(SubjectKey) != nullptr) {
			Client->RemoveSubject_AnyThread(SubjectKey);
			#if UE_VERSION_OLDER_THAN(5, 5, 0)
				m_SubjectsToDelete.RemoveAt(i, 1, true);
			#else 
				m_SubjectsToDelete.RemoveAt(i, 1, EAllowShrinking::Yes);
			#endif
			--i;
		}
	}
}

void FLiveLinkMvnSource::InitializeSettings( ULiveLinkSourceSettings* Settings )
{
	if ( Settings )
	{
		ULiveLinkMvnSourceSettings* pSet  = Cast<ULiveLinkMvnSourceSettings>( Settings );
		pSet->Factory = ULiveLinkMvnSourceFactory::StaticClass();

		if ( pSet->PortNumber != port )
		{
			if ( pSet->PortNumber != 0 )
			{
				port = pSet->PortNumber;
			}
		}

		pSet->PortNumber = port;

		if ( port != 0 )
		{
			//setup socket
			FIPv4Endpoint Endpoint( FIPv4Address::Any, port );
			int32 BufferSize = 1024 * 1024;

			Socket = FUdpSocketBuilder( TEXT( "MVNSOCKET" ) )
				.AsNonBlocking()
				.AsReusable()
				.BoundToEndpoint( Endpoint )
				.WithReceiveBufferSize( BufferSize );

			check( Socket != nullptr );
			check( Socket->GetSocketType() == SOCKTYPE_Datagram );

			SocketSubsystem = ISocketSubsystem::Get( PLATFORM_SOCKETSUBSYSTEM );
			if ( MvnRemoteControlManager::GetInstance() )
			{
				MvnRemoteControlManager::GetInstance()->AddReservedPort( port );
			}

			Start();
		}

	}
}


void FLiveLinkMvnSource::OnSettingsChanged( ULiveLinkSourceSettings* Settings, const FPropertyChangedEvent& PropertyChangedEvent )
{

}

TSubclassOf<ULiveLinkSourceSettings> FLiveLinkMvnSource::GetSettingsClass() const
{
	return ULiveLinkMvnSourceSettings::StaticClass();
}
#pragma optimize("", on)