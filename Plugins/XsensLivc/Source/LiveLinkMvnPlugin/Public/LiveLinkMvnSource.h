// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "ILiveLinkSource.h"

#include "Roles/LiveLinkAnimationTypes.h"
#include "MessageEndpoint.h"
#include "IMessageContext.h"
#include "HAL/ThreadSafeBool.h"
#include "Common/UdpSocketReceiver.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include <memory>
//custom
#include "ParserManager.h"
#include "ScaleDatagram.h"
#include "SegmentIndices.h"
#include "MetaDatagram.h"
#include "QuaternionDatagram.h"
#include "HAL/CriticalSection.h"
#include <LiveLinkMessageBusSource.h>

#include "LiveLinkMvnSource.generated.h"

class ILiveLinkClient;
struct FLiveLinkPongMessage;
struct FLiveLinkSubjectDataMessage;
struct FLiveLinkSubjectFrameMessage;
struct FLiveLinkHeartbeatMessage;
struct FLiveLinkClearSubject;
struct FReferenceSkeleton;

#define XS_MAX_ACTORS_NUM 5

typedef TArray<FVector> MvnTPose;

// basic data for a segment
struct SegData
{
	// transform holds position, rotation and scale of this segment (local space)
	FTransform Transform;

	// convert data from mvn default formated datagram
	void Set(QuaternionDatagram::Kinematics &data, int segmentIndex/*, SegData const* parent*/, FVector pelvisPos = FVector::OneVector)
	{
		//Rotate from MVN X-YZ to Unreal coordinate
		FVector pos = FVector(data.sensorPos[0] * 100, -data.sensorPos[1] * 100, data.sensorPos[2] * 100);
		FQuat rot = FQuat(-data.quatRotation[1], data.quatRotation[2], -data.quatRotation[3], data.quatRotation[0]).GetNormalized();

		Transform.SetLocation(pos);
		Transform.SetRotation(rot);
		Transform.SetScale3D(pelvisPos);
	}
};

// struct holding a model pose data including it's position, orientation and segments data
struct Pose
{
	int AvatarId;
	int FrameId;

	Pose()
	{
		AvatarId = -1;
		FrameId = -1;
	}
	std::vector<SegData> Segments;
};


/** VirtualSubjectSource Settings to be able to differentiate from live sources and keep a name associated to the source */
UCLASS()
class ULiveLinkMvnSourceSettings : public ULiveLinkSourceSettings
{
	GENERATED_BODY()

public:

	UPROPERTY( VisibleAnywhere, AdvancedDisplay, Category = "Settings")
	int PortNumber;
};


class LIVELINKMVNPLUGIN_API FLiveLinkMvnSource : public FLiveLinkMessageBusSource, public FRunnable
{
public:

	
	FLiveLinkMvnSource(const FText& InSourceType, const FText& InSourceMachineName, const FMessageAddress& InConnectionAddress, double InMachineTimeOffset);
	virtual ~FLiveLinkMvnSource();
	virtual void InitializeSettings( ULiveLinkSourceSettings* Settings ) override;
	virtual TSubclassOf<ULiveLinkSourceSettings> GetSettingsClass() const override;
	virtual void OnSettingsChanged( ULiveLinkSourceSettings* Settings, const FPropertyChangedEvent& PropertyChangedEvent );
	virtual void ReceiveClient(ILiveLinkClient* InClient, FGuid InSourceGuid) override;

	virtual double GetDeadSourceTimeout() const;

	virtual bool IsSourceStillValid() const override;

	virtual bool RequestSourceShutdown() override;

	virtual FText GetSourceType() const override
	{
		return FText::FromString(TEXT("MVN Live"));
	}
	virtual FText GetSourceMachineName() const override { return SourceMachineName; }
	virtual FText GetSourceStatus() const override { return SourceStatus; }

	virtual bool Init() override { return true; }
	virtual uint32 Run() override;
	void Start();
	virtual void Stop() override;
	virtual void Exit() override { }
	void Recv(const FArrayReaderPtr& ArrayReaderPtr, const FIPv4Endpoint& EndPt);
	void Send(int avatarId);

	FName GetSubjectName(int AvatarId, int PortNum) const;

private:

	ILiveLinkClient* Client = nullptr;

	// Our identifier in LiveLink
	FGuid SourceGuid;


	FText SourceType;
	FText SourceMachineName;
	FText SourceStatus;

	UPROPERTY()
	int port;

	/** Holds the network socket. */
	FSocket* Socket;

	/** Holds a pointer to the socket sub-system. */
	ISocketSubsystem* SocketSubsystem;

	/** Holds a flag indicating that the thread is stopping. */
	bool Stopping;

	/** Holds the thread object. */
	FRunnableThread* Thread;

	/** The receiver thread's name. */
	FString ThreadName;

	/** Holds the amount of time to wait for inbound packets. */
	FTimespan WaitTime;

	// frame counter for data
	int FrameCounter;

	// A copy of each xsens avatar's pose from the last frame, for easy and safe access
	TArray<TSharedPtr<Pose, ESPMode::ThreadSafe>> m_lastPoses;
	FCriticalSection m_critical;

	std::unique_ptr<ParserManager> Parser;

	// Clear all subjects created by this source
	void ClearSubject();
	void updateRefSkeleton(int avatarId, int numOfSegments);
	void DeleteDelayedSubjects();

	int32 CurrentSkeletonSegmentCount[XS_MAX_ACTORS_NUM];
	MvnTPose m_atposesPos[XS_MAX_ACTORS_NUM];

	TMap<int32, FString> m_AvatarNames;
	TArray<FLiveLinkSubjectKey> m_SubjectsToDelete;
};
