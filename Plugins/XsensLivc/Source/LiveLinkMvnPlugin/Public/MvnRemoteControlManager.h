// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "MessageEndpoint.h"
#include "IMessageContext.h"
#include <memory>
#include "HAL/CriticalSection.h"

class MvnRemoteControlSession;
struct FIPv4Address;

#include "MvnRemoteControlManager.generated.h"

UCLASS()
class LIVELINKMVNPLUGIN_API UMvnRemoteControlManager : public UObject
{
    GENERATED_BODY()

public:


	/** start recording for all sessions */
	UFUNCTION( BlueprintCallable, Category = "Xsens Mvn Remote Control" )
	static void StartRecording( float _fTimeOffsetInSeconds );

	/** stop recording for all sessions */
	UFUNCTION( BlueprintCallable, Category = "Xsens Mvn Remote Control" )
	static void StopRecording( float _fTimeOffsetInSeconds );

};

class LIVELINKMVNPLUGIN_API MvnRemoteControlManager
{
public:

    /** constructor */
    MvnRemoteControlManager();

    /** destructor */
    virtual ~MvnRemoteControlManager();

    /** checks if there is already a session on same ip and port*/
    bool CanCreateSession( const FIPv4Address& _ipAddress, int _iPort );

    /// create a new session, ip and port required
    MvnRemoteControlSession*  CreateSession( const FIPv4Address& _ipAddress, int _iPort );

    void DestroySession( MvnRemoteControlSession* _pSession );

    /** send start recording for all sessions */
    void CaptureStartAll( double _dTimeOffsetInSeconds );

    /** send stop recording for all sessions */
    void CaptureStopAll( double _dTimeOffsetInSeconds );

    /** get all sessions */
    const TArray< MvnRemoteControlSession* >& GetSessions();

    /** time sync enabled / disabled on all sessions */
    void SetTimeSyncEnabled( bool _bTimeSyncEnabled );

    /** add a reserved port */
    void AddReservedPort( int32 _iPort );

    /** remove a reserved port */
    void RemoveReservedPort( int32 _iPort );

    /** check if a port is reserved */
    bool IsPortReserved( int32 _iPort );

    /** get static instance */
    static MvnRemoteControlManager* GetInstance();

	/** return true if any session is live capturing */
    bool IsAnyRecording();


private:

    /** sessions */
    TArray< MvnRemoteControlSession* > Sessions;

    /** track reserved ports here */
    TArray< int32 > m_aReservedPorts;

    /** static instance */
    static MvnRemoteControlManager* s_pInstance;

};
