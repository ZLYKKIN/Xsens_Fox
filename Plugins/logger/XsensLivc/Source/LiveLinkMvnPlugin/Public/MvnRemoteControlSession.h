// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "MessageEndpoint.h"
#include "IMessageContext.h"
#include "HAL/ThreadSafeBool.h"
#include "Common/UdpSocketReceiver.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "HAL/CriticalSection.h"
#include "MvnRemoteControlMessage.h"


class LIVELINKMVNPLUGIN_API MvnRemoteControlSession : public FRunnable
{
public:

    /** constructor */
    MvnRemoteControlSession( const FIPv4Address& _ipAddress, const int _iPort );

    /** destructor */
    virtual ~MvnRemoteControlSession();
    
    /** Starts the thread */
    void Start();

    void Reset();

    /**  send/receives loop happens here */
    virtual uint32 Run() override;

    /** stop the loop */
    virtual void Stop() override;

    /** getter for session name */
    FString GetSessionName() const;

    /** Getter for take */
    FString GetTake() const;

    /** Getter for status */
    FString GetStatus() const;

    /** Getter for elapsed time */
    FString GetElapsedTime() const;

    /** Getter for IP Address and port */
    FString GetIPAddressAndPort() const;

    /** IP address getter */
    const FIPv4Address& GetDestIPAddress();

    /** identify command */
    void Identify();

    /** Set CapturenName and optional take*/
    void SetCaptureName( FString& _strCaptureName, int _iTakeCount = -1 );

    /** send capture command*/
    bool CaptureStart( double _dTimeOffsetInSeconds );

    /** send capture stop command*/
    bool CaptureStop( double _dTimeOffsetInSeconds );

    /** is recording */
    bool IsRecording();

    /** enable time sync */
    void SetTimeSyncEnabled( bool _bTimeSynEnabled );

    /** is time sync enabled */
    bool IsTimeSyncEnabled();

    /** change destination IP Address */
    void SetDestinationIPAddress( const FIPv4Address& _ipAddress );

    /** change destination port */
    void SetDestinationPort( int _iPort );

    /** getter for destination IP Address */
    const FIPv4Address& GetDestinationIPAddress();

    /** getter for destination port */
    int GetDestinationPort();

private:

    /** Holds the thread object. */
    FRunnableThread* m_pThread = nullptr;

    /** The receiver thread's name. */
    FString m_strThreadName;

    /** Holds the amount of time to wait for inbound packets. */
    FTimespan m_WaitTime;

    /** Holds the port. */
    int m_iDestPort;

    /** Holds the network socket. */
    FSocket* m_pSocket = nullptr;

    /** Holds a pointer to the socket sub-system. */
    ISocketSubsystem* m_pSocketSubsystem;

    /** holds the destination ip address */
    FIPv4Address m_DestIPAddress;

    /** Holds a flag indicating that the thread is stopping. */
    bool m_bStopping = false;

    /** an id */
    uint32 m_uiID = 0;

    /** session name*/
    FString  m_strSessionName;

    /** take name*/
    FString  m_strTake;

    /** IP and port */
    FString  m_strIPAddress;

    /** true if currently recording */
    bool m_bRecording = false;

    /** true if currently paused */
    bool m_bPaused = false;

    /** true for live sessions */
    bool m_bLive = false;

    /** toggle time sync messages */
    bool m_bTimeSyncEnabled = true;

    /** remains false until we get reply */
    bool m_bConnected = false;

    /** store last error/ failure reason as readable text */
    FString m_strLastError;

    /** time when recording started */
    FDateTime  m_StartRecordTime;

    /** last time session info was requested */
    FDateTime  m_LastSessionInfoRequestTime;

    /** last time session status was requested */
    FDateTime  m_LastSessionStatusRequestTime;

    /** last time a sync time was requested */
    FDateTime  m_LastSyncTimeRequestTime;

    /** Queue for commands, one is sent at a time */
    TArray< MvnRemoteControlMessage* > m_apCommandsQueue;

    /** critical section to protect commands queue */
    FCriticalSection m_CommandsCriticalSection;

    /**send a command/request array */
    bool SendCommand( MvnRemoteControlMessage* _pMessage, const FInternetAddr& Destination );

    /** Process a response */
    void ProcessResponse( FString& _strResponse );

    /** Queue a new remote control request message */
    void QueueRequest( MvnRemoteControlMessage* _pMessage );

    /** when CaptureNameAck is received */
    void OnCaptureNameAckReceived( MvnRemoteControlMessage* _pResponse );

    /** when CaptureStartAck is received */
    void OnCaptureStartAckReceived( MvnRemoteControlMessage* _pResponse );

    /** when CaptureStopAck is received */
    void OnCaptureStopAckReceived( MvnRemoteControlMessage* _pResponse );

    /** when SessionInfoAck is received */
    void OnSessionInfoAckReceived( MvnRemoteControlMessage* _pResponse );

    /** when SessionStatusAck is received */
    void OnSessionStatusAckReceived( MvnRemoteControlMessage* _pResponse );

    /** request session info*/
    void RequestSessionInfo();

    /** request session status*/
    void RequestSessionStatus();

    /** Request time sync */
    void RequestTimeSync();

    /** holds a serial id that is ever incrementing */
    static uint32 s_uiID;

};
