// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnRemoteControlSession.h"
#include "Async/Async.h"
#include <Serialization/ArrayWriter.h>
#include <Misc/EngineVersionComparison.h>
#include "MvnRemoteControlSessionUtil.h"

uint32 MvnRemoteControlSession::s_uiID = 0;


// ctor setup the socket for the given port and start the listener thread if isRunning is true
MvnRemoteControlSession::MvnRemoteControlSession( const FIPv4Address& _destIPAddress, int _iDestPort ) :
m_WaitTime( FTimespan::FromMilliseconds( 100 ) )
, m_iDestPort( _iDestPort )
, m_DestIPAddress( _destIPAddress )
{
    //setup socket
    // listener port has to be unique per instance
    int iListenerPort = 8008 + s_uiID;
    FIPv4Endpoint Endpoint( FIPv4Address::Any, iListenerPort );
    int32 BufferSize = 1024 * 1024;

    m_pSocket = FUdpSocketBuilder( TEXT( "MVNRCSOCKET" ) )
        .AsBlocking()
        .AsReusable()
        .WithBroadcast()
        .BoundToEndpoint( Endpoint )
        .WithReceiveBufferSize( BufferSize )
        .WithSendBufferSize( BufferSize );
        

    check( m_pSocket != nullptr );
    check( m_pSocket->GetSocketType() == SOCKTYPE_Datagram );

    
    m_pSocketSubsystem = ISocketSubsystem::Get( PLATFORM_SOCKETSUBSYSTEM );

    m_uiID = s_uiID;
    m_strTake = FString::Printf( TEXT( "001" ) );
    m_strIPAddress = FString::Printf( TEXT( "%d.%d.%d.%d:%d" ), _destIPAddress.A, _destIPAddress.B, _destIPAddress.C, _destIPAddress.D, _iDestPort );

    s_uiID++;

    RequestSessionInfo();
    Start();
}

MvnRemoteControlSession::~MvnRemoteControlSession()
{
    UE_LOG( LogTemp, Warning, TEXT( "MvnRemoteControlSession dtor" ) );
    if ( m_bRecording )
    {
        CaptureStop( 0.0 );
    }

    /// send all pending messages
    if ( m_CommandsCriticalSection.TryLock() )
    {
        TSharedRef<FInternetAddr> Destination = m_pSocketSubsystem->CreateInternetAddr();
        Destination->SetIp( m_DestIPAddress.Value );
        Destination->SetPort( m_iDestPort );

        if ( m_apCommandsQueue.Num() > 0 )
        {
            MvnRemoteControlMessage* pCommand = m_apCommandsQueue[ 0 ];
            SendCommand( pCommand, *Destination );
            m_apCommandsQueue.Remove( pCommand );
            delete pCommand;
        }
        m_CommandsCriticalSection.Unlock();
    }

    Stop();
    if ( m_pThread != nullptr )
    {
        //todo fix live link
        m_pThread->WaitForCompletion();
        delete m_pThread;
        m_pThread = nullptr;
    }

    if ( m_pSocket != nullptr )
    {
        m_pSocket->Close();
        ISocketSubsystem::Get( PLATFORM_SOCKETSUBSYSTEM )->DestroySocket( m_pSocket );
    }
}


// FRunnable interface
void MvnRemoteControlSession::Start()
{
    m_strThreadName = "MVN UDP Remote Control ";
    m_strThreadName.AppendInt( FAsyncThreadIndex::GetNext() );

    m_pThread = FRunnableThread::Create( this, *m_strThreadName, 128 * 1024, TPri_AboveNormal, FPlatformAffinity::GetPoolThreadMask() );
}

uint32 MvnRemoteControlSession::Run()
{
    TSharedRef<FInternetAddr> Sender = m_pSocketSubsystem->CreateInternetAddr();
    TSharedRef<FInternetAddr> Destination = m_pSocketSubsystem->CreateInternetAddr();
    Destination->SetIp( m_DestIPAddress.Value );
    Destination->SetPort( m_iDestPort );

    FDateTime dtStartTime = FDateTime::Now();
    FDateTime dtStartTimeOnTheSecond( dtStartTime.GetYear(), dtStartTime.GetMonth(), dtStartTime.GetDay(), dtStartTime.GetHour(), dtStartTime.GetMinute(), dtStartTime.GetSecond() );

    m_LastSessionInfoRequestTime   = dtStartTimeOnTheSecond;
    m_LastSessionStatusRequestTime = dtStartTimeOnTheSecond;
    m_LastSyncTimeRequestTime = dtStartTimeOnTheSecond;


    
    float fSessionInfoInterval = 1.0f;
    float fSessionStatusInterval = 0.1f;
    // less than one second to ensure we send exactly on the second
    float fTimeSyncInterval = 0.9f;

    while ( !m_bStopping )
    {
        ESocketConnectionState state =  m_pSocket->GetConnectionState();
        uint32 Size;

        uint32 uiActiveDestIP = 0;
        int32 iActiveDestPort = 0;
        Destination->GetIp( uiActiveDestIP );
        Destination->GetPort( iActiveDestPort );

        if ( iActiveDestPort != m_iDestPort )
        {
            Destination->SetPort( m_iDestPort );
        }

        if ( uiActiveDestIP != m_DestIPAddress.Value )
        {
            Destination->SetIp( m_DestIPAddress.Value );
        }

        Destination->SetIp( m_DestIPAddress.Value );
        Destination->SetPort( m_iDestPort );

        if ( m_CommandsCriticalSection.TryLock() )
        {
            if ( m_apCommandsQueue.Num() > 0 )
            {
                MvnRemoteControlMessage* pCommand = m_apCommandsQueue[ 0 ];
                SendCommand( pCommand, *Destination );
                m_apCommandsQueue.Remove(pCommand);
                delete pCommand;
            }
            m_CommandsCriticalSection.Unlock();
        }

        FDateTime dtNow = FDateTime::Now();
        FDateTime dtNowOnTheSecond( dtNow.GetYear(), dtNow.GetMonth(), dtNow.GetDay(), dtNow.GetHour(), dtNow.GetMinute(), dtNow.GetSecond() );

        FTimespan sessionInfoDiff   = dtNow - m_LastSessionInfoRequestTime;
        FTimespan sessionStatusDiff = dtNow - m_LastSessionStatusRequestTime;
        FTimespan syncTimeDiff = dtNowOnTheSecond - m_LastSyncTimeRequestTime;

        if ( m_bTimeSyncEnabled )
        {
            if ( syncTimeDiff.GetTotalSeconds() > fTimeSyncInterval )
            {
                FTimespan toSecond = dtNow - dtNowOnTheSecond;
                double dTimeDiffMS = toSecond.GetTotalMilliseconds();
                if ( dTimeDiffMS < 2.0 )
                {
                    // should send on the second
                    RequestTimeSync();
                }
                else
                {
                    UE_LOG( LogTemp, Warning, TEXT( "time out of sync by %f milliseconds, skipping sync" ), dTimeDiffMS );
                }
                m_LastSyncTimeRequestTime = dtNowOnTheSecond;
            }
        }

        if ( sessionInfoDiff.GetTotalSeconds() > fSessionInfoInterval )
        {
            m_LastSessionInfoRequestTime = dtNow;
            RequestSessionInfo();
        }

        if ( sessionStatusDiff.GetTotalSeconds() > fSessionStatusInterval )
        {
            m_LastSessionStatusRequestTime = dtNow;
            RequestSessionStatus();
        }

        while ( m_pSocket->HasPendingData( Size ) )
        {
            FArrayReaderPtr Reader = MakeShareable( new FArrayReader( true ) );
            Reader->SetNumUninitialized( FMath::Min( Size, 65507u ) );
            int32 Read = 0;
            if ( m_pSocket->RecvFrom( Reader->GetData(), Reader->Num(), Read, *Sender ) )
            {
                m_bConnected = true;

                #if UE_VERSION_OLDER_THAN(5, 5, 0)
                   Reader->RemoveAt(Read, Reader->Num() - Read, false);
                #else 
                   Reader->RemoveAt(Read, Reader->Num() - Read, EAllowShrinking::No);
                #endif
                
                TArray<uint8_t> RawData;
                uint32 DataReceivedNum = Reader->Num();
                RawData.Append( Reader->GetData(), DataReceivedNum );
                FString strResponse = reinterpret_cast< char* >( RawData.GetData() );

                int iStrLength = strResponse.Len();

                int iDataLength = RawData.Num();
                /// trim extra characters
                int iChopCount = iStrLength - iDataLength;
                if ( iChopCount > 0 )
                {
                    strResponse = strResponse.LeftChop( iChopCount );
                }
                ProcessResponse(strResponse);
            }
        }
    }
    return 0;
}

void MvnRemoteControlSession::Stop()
{
    m_bStopping = true;
}

bool MvnRemoteControlSession::SendCommand( MvnRemoteControlMessage* _pMessage, const FInternetAddr& Destination )
{
    const char* pcText = TCHAR_TO_UTF8( *_pMessage->m_strXml );
    if ( !m_pSocket )
    {
        return false;
    }
    int32 iBytesSentCount = 0;
    m_pSocket->SendTo( (const uint8*)pcText, strlen( pcText ), iBytesSentCount, Destination );
    if ( iBytesSentCount <= 0 )
    {
        return false;
    }
    return true;
}

FString MvnRemoteControlSession::GetSessionName() const 
{
    return m_strSessionName; 
}

FString MvnRemoteControlSession::GetTake() const 
{
    return m_strTake;
}

FString MvnRemoteControlSession::GetIPAddressAndPort() const
{
    return m_strIPAddress; 
}

FString MvnRemoteControlSession::GetStatus() const 
{
    if ( m_strLastError.IsEmpty() == false )
    {
        return m_strLastError;
    }

    if ( m_bConnected == false )
    {
        return "No Connection";
    }

    if ( m_strSessionName.IsEmpty() )
    {
        return "No Session";
    }

    /*
    if ( m_bLive == false )
    {
        return "No Live session";
    }

    if ( m_bPaused )
    {
        return "Paused";
    }
    */

    if ( m_bRecording )
    {
        return "Recording";
    }
    else
    {
        return "Idle";
    }
}

FString MvnRemoteControlSession::GetElapsedTime() const 
{
    if ( m_bRecording )
    {
        FTimespan diff = FDateTime::Now() - m_StartRecordTime;
        FString strTime = diff.ToString( TEXT( "%h:%m:%s" ) ).Replace( TEXT( "+" ), TEXT( "" ) );
        return strTime;
    }
    else
    {
        return "00:00:00";
    }
}

void MvnRemoteControlSession::ProcessResponse( FString& _strResponse )
{
    MvnRemoteControlMessage* pResponse = MvnRemoteControlSessionUtil::CreateResponseMessage(_strResponse);
    if ( pResponse )
    {
        // on start recording Ack received
        if ( pResponse->m_Command == eMvnRemoteControlMessageCommand::CaptureStart )
        {
            OnCaptureStartAckReceived(pResponse);
        }
        // on stop recording Ack received
        if ( pResponse->m_Command == eMvnRemoteControlMessageCommand::CaptureStop )
        {
            OnCaptureStopAckReceived( pResponse );
        }
        // session info ack received
        if ( pResponse->m_Command == eMvnRemoteControlMessageCommand::SessionInfo )
        {
            OnSessionInfoAckReceived( pResponse );
        }
        // session status ack received
        if ( pResponse->m_Command == eMvnRemoteControlMessageCommand::SessionStatus )
        {
            OnSessionStatusAckReceived( pResponse );
        }
        // Capture name ack received
        if ( pResponse->m_Command == eMvnRemoteControlMessageCommand::CaptureName )
        {
            OnCaptureNameAckReceived( pResponse );
        }

        delete pResponse;
    }
}


const FIPv4Address& MvnRemoteControlSession::GetDestIPAddress()
{
    return m_DestIPAddress;
}

void MvnRemoteControlSession::Identify()
{
    QueueRequest( MvnRemoteControlSessionUtil::CreateIdentifyRequest() );
}

void MvnRemoteControlSession::SetCaptureName( FString& _strCaptureName, int _iTakeCount )
{
    QueueRequest( MvnRemoteControlSessionUtil::CreateCaptureNameRequest(_strCaptureName, _iTakeCount) );
}

bool MvnRemoteControlSession::CaptureStart( double _dTimeOffsetInSeconds )
{
    if ( m_bRecording )
    {
        return false;
    }

    FString strDescription = "";
    QueueRequest( MvnRemoteControlSessionUtil::CreateCaptureStartRequest( strDescription, _dTimeOffsetInSeconds ) );
    return true;
}
void MvnRemoteControlSession::OnCaptureStartAckReceived( MvnRemoteControlMessage* _pResponse )
{
    bool bSuccess = _pResponse->m_bResult;

    // if success, start timer
    if ( bSuccess )
    {
        m_StartRecordTime = FDateTime::Now();
        m_bRecording = true;
        m_strLastError = "";
    }
    else
    {
        m_strLastError = _pResponse->m_strReason;
    }
}

bool MvnRemoteControlSession::CaptureStop( double _dTimeOffsetInSeconds )
{
    if ( !m_bRecording )
    {
        return false;
    }
    QueueRequest( MvnRemoteControlSessionUtil::CreateCaptureStopRequest( _dTimeOffsetInSeconds ) );
    return true;
}

void MvnRemoteControlSession::OnCaptureStopAckReceived( MvnRemoteControlMessage* _pResponse )
{
    bool bSuccess = _pResponse->m_bResult;

    if ( bSuccess )
    {
        m_bRecording = false;
        m_strLastError = "";
    }
    else
    {
        m_strLastError = _pResponse->m_strReason;
    }
}


bool MvnRemoteControlSession::IsRecording()
{
    return m_bRecording;
}


void MvnRemoteControlSession::SetTimeSyncEnabled( bool _bTimeSynEnabled )
{
    m_bTimeSyncEnabled = _bTimeSynEnabled;
}


bool MvnRemoteControlSession::IsTimeSyncEnabled()
{
    return m_bTimeSyncEnabled;
}



void MvnRemoteControlSession::SetDestinationIPAddress( const FIPv4Address& _destIPAddress )
{
    m_DestIPAddress = _destIPAddress;
}


void MvnRemoteControlSession::SetDestinationPort( int _iDestPort )
{
    m_iDestPort = _iDestPort;
}


const FIPv4Address& MvnRemoteControlSession::GetDestinationIPAddress()
{
    return m_DestIPAddress;
}


int MvnRemoteControlSession::GetDestinationPort()
{
    return m_iDestPort;
}

void MvnRemoteControlSession::Reset()
{
    m_strLastError = TEXT("");
}

void MvnRemoteControlSession::OnSessionInfoAckReceived( MvnRemoteControlMessage* _pResponse )
{
    FString strSessionNameKey = TEXT("SessionName");

    FString strRecordingTrialKey = TEXT("recordingTrial");

    if ( _pResponse->m_Attributes.Contains( strSessionNameKey ) )
    {
        FString SessionName = _pResponse->m_Attributes[strSessionNameKey];
        if (m_strSessionName !=  SessionName)
        {
            Reset();
        }

        m_strSessionName = SessionName;
    }

    if ( _pResponse->m_Attributes.Contains( strRecordingTrialKey ) )
    {
        m_strTake = _pResponse->m_Attributes[ strRecordingTrialKey ];
    }
}

void MvnRemoteControlSession::OnSessionStatusAckReceived( MvnRemoteControlMessage* _pResponse )
{
    FString strIsRecordingKey = TEXT("isRecording");
    FString strIsPausedKey = TEXT("isPaused");
    FString strIsLiveKey = TEXT("isLive");

    if ( _pResponse->m_Attributes.Contains( strIsRecordingKey ) )
    {
        FString strIsRecording = _pResponse->m_Attributes[ strIsRecordingKey ];
        if ( strIsRecording.Compare( TEXT("true"), ESearchCase::IgnoreCase ) == 0 )
        {
            if ( m_bRecording == false )
            {
                m_StartRecordTime = FDateTime::Now();
            }
            m_bRecording = true;
        }
        else
        {
            m_bRecording = false;
        }
    }

    if ( _pResponse->m_Attributes.Contains( strIsPausedKey ) )
    {
        FString strIsPaused = _pResponse->m_Attributes[ strIsPausedKey ];
        if ( strIsPaused.Compare( TEXT( "true" ), ESearchCase::IgnoreCase ) == 0 )
        {
            m_bPaused = true;
        }
        else
        {
            m_bPaused = false;
        }
    }

    if ( _pResponse->m_Attributes.Contains( strIsLiveKey ) )
    {
        FString strIsLive = _pResponse->m_Attributes[ strIsLiveKey ];
        if ( strIsLiveKey.Compare( TEXT( "true" ), ESearchCase::IgnoreCase ) == 0 )
        {
            m_bLive = true;
        }
        else
        {
            m_bLive = false;
        }
    }
}

void MvnRemoteControlSession::OnCaptureNameAckReceived( MvnRemoteControlMessage* _pResponse )
{
    FString strSuccessKey = TEXT("Success");

    FString strSuccessValue = _pResponse->m_Attributes[ strSuccessKey ];
    if ( strSuccessValue.Compare( TEXT("true"), ESearchCase::IgnoreCase ) == 0 )
    {
        m_strLastError = TEXT("");
    }
    else
    {
        m_strLastError = TEXT("Failed to change capture name");
    }
}

void MvnRemoteControlSession::RequestSessionInfo()
{
    QueueRequest( MvnRemoteControlSessionUtil::CreateSessionInfoRequest() );
}

void MvnRemoteControlSession::RequestSessionStatus()
{
    QueueRequest( MvnRemoteControlSessionUtil::CreateSessionStatusRequest() );
}

void MvnRemoteControlSession::RequestTimeSync()
{
    QueueRequest( MvnRemoteControlSessionUtil::CreateTimeSyncRequest( false ) );
}

void MvnRemoteControlSession::QueueRequest( MvnRemoteControlMessage* _pMessage )
{
    if ( _pMessage )
    {
        m_CommandsCriticalSection.Lock();
        m_apCommandsQueue.Add( _pMessage );
        m_CommandsCriticalSection.Unlock();
    }
}


