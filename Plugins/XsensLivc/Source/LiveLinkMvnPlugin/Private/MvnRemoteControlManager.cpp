// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnRemoteControlManager.h"
#include "MvnRemoteControlSession.h"

MvnRemoteControlManager* MvnRemoteControlManager::s_pInstance = nullptr;

void UMvnRemoteControlManager::StartRecording( float _fTimeOffsetInSeconds )
{
	MvnRemoteControlManager* pManager = MvnRemoteControlManager::GetInstance();
	if (pManager)
	{
		pManager->CaptureStartAll( _fTimeOffsetInSeconds );
	}
}


void UMvnRemoteControlManager::StopRecording( float _fTimeOffsetInSeconds )
{
	MvnRemoteControlManager* pManager = MvnRemoteControlManager::GetInstance();
	if (pManager)
	{
		pManager->CaptureStopAll( _fTimeOffsetInSeconds );
	}
}

MvnRemoteControlManager::MvnRemoteControlManager()
{
    s_pInstance = this;
    /// reserve capture port 
    AddReservedPort( 9763 );
}

MvnRemoteControlManager::~MvnRemoteControlManager()
{
    for ( int32 i = 0; i < Sessions.Num(); i++ )
    {
        delete Sessions[ i ];
    }
    Sessions.Empty();
}

MvnRemoteControlSession* MvnRemoteControlManager::CreateSession( const FIPv4Address& _ipAddress, int _iPort )
{
    MvnRemoteControlSession* pNewSession = new MvnRemoteControlSession( _ipAddress, _iPort );
    Sessions.Add( pNewSession );
    return pNewSession;
}

bool MvnRemoteControlManager::CanCreateSession( const FIPv4Address& _ipAddress, int _iPort )
{
    bool bCanCreate = true;
    for ( int i = 0; i < Sessions.Num(); i++ )
    {
        if ( Sessions[ i ]->GetDestIPAddress() == _ipAddress )
        {
            bCanCreate = false;
        }
    }
    return bCanCreate;
}

void MvnRemoteControlManager::DestroySession( MvnRemoteControlSession* _pSession )
{
    if ( Sessions.Contains( _pSession ) )
    {
        Sessions.Remove( _pSession );
        delete _pSession;
    }
}

void MvnRemoteControlManager::CaptureStartAll( double _dTimeOffsetInSeconds )
{
    for ( int i = 0; i < Sessions.Num(); i++ )
    {
        Sessions[ i ]->CaptureStart( _dTimeOffsetInSeconds );
    }
}

void MvnRemoteControlManager::CaptureStopAll( double _dTimeOffsetInSeconds )
{
    for ( int i = 0; i < Sessions.Num(); i++ )
    {
        Sessions[ i ]->CaptureStop( _dTimeOffsetInSeconds );
    }
}

const TArray< MvnRemoteControlSession* >& MvnRemoteControlManager::GetSessions()
{
    return Sessions;
}

bool MvnRemoteControlManager::IsAnyRecording()
{
    for ( int i = 0; i < Sessions.Num(); i++ )
    {
        if ( Sessions[ i ]->IsRecording() )
        {
            return true;
        }
    }
    return false;
}

void MvnRemoteControlManager::SetTimeSyncEnabled( bool _bTimeSyncEnabled )
{
    for ( int i = 0; i < Sessions.Num(); i++ )
    {
        Sessions[ i ]->SetTimeSyncEnabled( _bTimeSyncEnabled);
    }
}

void MvnRemoteControlManager::AddReservedPort( int32 _iPort )
{
    m_aReservedPorts.Add( _iPort );
}

void MvnRemoteControlManager::RemoveReservedPort( int32 _iPort )
{
    m_aReservedPorts.Remove( _iPort );
}

bool MvnRemoteControlManager::IsPortReserved( int32 _iPort )
{
    return m_aReservedPorts.Contains( _iPort );
}

MvnRemoteControlManager* MvnRemoteControlManager::GetInstance()
{
    return s_pInstance;
}




