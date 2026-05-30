// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "MessageEndpoint.h"
#include "IMessageContext.h"
#include "HAL/ThreadSafeBool.h"
#include "Common/UdpSocketReceiver.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include <memory>
#include "HAL/CriticalSection.h"
#include "XmlParser.h"
#include "FastXml.h"
#include "MvnRemoteControlMessage.h"

/** Helper namespace for creating and processing Mvn Remote control messages */
namespace MvnRemoteControlSessionUtil
{
    /** Get matching message from text */
    eMvnRemoteControlMessageCommand GetMatchingMesssageCommand( const TCHAR* AttributeName );

    /** Create a response message using the xml string coming from mvn side */
    MvnRemoteControlMessage* CreateResponseMessage( FString& _strXMLResponse );

    /** Create a new request message*/
    MvnRemoteControlMessage* CreateRequest( eMvnRemoteControlMessageCommand _command );

    /** Create a new identify request message*/
    MvnRemoteControlMessage* CreateIdentifyRequest();

    /** Create a new capture name request message*/
    MvnRemoteControlMessage* CreateCaptureNameRequest( FString& _strCaptureName, int _iTakeCount = -1 );

    /** Create a new capture start request message*/
    MvnRemoteControlMessage* CreateCaptureStartRequest( FString& _strNotes, double _dTimeOffsetInSeconds );

    /** Create a new capture stop request message*/
    MvnRemoteControlMessage* CreateCaptureStopRequest( double _dTimeOffsetInSeconds );

    /** Create a new time sync request message*/
    MvnRemoteControlMessage* CreateTimeSyncRequest( bool _bReset );

    /** Create a new session info request message*/
    MvnRemoteControlMessage* CreateSessionInfoRequest();

    /** Create a new session status request message*/
    MvnRemoteControlMessage* CreateSessionStatusRequest();

    /** formulate time code string from offset of current time in seconds*/
    FString FormulateTimeCodeString( double _dTimeOffsetInSeconds );
}

