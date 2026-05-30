// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MvnRemoteControlSessionUtil.h"
#include "Misc/App.h"


namespace MvnRemoteControlSessionUtil
{
    static const FName IdentifyAckAckName( TEXT( "IdentifyAck" ) );
    static const FName CaptureNameAckName( TEXT( "CaptureNameAck" ) );
    static const FName CaptureStartAckName( TEXT( "CaptureStartAck" ) );
    static const FName CaptureStopAckName( TEXT( "CaptureStopAck" ) );
    static const FName StartMeasuringAckName( TEXT( "StartMeasuringAck" ) );
    static const FName StopMeasuringAckName( TEXT( "StopMeasuringAck" ) );
    static const FName StartRecordingAckName( TEXT( "StartRecordingAck" ) );
    static const FName StopRecordingAckName( TEXT( "StopRecordingAck" ) );
    static const FName PlayPauseAckName( TEXT( "PlayPauseAck" ) );
    static const FName NavigateToStartAckName( TEXT( "NavigateToStartAck" ) );
    static const FName NavigateToEndAckName( TEXT( "NavigateToEndAck" ) );
    static const FName NextFrameAckName( TEXT( "NextFrameAck" ) );
    static const FName ToggleRepeatAckName( TEXT( "ToggleRepeatAck" ) );
    static const FName AddMarkerAckName( TEXT( "AddMarkerAck" ) );
    static const FName AddNetworkStreamingTargetAckName( TEXT( "AddNetworkStreamingTargetAck" ) );
    static const FName SessionStatusAckName( TEXT( "SessionStatusAck" ) );
    static const FName MoveCharacterToOriginAckName( TEXT( "MoveCharacterToOriginAck" ) );
    static const FName SessionInfoAckName( TEXT( "SessionInfoAck" ) );
    static const FName JumpToFrameAckName( TEXT( "JumpToFrameAck" ) );
    static const FName SetMediaRecorderAddressAckName( TEXT( "SetMediaRecorderAddressAck" ) );
    static const FName SetSessionNameAckName( TEXT( "SetSessionNameAck" ) );

};


class MvnXmlParserCallback : public  IFastXmlCallback
{
public:

    MvnRemoteControlMessage* m_pMessage = nullptr;

    int m_iTreeLevel = 0;

private:

    bool ProcessXmlDeclaration( const TCHAR* ElementData, int32 XmlFileLineNumber )
    {
        return true;
    }

    bool ProcessElement( const TCHAR* ElementName, const TCHAR* ElementData, int32 XmlFileLineNumber )
    {
        m_iTreeLevel++;
        if ( m_pMessage == nullptr )
        {
            eMvnRemoteControlMessageCommand commandResponse = MvnRemoteControlSessionUtil::GetMatchingMesssageCommand( ElementName );
            if ( commandResponse != eMvnRemoteControlMessageCommand::Undefined )
            {
                m_pMessage = new MvnRemoteControlMessage();
                m_pMessage->m_Type = eMvnRemoteControlMessageType::Response;
                m_pMessage->m_Command = commandResponse;
            }
        }
        return true;
    }

    bool ProcessAttribute( const TCHAR* AttributeName, const TCHAR* AttributeValue )
    {
        /// process attributes at level 1
        if ( ( m_iTreeLevel == 1 ) && m_pMessage )
        {
            FString strKey = AttributeName;
            FString strValue = AttributeValue;

            // replace escape sequences
            FString strValueUnescaped = strValue.Replace( TEXT( "&amp;" ), TEXT( "&" ) )
                .Replace( TEXT( "&quot;" ), TEXT( "\"" ) )
                .Replace( TEXT( "&apos;" ), TEXT( "'" ) )
                .Replace( TEXT( "&lt;" ), TEXT( "<" ) )
                .Replace( TEXT( "&gt;" ), TEXT( ">" ) );


            m_pMessage->m_Attributes.Add(strKey, strValueUnescaped);

            if ( strKey == "Result" )
            {
                if ( strValue.Compare( L"true", ESearchCase::IgnoreCase ) == 0 )
                {
                    m_pMessage->m_bResult = true;
                }
                else
                {
                    m_pMessage->m_bResult = false;
                }
            }
            if ( strKey == "Reason" )
            {
                m_pMessage->m_strReason = strValueUnescaped;
            }
        }
        return true;
    }

    bool ProcessClose( const TCHAR* Element )
    {
        m_iTreeLevel--;
        return true;
    }

    bool ProcessComment( const TCHAR* Comment )
    {
        return true;
    }
};

eMvnRemoteControlMessageCommand MvnRemoteControlSessionUtil::GetMatchingMesssageCommand( const TCHAR* AttributeName )
{
    eMvnRemoteControlMessageCommand result = eMvnRemoteControlMessageCommand::Undefined;
    if ( AttributeName == IdentifyAckAckName )
    {
        result = eMvnRemoteControlMessageCommand::Identify;
    }
    else if ( AttributeName == CaptureNameAckName )
    {
        result = eMvnRemoteControlMessageCommand::CaptureName;
    }
    else if ( AttributeName == CaptureStartAckName )
    {
        result = eMvnRemoteControlMessageCommand::CaptureStart;
    }
    else if ( AttributeName == CaptureStopAckName )
    {
        result = eMvnRemoteControlMessageCommand::CaptureStop;
    }
    else if ( AttributeName == StartMeasuringAckName )
    {
        result = eMvnRemoteControlMessageCommand::StartMeasuring;
    }
    else if ( AttributeName == StopMeasuringAckName )
    {
        result = eMvnRemoteControlMessageCommand::StopMeasuring;
    }
    else if ( AttributeName == StartRecordingAckName )
    {
        result = eMvnRemoteControlMessageCommand::StartRecording;
    }
    else if ( AttributeName == StopRecordingAckName )
    {
        result = eMvnRemoteControlMessageCommand::StopRecording;
    }
    else if ( AttributeName == PlayPauseAckName )
    {
        result = eMvnRemoteControlMessageCommand::PlayPause;
    }
    else if ( AttributeName == NavigateToStartAckName )
    {
        result = eMvnRemoteControlMessageCommand::NavigateToStart;
    }
    else if ( AttributeName == NavigateToEndAckName )
    {
        result = eMvnRemoteControlMessageCommand::NavigateToEnd;
    }
    else if ( AttributeName == NextFrameAckName )
    {
        result = eMvnRemoteControlMessageCommand::NextFrame;
    }
    else if ( AttributeName == ToggleRepeatAckName )
    {
        result = eMvnRemoteControlMessageCommand::ToggleRepeat;
    }
    else if ( AttributeName == AddMarkerAckName )
    {
        result = eMvnRemoteControlMessageCommand::AddMarker;
    }
    else if ( AttributeName == AddNetworkStreamingTargetAckName )
    {
        result = eMvnRemoteControlMessageCommand::AddNetworkStreamingTarget;
    }
    else if ( AttributeName == SessionStatusAckName )
    {
        result = eMvnRemoteControlMessageCommand::SessionStatus;
    }
    else if ( AttributeName == MoveCharacterToOriginAckName )
    {
        result = eMvnRemoteControlMessageCommand::MoveCharacterToOrigin;
    }
    else if ( AttributeName == SessionInfoAckName )
    {
        result = eMvnRemoteControlMessageCommand::SessionInfo;
    }
    else if ( AttributeName == JumpToFrameAckName )
    {
        result = eMvnRemoteControlMessageCommand::JumpToFrame;
    }
    else if ( AttributeName == SetMediaRecorderAddressAckName )
    {
        result = eMvnRemoteControlMessageCommand::SetMediaRecorderAddress;
    }
    else if ( AttributeName == SetSessionNameAckName )
    {
        result = eMvnRemoteControlMessageCommand::SetSessionName;
    }
    return result;
}


MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateResponseMessage( FString& _strXMLResponse )
{
    FFastXml xmlParser;
    FText errorMsg;
    int iErrorLine = -1;
    MvnXmlParserCallback callBack;
    TArray<TCHAR> aChars = _strXMLResponse.GetCharArray();
    TCHAR* pCharData = aChars.GetData();
    xmlParser.ParseXmlFile( &callBack, nullptr, pCharData, nullptr, false, false, errorMsg, iErrorLine );
    if ( callBack.m_pMessage )
    {
        callBack.m_pMessage->m_strXml = _strXMLResponse;
    }
    return callBack.m_pMessage;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateRequest( eMvnRemoteControlMessageCommand _command )
{
    MvnRemoteControlMessage* pNewCommand = new MvnRemoteControlMessage();
    pNewCommand->m_Type = eMvnRemoteControlMessageType::Request;
    pNewCommand->m_Command = _command;
    return pNewCommand;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateIdentifyRequest()
{
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::Identify );
    pNewCommand->m_strXml = "<IdentifyReq/>";
    return pNewCommand;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateCaptureNameRequest( FString& _strCaptureName, int _iTakeCount )
{
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::CaptureName );
    FString strXml;
    if ( _iTakeCount > -1 )
    {
        strXml = FString::Printf( L"<CaptureName><Name VALUE=\"%s\"/><Take VALUE=\"%03d\"/></CaptureName>", *_strCaptureName, _iTakeCount );
    }
    else
    {
        strXml = FString::Printf( L"<CaptureName><Name VALUE=\"%s\"/></CaptureName>", *_strCaptureName);
    }
    pNewCommand->m_strXml = strXml;
    return pNewCommand;
}


MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateCaptureStartRequest( FString& _strNotes, double _dTimeOffsetInSeconds  )
{
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::CaptureStart );
    FString strTimeCode = FormulateTimeCodeString( _dTimeOffsetInSeconds );
    FString strXml;
    FString strTimeCodeTag = "";

    if ( _dTimeOffsetInSeconds != 0.0 )
    {
        strTimeCodeTag = FString::Printf( L"<TimeCode VALUE=\"%s\"/>", *strTimeCode );
    }

    if ( _strNotes.IsEmpty() )
    {
        strXml = FString::Printf( L"<CaptureStart>%s</CaptureStart>", *strTimeCodeTag );
    }
    else
    {
        strXml = FString::Printf( L"<CaptureStart>%s<Notes>%s</Notes></CaptureStart>", *strTimeCodeTag, *_strNotes );
    }
    pNewCommand->m_strXml = strXml;
    return pNewCommand;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateCaptureStopRequest( double _dTimeOffsetInSeconds )
{
    FString strTimeCode = FormulateTimeCodeString( _dTimeOffsetInSeconds );
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::StopRecording );
    pNewCommand->m_strXml = FString::Printf( L"<CaptureStop><TimeCode VALUE=\"%s\"/></CaptureStop>", *strTimeCode );
    return pNewCommand;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateTimeSyncRequest( bool _bReset )
{
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::TimeSync );
    FString strTimeCode = FormulateTimeCodeString( 0 );
    FString strReset = _bReset ? "True" : "False";
    pNewCommand->m_strXml = FString::Printf( L"<TimeSync TimeCode=\"%s\"/>", *strTimeCode, *strReset );
    return pNewCommand;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateSessionInfoRequest()
{
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::SessionInfo );
    pNewCommand->m_strXml = "<SessionInfoReq/>";
    return pNewCommand;
}

MvnRemoteControlMessage* MvnRemoteControlSessionUtil::CreateSessionStatusRequest()
{
    MvnRemoteControlMessage* pNewCommand = CreateRequest( eMvnRemoteControlMessageCommand::SessionStatus );
    pNewCommand->m_strXml = "<SessionStatusReq/>";
    return pNewCommand;
}

FString MvnRemoteControlSessionUtil::FormulateTimeCodeString( double _dTimeOffsetInSeconds )
{
    FTimespan offsetSpan = FTimespan::FromSeconds( _dTimeOffsetInSeconds );

    // 1. Start with current time code (can be different from system time)
    FTimecode now = FApp::GetTimecode();
    FTimespan timeCode = now.ToTimespan( FApp::GetTimecodeFrameRate() );

    // 2. Add provided offset
    timeCode += offsetSpan;

    // 3. <CaptureStart> and <CaptureStop> messages should be the current time increased with half a second, and be rounded up to the nearest whole second.
    timeCode += FTimespan::FromSeconds( 0.5 );

    return FString::Printf( TEXT( "%02d %02d %02d" ), timeCode.GetHours(), timeCode.GetMinutes(), timeCode.GetSeconds() );
}
