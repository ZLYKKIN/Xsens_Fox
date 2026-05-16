// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "MessageEndpoint.h"
#include "IMessageContext.h"
#include "HAL/ThreadSafeBool.h"
#include "FastXml.h"

/** enum for all Mvn Remote control messages */
enum class eMvnRemoteControlMessageCommand
{
    Undefined,
    Identify,
    CaptureName,
    CaptureStart,
    CaptureStop,
    TimeSync,
    StartMeasuring,
    StopMeasuring,
    StartRecording,
    StopRecording,
    PlayPause,
    NavigateToStart,
    NavigateToEnd,
    NextFrame,
    ToggleRepeat,
    AddMarker,
    AddNetworkStreamingTarget,
    SessionStatus,
    MoveCharacterToOrigin,
    SessionInfo,
    JumpToFrame,
    SetMediaRecorderAddress,
    SetSessionName
};

/** enum for all Mvn Remote control messages type ( Request or Response (ack) )*/
enum class eMvnRemoteControlMessageType
{
    Undefined,
    Request,
    Response
};

/** represent an Mvn Remote control message */
struct MvnRemoteControlMessage
{
    /** Command this message is related to */
    eMvnRemoteControlMessageCommand m_Command = eMvnRemoteControlMessageCommand::Undefined;

    /** Type : Request / Response */
    eMvnRemoteControlMessageType m_Type = eMvnRemoteControlMessageType::Undefined;

    /** result of a command ( true = success, false = fail )*/
    bool m_bResult = false;

    /** reason */
    FString m_strReason;

    /** attributes */
    TMap< FString, FString> m_Attributes;

    /** xml string */
    FString m_strXml;


};


