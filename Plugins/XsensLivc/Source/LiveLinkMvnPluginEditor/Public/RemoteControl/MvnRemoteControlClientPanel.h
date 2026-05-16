// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "MvnRemoteControlUtils.h"

class ITableRow;
class FMenuBuilder;
class FUICommandList;
class STableViewBase;
class SComboButton;

class MvnRemoteControlManager;
class SMvnRemoteControlSessionsListView;
class FSlateStyleSet;

struct FMvnRemoteControlSessionUIEntry;

typedef TSharedPtr<FMvnRemoteControlSessionUIEntry> FMvnRemoteControlSessionUIEntryPtr;

class SMvnRemoteControlClientPanel : public SCompoundWidget, public FGCObject, public FEditorUndoClient
{
    friend class SMvnRemoteControlNewSessionWidget;

	SLATE_BEGIN_ARGS(SMvnRemoteControlClientPanel){}
	SLATE_END_ARGS()

	~SMvnRemoteControlClientPanel();

	void Construct(const FArguments& Args, MvnRemoteControlManager* _RemoteControlManager);

    /** FGCObject interface */
    void AddReferencedObjects( FReferenceCollector& Collector ) override;
	virtual FString GetReferencerName() const override { return TEXT("SMvnRemoteControlClientPanel"); }

    /** remove a session */
    FReply RemoveRemoteControlSession( FMvnRemoteControlSessionUIEntryPtr _pSessionEntry );

    /** getter for selected entry */
    FMvnRemoteControlSessionUIEntryPtr GetSelectedEntry() const;


    void OnSelectedSessionChanged( FMvnRemoteControlSessionUIEntryPtr Entry, ESelectInfo::Type SelectionType );

private:

    /** remote control sessions widget */
    TSharedPtr<SMvnRemoteControlSessionsListView> m_pSessionsListView;

    /** popup combo for adding new sessions */
    TSharedPtr<SComboButton> m_pNewSessionComboButton;

    /** Source list items */
    TArray<FMvnRemoteControlSessionUIEntryPtr> m_aSessionsData;

    /** prepare the sessions list widget */
    TSharedRef<ITableRow> MakeSessionListViewWidget( FMvnRemoteControlSessionUIEntryPtr Entry, const TSharedRef<STableViewBase>& OwnerTable );

    /** adds a new remote control instance */
    FReply AddRemoteControlInstance();

    /** remove selected instance */
    FReply RemoveSelectedRemoteControlInstance();

    /** update selected session */
    FReply UpdateSelectedSession();

    /** start Recording */
    FReply StartRecording();

    /** stop Recording */
    FReply StopRecording();
    
    /** toggle recording */
    void OnToggleRecording(ECheckBoxState);

    /** is recording */
    ECheckBoxState IsRecording() const;

    /** toggle time sync */
    void OnToggleTimeSyncEnabled( ECheckBoxState );

    /** is time sync enabled */
    ECheckBoxState IsTimeSyncEnabled() const;

    /** can record */
    bool CanRecord() const;

    /** remote control manager */
    MvnRemoteControlManager* RemoteControlManager = nullptr;

    /** session name as displayed */
    FText m_SessionName;

    /** session take as displayed */
    FText m_SessionTake;

    /** destination IP Address as displayed */
    FText m_SessionIPAddress;

    /** destination port */
    int m_iSessionPort = 0;

    /** new session validation text text */
    FText m_SessionFeedback;

    /** destination IP Address as displayed */
    FText m_NewSessionIPAddress;

    /** destination port */
    FText m_iNewSessionPort;

    /** new session validation text text */
    FText m_NewSessionFeedback;

    /** sync time offset */
    double m_dDefaultCaptureStartStopDelay = 0.5;

    /** time sync enabled/disabled*/
    bool m_bTimeSyncEnabled = true;

    /** selected entry */
    FMvnRemoteControlSessionUIEntryPtr m_pSelectedEntry = nullptr;

    mutable bool bSelectionChangedGuard = false;

    FText GetSessionName() const;

    void GetSessionNameFromInput( const FText& Text );

    FText GetTake() const;

    FText GetSessionIPAddress() const;

    void GetSessionIPAddressFromInput( const FText& Text );

    FText GetSessionPort() const;

    void GetSessionPortFromInput( const FText& Text );

    FText GetSessionFeedback() const;

    void GetTakeFromInput( const FText& Text );

    FText GetNewSessionIPAddress() const;

    void GetNewSessionIPAddressFromInput( const FText& Text );

    FText GetNewSessionPort() const;

    void GetNewSessionPortFromInput( const FText& Text );

    FText GetNewSessionFeedback() const;

    FText GetStartStopDelay() const;

    void GetStartStopDelayFromInput( const FText& Text );

    TSharedRef<SWidget> MakeAddSessionWidget();

};
