// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "RemoteControl/MvnRemoteControlClientPanel.h"

#include "Templates/SharedPointer.h"

#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SNullWidget.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Layout/SSplitter.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/STextEntryPopup.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Views/SListView.h"

#include "Editor.h"
#include "Editor/EditorPerformanceSettings.h"
#include "EditorFontGlyphs.h"
#include "EditorStyleSet.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "UObject/UObjectHash.h"
#include "MvnRemoteControlSession.h"
#include "MvnRemoteControlManager.h"
#include "Styling/SlateStyle.h"
#include "Styling/SlateStyleRegistry.h"
#include "Interfaces/IPluginManager.h"
#include "RemoteControl/MvnRemoteControlNewSessionWidget.h"
#include "Runtime/Launch/Resources/Version.h"
#include <Widgets/Input/SComboButton.h>

#define LOCTEXT_NAMESPACE "MvnRemoteControlClientPanel"

#define IMAGE_PLUGIN_BRUSH( RelativePath, ... ) FSlateImageBrush( MvnRemoteControlUtils::InPluginContent( RelativePath, ".png" ), __VA_ARGS__ )

//#define MVN_RC_ENABLE_CHANGE_IP_PORT

// Static Source UI column names
namespace MvnRemoteControlListUI
{
    static const FName InstanceColumnName( TEXT( "Instance" ) );
    static const FName TakeColumnName( TEXT( "Take" ) );
    static const FName RecordingColumnName( TEXT( "Recording" ) );
    static const FName ElapsedTimeColumnName( TEXT( "ElapsedTime" ) );
    static const FName IPAddressColumnName( TEXT( "IP Address" ) );
    static const FName RemoveSessionColumnName( TEXT( "Remove" ) );  
};

// Structure that defines a single entry in the source UI
struct FMvnRemoteControlSessionUIEntry
{
public:
    FMvnRemoteControlSessionUIEntry( FGuid InEntryGuid, MvnRemoteControlSession* _session, MvnRemoteControlManager* _manager )  : EntryGuid( InEntryGuid ), Session( _session ), Manager( _manager )
    {}

    FGuid GetGuid() const { return EntryGuid; }
    FString GetInstance() const { return Session->GetSessionName(); }
    FString GetTake() const { return Session->GetTake();  }
    FString GetRecording() const { return Session->GetStatus();   }
    FString GetElapsedTime() const { return Session->GetElapsedTime();  }
    FString GetIPAddressAndPort() const { return Session->GetIPAddressAndPort();  }
    const FIPv4Address& GetDestinationIPAddress() { return Session->GetDestinationIPAddress(); }
    int GetDestinationPort() { return Session->GetDestinationPort(); }

    void Remove() const { Manager->DestroySession(Session); }

    MvnRemoteControlSession* GetSession()
    {
        return Session;
    }

    void SetSelected( bool _bSelected )
    {
        m_bSelected = _bSelected;
    }

    bool IsSelected() const
    {
        return m_bSelected;
    }
 
private:

    FGuid EntryGuid;
    MvnRemoteControlSession* Session;
    MvnRemoteControlManager* Manager;
    bool m_bSelected = false;
   
};


class SMvnRemoteControlSessionPanelRow : public SMultiColumnTableRow<FMvnRemoteControlSessionUIEntryPtr>
{
public:
    SLATE_BEGIN_ARGS( SMvnRemoteControlSessionPanelRow ) {}
        SLATE_ARGUMENT( FMvnRemoteControlSessionUIEntryPtr, Entry )
        SLATE_ARGUMENT( SMvnRemoteControlClientPanel*, Panel )
    SLATE_END_ARGS()

    void Construct( const FArguments& Args, const TSharedRef<STableViewBase>& OwnerTableView )
    {
        EntryPtr = Args._Entry;
        m_pPanel = Args._Panel;

        SMultiColumnTableRow<FMvnRemoteControlSessionUIEntryPtr>::Construct(
            FSuperRowType::FArguments()
            .Padding( 1.0f ),
            OwnerTableView
        );
    }

    /** Overridden from SMultiColumnTableRow.  Generates a widget for this column of the list view. */
    virtual TSharedRef<SWidget> GenerateWidgetForColumn( const FName& ColumnName ) override
    {
        if ( ColumnName == MvnRemoteControlListUI::InstanceColumnName )
        {
            return	SNew( STextBlock )
                .Text( MakeAttributeSP( this, &SMvnRemoteControlSessionPanelRow::GetInstance ) );
        }
        else if ( ColumnName == MvnRemoteControlListUI::TakeColumnName )
        {
            return	SNew( STextBlock )
                .Text( MakeAttributeSP( this, &SMvnRemoteControlSessionPanelRow::GetTake ) );
        }
        else if ( ColumnName == MvnRemoteControlListUI::RecordingColumnName )
        {
            return	SNew( STextBlock )
                .Text( MakeAttributeSP( this, &SMvnRemoteControlSessionPanelRow::GetRecording ) );
        }
        else if ( ColumnName == MvnRemoteControlListUI::IPAddressColumnName )
        {
            return	SNew( STextBlock )
                .Text( MakeAttributeSP( this, &SMvnRemoteControlSessionPanelRow::GetIPAddress ) );
        }
        else if ( ColumnName == MvnRemoteControlListUI::ElapsedTimeColumnName )
        {
            return	SNew( STextBlock )
                .Text( MakeAttributeSP( this, &SMvnRemoteControlSessionPanelRow::GetElapsedTime ) );
        }
        else if ( ColumnName == MvnRemoteControlListUI::RemoveSessionColumnName )
        {
            return SNew( SButton )
                .ButtonStyle( FAppStyle::Get(), "HoverHintOnly" )
                .HAlign( HAlign_Center )
                .VAlign( VAlign_Center )
                .OnClicked( this, &SMvnRemoteControlSessionPanelRow::OnRemoveClicked )
                .ToolTipText( LOCTEXT( "RemoveInstance", "Remove Instance" ) )
                .ContentPadding( 0.f )
                .ForegroundColor( FSlateColor::UseForeground() )
                .IsFocusable( false )
                [
                    SNew( SImage )
#if ENGINE_MAJOR_VERSION >= 5
                    .Image( FAppStyle::GetBrush( "Icons.Delete" ) )
#else
					.Image( FAppStyle::GetBrush( "PropertyWindow.Button_EmptyArray" ) )
#endif
                    .ColorAndOpacity( FSlateColor::UseForeground() )
                ];
        }


        return SNullWidget::NullWidget;
    }


private:
    FText GetInstance() const
    {
        FString strInstanceName = EntryPtr->GetInstance();
        if ( strInstanceName != m_strPreviousInstanceName )
        {
            if ( m_pPanel->GetSelectedEntry() == EntryPtr )
            {
                m_pPanel->OnSelectedSessionChanged( EntryPtr, ESelectInfo::Direct );
            }
            m_strPreviousInstanceName = strInstanceName;
        }
        return FText::FromString( strInstanceName );
    }
    FText GetTake() const
    {
        FString strTakeName = EntryPtr->GetTake();
        if ( strTakeName != m_strPreviousTake )
        {
            if ( m_pPanel->GetSelectedEntry() == EntryPtr )
            {
                m_pPanel->OnSelectedSessionChanged( EntryPtr, ESelectInfo::Direct );
            }
            m_strPreviousTake = strTakeName;
        }

        return FText::FromString( EntryPtr->GetTake() );
    }
    FText GetRecording() const
    {
        return FText::FromString( EntryPtr->GetRecording() );
    }
    FText GetElapsedTime() const
    {
        return FText::FromString( EntryPtr->GetElapsedTime() );
    }
    FText GetIPAddress() const
    {
        return FText::FromString( EntryPtr->GetIPAddressAndPort() );
    }
    FReply OnRemoveClicked()
    {
        m_pPanel->RemoveRemoteControlSession(EntryPtr);
        return FReply::Handled();
    }

    mutable FString m_strPreviousInstanceName;

    mutable FString m_strPreviousTake;

    FMvnRemoteControlSessionUIEntryPtr EntryPtr;
    SMvnRemoteControlClientPanel* m_pPanel = nullptr;
};


class SMvnRemoteControlSessionsListView : public SListView<FMvnRemoteControlSessionUIEntryPtr>
{
public:
    void Construct( FArguments Args )
    {
        SListView<FMvnRemoteControlSessionUIEntryPtr>::Construct( Args );
    }

public:
    virtual FReply OnKeyDown( const FGeometry& MyGeometry, const FKeyEvent& InKeyEvent ) override
    {
        if ( InKeyEvent.GetKey() == EKeys::Delete || InKeyEvent.GetKey() == EKeys::BackSpace )
        {
            TArray<FMvnRemoteControlSessionUIEntryPtr> SelectedItem = GetSelectedItems();
            for ( FMvnRemoteControlSessionUIEntryPtr Item : SelectedItem )
            {
                // TODO :disconnect and delete
            }
            return FReply::Handled();
        }

        return FReply::Unhandled();
    }


};


SMvnRemoteControlClientPanel::~SMvnRemoteControlClientPanel()
{
	GEditor->UnregisterForUndo(this);
}

void SMvnRemoteControlClientPanel::Construct( const FArguments& Args, MvnRemoteControlManager* _RemoteControlManager )
{
    bSelectionChangedGuard = false;
    m_NewSessionIPAddress = FText::FromString( TEXT( "127.0.0.1" ) );
	m_iNewSessionPort = FText::FromString( TEXT( "6004" ) );
    GEditor->RegisterForUndo( this );
    RemoteControlManager = _RemoteControlManager;

    const TArray< MvnRemoteControlSession* >& aSessions = RemoteControlManager->GetSessions();

    for ( int32 i = 0; i < aSessions.Num(); i++ )
    {
        TSharedPtr<FMvnRemoteControlSessionUIEntry> pNewSessionEntry = TSharedPtr<FMvnRemoteControlSessionUIEntry>( new FMvnRemoteControlSessionUIEntry( FGuid::NewGuid(), aSessions[ i ], RemoteControlManager ) );
        m_aSessionsData.Add( pNewSessionEntry );
    }

    ChildSlot
    [
         SNew( SBorder )
         .BorderImage( FAppStyle::GetBrush( "MessageLog.ListBorder" ) )
         [
            SNew( SVerticalBox )
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
            [
                SNew( SHorizontalBox )
                + SHorizontalBox::Slot()
                .FillWidth( 0.85f )
                .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                [
                    SNew( SVerticalBox )
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SHorizontalBox )
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( STextBlock )
                            .Text( LOCTEXT( "Instance Name", "Instance Name" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( SEditableTextBox )
                            .Text(this, &SMvnRemoteControlClientPanel::GetSessionName)
                        .   OnTextChanged_Raw(this, &SMvnRemoteControlClientPanel::GetSessionNameFromInput)
                        ]
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SHorizontalBox )
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        .VAlign( VAlign_Top )
                        [
                            SNew( STextBlock )
                            .Text( LOCTEXT( "Take", "Take" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        .VAlign( VAlign_Top )
                        [
                            SNew( SEditableTextBox )
                            .Text( this, &SMvnRemoteControlClientPanel::GetTake )
                            .OnTextChanged_Raw( this, &SMvnRemoteControlClientPanel::GetTakeFromInput )
                        ]
                    ]
#ifdef MVN_RC_ENABLE_CHANGE_IP_PORT
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SHorizontalBox )
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        .VAlign( VAlign_Top )
                        [
                            SNew( STextBlock )
                            .Text( LOCTEXT( "IP Address", "IP Address" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        .VAlign( VAlign_Top )
                        [
                            SNew( SEditableTextBox )
                            .Text( this, &SMvnRemoteControlClientPanel::GetSessionIPAddress )
                        .OnTextChanged_Raw( this, &SMvnRemoteControlClientPanel::GetSessionIPAddressFromInput )
                        ]
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SHorizontalBox )
                        +SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        .VAlign( VAlign_Top )
                        [
                            SNew( STextBlock )
                            .Text( LOCTEXT( "Port", "Port" ) )
                        ]
                        +SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        .VAlign( VAlign_Top )
                        [
                            SNew( SEditableTextBox )
                            .Text( this, &SMvnRemoteControlClientPanel::GetSessionPort )
                            .OnTextChanged_Raw( this, &SMvnRemoteControlClientPanel::GetSessionPortFromInput )
                        ]
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                            SNew( SVerticalBox )
                            + SVerticalBox::Slot()
                            .AutoHeight()
                            .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                            [
                                SNew( SHorizontalBox )
                                + SHorizontalBox::Slot()
                                .FillWidth( 0.5f )
                                .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                                [
                                    SNew( STextBlock )
                                    .ColorAndOpacity( FColor::Red )
                                    .Text( this, &SMvnRemoteControlClientPanel::GetSessionFeedback )
                                ]
                            ]
                    ]
#endif // MVN_RC_ENABLE_CHANGE_IP_PORT
                ]
                + SHorizontalBox::Slot()
                .FillWidth( 0.15f )
                .HAlign( HAlign_Center )
                .VAlign( VAlign_Top )
                [
                    SNew( SVerticalBox )
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 10.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SButton )
                        .HAlign( EHorizontalAlignment::HAlign_Left )
                        .VAlign(VAlign_Top)
                        .ButtonStyle( FAppStyle::Get(), "FlatButton.Success" )
                        .ContentPadding( FMargin( 12.0f, 1.0f ) )
                        .OnClicked( this, &SMvnRemoteControlClientPanel::UpdateSelectedSession )
                        [
                            SNew( STextBlock )
                            .TextStyle( FAppStyle::Get(), "NormalText.Important" )
                            .Text( LOCTEXT( "Set", "Set" ) )
                        ]
                    ]
                ]
            ]
            + SVerticalBox::Slot()
            .HAlign(HAlign_Left)
            .AutoHeight()
            .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
            [
                SAssignNew( m_pNewSessionComboButton, SComboButton )
                .OnGetMenuContent( this, &SMvnRemoteControlClientPanel::MakeAddSessionWidget )
                .ButtonStyle( FAppStyle::Get(), "FlatButton.Success" )
                .ContentPadding( FMargin( 2.0f, 1.0f ) )
                .HasDownArrow( false )
                .ButtonContent()
                [
                    SNew( SHorizontalBox )
                    + SHorizontalBox::Slot()
                    .VAlign( VAlign_Center )
                    .AutoWidth()
                    [
                        SNew( STextBlock )
                        .TextStyle( FAppStyle::Get(), "NormalText.Important" )
                        .Font( FAppStyle::Get().GetFontStyle( "FontAwesome.10" ) )
                        .Text( FEditorFontGlyphs::Plus )
                    ]
                    + SHorizontalBox::Slot()
                    .AutoWidth()
                    .Padding( 4, 0, 0, 0 )
                    [
                        SNew( STextBlock )
                        .TextStyle( FAppStyle::Get(), "NormalText.Important" )
                        .Text( LOCTEXT( "Add", "Add" ) )
                    ]
                ]
            ]
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
            [
                SNew( SBorder )
                .Padding( FMargin( 4.0f, 4.0f ) )
                [
                    SAssignNew( m_pSessionsListView, SMvnRemoteControlSessionsListView )
                    .ListItemsSource(&m_aSessionsData)
                    .SelectionMode(ESelectionMode::Single)
                	.OnGenerateRow(this, &SMvnRemoteControlClientPanel::MakeSessionListViewWidget)
                	.OnSelectionChanged(this, &SMvnRemoteControlClientPanel::OnSelectedSessionChanged)
                    .HeaderRow
                    (
                        SNew( SHeaderRow )
                        + SHeaderRow::Column( MvnRemoteControlListUI::InstanceColumnName )
                        .FillWidth( 20.f )
                        .DefaultLabel( LOCTEXT( "InstanceColumnHeaderName", "Instance" ) )
                        + SHeaderRow::Column( MvnRemoteControlListUI::TakeColumnName )
                        .FillWidth( 10.f )
                        .DefaultLabel( LOCTEXT( "TakeColumnHeaderName", "Take" ) )
                        + SHeaderRow::Column( MvnRemoteControlListUI::IPAddressColumnName )
                        .FillWidth( 15.f )
                        .DefaultLabel( LOCTEXT( "IPAddressColumnName", "IP Address" ) )
                        + SHeaderRow::Column( MvnRemoteControlListUI::RecordingColumnName )
                        .FillWidth( 25.f )
                        .DefaultLabel( LOCTEXT( "RecordingColumnHeaderName", "Status" ) )
                        + SHeaderRow::Column( MvnRemoteControlListUI::ElapsedTimeColumnName )
                        .FillWidth( 20.f )
                        .DefaultLabel( LOCTEXT( "ElapsedTimeColumnHeaderName", "Elapsed Time" ) )
                        + SHeaderRow::Column( MvnRemoteControlListUI::RemoveSessionColumnName )
                        .FillWidth( 10.f )
                        .DefaultLabel( LOCTEXT( "", "" ) )
                    )
                ]
            ]
            + SVerticalBox::Slot()
            .AutoHeight()
            .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
            [
                SNew( SHorizontalBox )
                + SHorizontalBox::Slot()
                .FillWidth( 0.1f )
                .HAlign( HAlign_Center )
                [
                    SNew( SCheckBox )
                    .Style( MvnRemoteControlUtils::GetStyleSet().Get(), "Xsens.RecordButton" )
                    .OnCheckStateChanged( this, &SMvnRemoteControlClientPanel::OnToggleRecording )
                    .IsChecked( this, &SMvnRemoteControlClientPanel::IsRecording )
                    .IsEnabled( this, &SMvnRemoteControlClientPanel::CanRecord )
                ]
                + SHorizontalBox::Slot()
                .FillWidth( 0.35f )
                .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                [
                    SNew( SVerticalBox )
                    +SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SHorizontalBox )
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( STextBlock )
                            .Text( LOCTEXT( "Start/Stop delay (seconds)", "Record Start/Stop delay (seconds)" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( SEditableTextBox )
                            .Text( this, &SMvnRemoteControlClientPanel::GetStartStopDelay )
                            .OnTextChanged_Raw( this, &SMvnRemoteControlClientPanel::GetStartStopDelayFromInput )
                        ]
                    ]
                    + SVerticalBox::Slot()
                    .AutoHeight()
                    .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                    [
                        SNew( SHorizontalBox )
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( STextBlock )
                            .Text( LOCTEXT( "TimeSync Enabled", "TimeSync Enabled" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.35f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( SCheckBox )
                            .OnCheckStateChanged( this, &SMvnRemoteControlClientPanel::OnToggleTimeSyncEnabled )
                            .IsChecked( this, &SMvnRemoteControlClientPanel::IsTimeSyncEnabled )
                        ]   
                    ]
                ]
            ]
        ]
    ];
}

void SMvnRemoteControlClientPanel::AddReferencedObjects( FReferenceCollector& Collector )
{

}

TSharedRef<ITableRow> SMvnRemoteControlClientPanel::MakeSessionListViewWidget( FMvnRemoteControlSessionUIEntryPtr Entry, const TSharedRef<STableViewBase>& OwnerTable )
{
    return SNew( SMvnRemoteControlSessionPanelRow, OwnerTable ).Entry( Entry ).Panel( this );
}

static bool IsUnsignedInteger(const FString& InString)
{
	bool bResult = !InString.IsEmpty();
	for (auto CharacterIter : InString)
	{
		if (!FChar::IsDigit(CharacterIter))
		{
			bResult = false;
			break;
		}
	}
	return bResult;
}


static bool IsValidIPv4Address(const FString& InString)
{
	FString Temp = InString;
	FString Str;

	for (int32 Index = 0; Index < 4; Index++)
	{
		// Split with '.' character
		if (Index == 3)
		{
			Str = Temp;
		}
		else if (!Temp.Split(TEXT("."), &Str, &Temp))
		{
			return false;
		}
		// Validate the number
		if (!IsUnsignedInteger(Str))
		{
			return false;
		}
		uint32 Value;
		LexFromString(Value, *Str);
		if (Value >= 256)
		{
			return false;
		}
	}
	return true;
}

FReply SMvnRemoteControlClientPanel::AddRemoteControlInstance()
{
    FIPv4Address ipAddress;
    bool bValidIP = IsValidIPv4Address(m_NewSessionIPAddress.ToString()) && FIPv4Address::Parse( m_NewSessionIPAddress.ToString(), ipAddress );
	bool bValidPort = IsUnsignedInteger(m_iNewSessionPort.ToString());

	uint32 Port = FCString::Atoi(*m_iNewSessionPort.ToString());	
	bValidPort &= ( Port > 1023 ) && ( Port < 65354 );

    if ( RemoteControlManager->IsPortReserved( Port ) )
    {
        m_NewSessionFeedback = FText::FromString( "Please enter a non reserved Port" );
    }
    else if ( bValidPort == false )
    {
        m_NewSessionFeedback = FText::FromString( "Please enter a valid Port" );
    }
    else if ( bValidIP == false )
    {
        m_NewSessionFeedback = FText::FromString( "Please enter a valid IP address" );
    }
    else if ( RemoteControlManager->CanCreateSession( ipAddress, Port ) == false )
    {
        m_NewSessionFeedback = FText::FromString( "Cannot create duplicate session" );
    }
    else
    {
        MvnRemoteControlSession* _pTestRemoteSession = RemoteControlManager->CreateSession( ipAddress, Port );
        TSharedPtr<FMvnRemoteControlSessionUIEntry> pNewSessionEntry = TSharedPtr<FMvnRemoteControlSessionUIEntry>( new FMvnRemoteControlSessionUIEntry( FGuid::NewGuid(), _pTestRemoteSession, RemoteControlManager ) );
        m_aSessionsData.Add( pNewSessionEntry );
        m_pSessionsListView->RequestListRefresh();
        m_pNewSessionComboButton->SetIsOpen(false);
        m_NewSessionFeedback = FText::FromString( "" );
		return FReply::Handled();
	}

	return FReply::Unhandled();
}

FReply SMvnRemoteControlClientPanel::RemoveRemoteControlSession( FMvnRemoteControlSessionUIEntryPtr _pSessionEntry )
{
    if ( _pSessionEntry )
    {
        RemoteControlManager->DestroySession( _pSessionEntry->GetSession() );
        m_aSessionsData.Remove( _pSessionEntry );
        m_pSessionsListView->RequestListRefresh();
    }
    return FReply::Handled();
}


FReply SMvnRemoteControlClientPanel::RemoveSelectedRemoteControlInstance()
{
    if ( m_pSelectedEntry )
    {
        RemoteControlManager->DestroySession(m_pSelectedEntry->GetSession());
        m_aSessionsData.Remove(m_pSelectedEntry);
        m_pSessionsListView->RequestListRefresh();
    }
    return FReply::Handled();
}

FReply SMvnRemoteControlClientPanel::UpdateSelectedSession()
{
    UE_LOG(LogTemp, Log, TEXT("Updaing Selected Session"));
    if ( m_pSelectedEntry )
    {
        MvnRemoteControlSession* pSession = m_pSelectedEntry->GetSession();
        FString strTake = m_SessionTake.ToString();
        FString strSessionName = m_SessionName.ToString();
        int iTakeCount = atoi( TCHAR_TO_UTF8( *strTake ) );
        pSession->SetCaptureName( strSessionName, iTakeCount );

        m_SessionFeedback = FText::FromString("");
        FIPv4Address ipAddress;
        bool bValidIP = FIPv4Address::Parse( m_SessionIPAddress.ToString(), ipAddress );
        bool bValidPort = ( m_iSessionPort > 1023 ) && ( m_iSessionPort < 65354 );

        if ( bValidPort == false )
        {
            m_SessionFeedback = FText::FromString( "Please enter a valid Port" );
            UE_LOG(LogTemp, Log, TEXT("Port is invalid"));
        }
        else if ( bValidIP == false )
        {
            m_SessionFeedback = FText::FromString( "Please enter a valid IP address" );
            UE_LOG(LogTemp, Log, TEXT("IP is invalid"));
        }
        else
        {
            /// change IP address and port
            pSession->SetDestinationIPAddress( ipAddress );
            pSession->SetDestinationPort( m_iSessionPort );
        }

        m_pSessionsListView->RequestListRefresh();
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("Selected Entry is null. Aborting"));
    }
    return FReply::Handled();
}

FReply SMvnRemoteControlClientPanel::StartRecording()
{
    RemoteControlManager->CaptureStartAll( m_dDefaultCaptureStartStopDelay );
    return FReply::Handled();
}

FReply SMvnRemoteControlClientPanel::StopRecording()
{
    RemoteControlManager->CaptureStopAll( m_dDefaultCaptureStartStopDelay );
    return FReply::Handled();
}


FText SMvnRemoteControlClientPanel::GetSessionName() const
{
    return m_SessionName;
}

void SMvnRemoteControlClientPanel::GetSessionNameFromInput( const FText& Text )
{
    m_SessionName = Text;
}

FText SMvnRemoteControlClientPanel::GetTake() const
{
    return m_SessionTake;
}

void SMvnRemoteControlClientPanel::GetTakeFromInput( const FText& Text )
{
    m_SessionTake = Text;
}

FText SMvnRemoteControlClientPanel::GetNewSessionIPAddress() const
{
    return m_NewSessionIPAddress;
}

void SMvnRemoteControlClientPanel::GetNewSessionIPAddressFromInput( const FText& Text )
{
    m_NewSessionIPAddress = Text;
}

FText SMvnRemoteControlClientPanel::GetNewSessionPort() const
{
    return m_iNewSessionPort;
}

void SMvnRemoteControlClientPanel::GetNewSessionPortFromInput( const FText& Text )
{
    m_iNewSessionPort = Text;
}


FText SMvnRemoteControlClientPanel::GetNewSessionFeedback() const
{
    return m_NewSessionFeedback;
}

FText SMvnRemoteControlClientPanel::GetSessionIPAddress() const
{
    return m_SessionIPAddress;
}

void SMvnRemoteControlClientPanel::GetSessionIPAddressFromInput( const FText& Text )
{
    m_SessionIPAddress = Text;
}

FText SMvnRemoteControlClientPanel::GetSessionPort() const
{
    FString str = FString::FromInt( m_iSessionPort );
    if ( m_iSessionPort == 0 )
    {
        str = "";
    }
    return FText::FromString( str );
}

void SMvnRemoteControlClientPanel::GetSessionPortFromInput( const FText& Text )
{
    m_iSessionPort = FCString::Atoi( *Text.ToString() );
}


FText SMvnRemoteControlClientPanel::GetSessionFeedback() const
{
    return m_SessionFeedback;
}



void SMvnRemoteControlClientPanel::OnSelectedSessionChanged( FMvnRemoteControlSessionUIEntryPtr Entry, ESelectInfo::Type SelectionType ) 
{
    if ( bSelectionChangedGuard )
    {
        return;
    }
    TGuardValue<bool> ReentrantGuard( bSelectionChangedGuard, true );
    if ( Entry.IsValid() )
    {
        m_SessionName = FText::FromString( Entry->GetInstance() );
        m_SessionTake = FText::FromString( Entry->GetTake() );

        FIPv4Address destIPAddess = Entry->GetDestinationIPAddress();
        FString strIPAddress = FString::Printf( TEXT( "%d.%d.%d.%d" ), destIPAddess.A, destIPAddess.B, destIPAddess.C, destIPAddess.D );
        int iDestPort = Entry->GetDestinationPort();

        m_SessionIPAddress = FText::FromString( strIPAddress );
        m_iSessionPort = iDestPort;

        if ( m_pSelectedEntry )
        {
            m_pSelectedEntry->SetSelected( false );
        }

        Entry->SetSelected( true );

        m_pSelectedEntry = Entry;


    }
    else
    {
        m_pSelectedEntry = nullptr;
    }
}

void SMvnRemoteControlClientPanel::OnToggleRecording(ECheckBoxState)
{
    if ( ! RemoteControlManager->IsAnyRecording() )
    {
        RemoteControlManager->CaptureStartAll( m_dDefaultCaptureStartStopDelay );
    }
    else
    {
        RemoteControlManager->CaptureStopAll( m_dDefaultCaptureStartStopDelay );
    }
}

ECheckBoxState SMvnRemoteControlClientPanel::IsRecording() const
{
    return RemoteControlManager->IsAnyRecording() ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
}

bool SMvnRemoteControlClientPanel::CanRecord() const
{
    return m_aSessionsData.Num() > 0;
}

void SMvnRemoteControlClientPanel::OnToggleTimeSyncEnabled( ECheckBoxState _checkBoxState )
{
    m_bTimeSyncEnabled = ( _checkBoxState == ECheckBoxState::Checked );
    RemoteControlManager->SetTimeSyncEnabled( m_bTimeSyncEnabled );
}

ECheckBoxState SMvnRemoteControlClientPanel::IsTimeSyncEnabled() const
{
    return m_bTimeSyncEnabled ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
}

TSharedRef<SWidget> SMvnRemoteControlClientPanel::MakeAddSessionWidget()
{
    TSharedRef<SWidget>  newSessionWidget = SNew( SMvnRemoteControlNewSessionWidget ).Panel(this);
    return newSessionWidget;
}

FMvnRemoteControlSessionUIEntryPtr SMvnRemoteControlClientPanel::GetSelectedEntry() const
{
    return m_pSelectedEntry;
}

FText SMvnRemoteControlClientPanel::GetStartStopDelay() const
{
    return FText::FromString( FString::Printf( L"%f", m_dDefaultCaptureStartStopDelay ) );
}

void SMvnRemoteControlClientPanel::GetStartStopDelayFromInput( const FText& Text )
{
    m_dDefaultCaptureStartStopDelay = FCString::Atod( *Text.ToString() );
}

#undef LOCTEXT_NAMESPACE