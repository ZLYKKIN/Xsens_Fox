// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "RemoteControl/MvnRemoteControlNewSessionWidget.h"
#include "LiveLinkMessages.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"

#include "MessageEndpointBuilder.h"
#include "RemoteControl/MvnRemoteControlClientPanel.h"

#define LOCTEXT_NAMESPACE "MvnRemoteControlNewSessionWidget"

SMvnRemoteControlNewSessionWidget::~SMvnRemoteControlNewSessionWidget()
{
}

void SMvnRemoteControlNewSessionWidget::Construct(const FArguments& Args)
{
    SMvnRemoteControlClientPanel* pPanel = Args._Panel;
	// Clear the previous error message
	pPanel->m_NewSessionFeedback = FText::FromString("");

	ChildSlot
	[
		SNew(SBox)
		.WidthOverride(250)
		[
            SNew( SBorder )
            .Padding( FMargin( 4.0f, 4.0f ) )
            [
			    SNew(SVerticalBox)
			    + SVerticalBox::Slot()
			    .AutoHeight()
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
                            .Text( LOCTEXT( "IP Address", "IP Address" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.5f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( SEditableTextBox )
                            .Text( pPanel, &SMvnRemoteControlClientPanel::GetNewSessionIPAddress )
                            .OnTextChanged_Raw( pPanel, &SMvnRemoteControlClientPanel::GetNewSessionIPAddressFromInput )
                        ]
                    ]
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
                            .Text( LOCTEXT( "Port", "Port" ) )
                        ]
                        + SHorizontalBox::Slot()
                        .FillWidth( 0.5f )
                        .Padding( FMargin( 0.0f, 4.0f, 0.0f, 0.0f ) )
                        [
                            SNew( SEditableTextBox )
                            .Text( pPanel, &SMvnRemoteControlClientPanel::GetNewSessionPort )
                            .OnTextChanged_Raw( pPanel, &SMvnRemoteControlClientPanel::GetNewSessionPortFromInput )
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
                                .Text( pPanel, &SMvnRemoteControlClientPanel::GetNewSessionFeedback )
                            ]
                        ]
                    ]
                ]
			    + SVerticalBox::Slot()
			    .HAlign(HAlign_Right)
			    .AutoHeight()
			    [
				    SNew(SButton)
				    .OnClicked( pPanel, &SMvnRemoteControlClientPanel::AddRemoteControlInstance )
				    [
					    SNew(STextBlock)
					    .Text(LOCTEXT("Add", "Add"))
				    ]
			    ]
		    ]
        ]
	];
}


#undef LOCTEXT_NAMESPACE
