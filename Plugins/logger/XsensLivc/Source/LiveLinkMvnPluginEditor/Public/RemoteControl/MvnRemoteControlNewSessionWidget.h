// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/SListView.h"
#include "IMessageContext.h"
#include "MessageEndpoint.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Misc/Guid.h"

class SMvnRemoteControlClientPanel;

class SMvnRemoteControlNewSessionWidget : public SCompoundWidget
{
public:

	SLATE_BEGIN_ARGS( SMvnRemoteControlNewSessionWidget ) {}
		SLATE_ARGUMENT( SMvnRemoteControlClientPanel*, Panel )
	SLATE_END_ARGS()
	
	~SMvnRemoteControlNewSessionWidget();

	void Construct(const FArguments& Args);

};

