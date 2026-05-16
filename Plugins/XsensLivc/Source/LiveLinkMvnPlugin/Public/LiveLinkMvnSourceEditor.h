// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/SListView.h"
#include "IMessageContext.h"
#include "MessageEndpoint.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Misc/Guid.h"
#include <LiveLinkMessageBusFinder.h>

struct FLiveLinkPongMessage;
struct FMessageAddress;
class ITableRow;
class STableViewBase;



class SLiveLinkMvnSourceEditor : public SCompoundWidget
{
public:
	
	DECLARE_DELEGATE_OneParam(FOnMVNLiveLinkSourceSelected, FProviderPollResultPtr);

	SLATE_BEGIN_ARGS(SLiveLinkMvnSourceEditor){}
		SLATE_EVENT(FOnMVNLiveLinkSourceSelected, OnSourceSelected)
	SLATE_END_ARGS()
	
	~SLiveLinkMvnSourceEditor();

	void Construct(const FArguments& Args);
	virtual void Tick(const FGeometry& AllottedGeometry, const double InCurrentTime, const float InDeltaTime);
	
private:

	TSharedPtr<SListView<TSharedPtr<struct FMVNLiveLinkSource>>> ListView;	//List view slate widget
	TArray<TSharedPtr<struct FMVNLiveLinkSource>> Sources;					//List of all sources
	TArray<FProviderPollResultPtr> Results;									//Populated Results
	FProviderPollResultPtr SelectedResult;									//Selected Source
	FOnMVNLiveLinkSourceSelected OnSourceSelected;							//Delegate to call on source selection

	// Time since our UI was last ticked, allow us to refresh if we haven't been onscreen for a while
	double LastTickTime;

	void OnSelectionChanged(TSharedPtr<struct FMVNLiveLinkSource> Source, ESelectInfo::Type Type);
	TSharedRef<class ITableRow> CreateListView(TSharedPtr<struct FMVNLiveLinkSource> Source, const TSharedRef<class STableViewBase>& TableView) const;


};

