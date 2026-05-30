// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "LiveLinkSourceFactory.h"
#include "LiveLinkMessages.h"
#include "LiveLinkMessageBusSourceFactory.h"
#include "MVNLiveLinkMessageBusSource.h"
#include "LiveLinkMvnSourceFactory.generated.h"

class SLiveLinkMvnSourceEditor;

//UCLASS()
class FLiveLinkMvnEditorPanel //: public SPanel
{
//	GENERATED_BODY()
	
	UPROPERTY(EditAnywhere, Category = "LiveLink")
	int Port;
	
	FLiveLinkMvnEditorPanel() : Port(9763) {}
};

UCLASS()
class ULiveLinkMvnSourceFactory : public ULiveLinkSourceFactory
{
public:

	GENERATED_BODY()

	virtual FText GetSourceDisplayName() const;
	virtual FText GetSourceTooltip() const;
	virtual EMenuType GetMenuType() const override { return EMenuType::SubPanel; }
	virtual TSharedPtr<SWidget> BuildCreationPanel(FOnLiveLinkSourceCreated OnLiveLinkSourceCreated) const override;
	virtual TSharedPtr<ILiveLinkSource> CreateSource(const FString& ConnectionString) const override;
	TSharedPtr<class FLiveLinkMvnSource> MakeSource(const FText& Name, const FText& MachineName, const FMessageAddress& Address, double TimeOffset) const;

private:

	void OnSourceSelected(FProviderPollResultPtr SelectedSource, FOnLiveLinkSourceCreated InOnLiveLinkSourceCreated) const;
};
