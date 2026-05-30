// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnSourceFactory.h"
#include "LiveLinkMvnSource.h"
#include "LiveLinkMvnSourceEditor.h"

#define LOCTEXT_NAMESPACE "LiveLinkMvnSourceFactory"

FText ULiveLinkMvnSourceFactory::GetSourceDisplayName() const
{
	return LOCTEXT("SourceDisplayName", "Xsens Live Source");
}

FText ULiveLinkMvnSourceFactory::GetSourceTooltip() const
{
	return LOCTEXT("SourceTooltip", "Creates a connection to a MVN Live Stream");
}

TSharedPtr<SWidget> ULiveLinkMvnSourceFactory::BuildCreationPanel(FOnLiveLinkSourceCreated InOnLiveLinkSourceCreated) const
{
	return SNew(SLiveLinkMvnSourceEditor)
		.OnSourceSelected(SLiveLinkMvnSourceEditor::FOnMVNLiveLinkSourceSelected::CreateUObject(this, &ULiveLinkMvnSourceFactory::OnSourceSelected, InOnLiveLinkSourceCreated));
}

TSharedPtr<ILiveLinkSource> ULiveLinkMvnSourceFactory::CreateSource(const FString& ConnectionString) const
{
	TArray<FString> ParsedName;
	ConnectionString.ParseIntoArray(ParsedName, TEXT("="));

	TSharedPtr<FLiveLinkMvnSource> SharedPtr = MakeSource(FText::FromString(ParsedName[1]), FText::FromString(ParsedName[1]), FMessageAddress(), 0);
	return SharedPtr;
}

TSharedPtr<FLiveLinkMvnSource> ULiveLinkMvnSourceFactory::MakeSource(const FText& Name, const FText& MachineName, const FMessageAddress& Address, double TimeOffset) const
{
	return MakeShared<FMVNLiveLinkMessageBusSource>(Name, MachineName, Address, TimeOffset);
}

void ULiveLinkMvnSourceFactory::OnSourceSelected(FProviderPollResultPtr SelectedSource, FOnLiveLinkSourceCreated InOnLiveLinkSourceCreated) const
{
	TSharedPtr<FLiveLinkMvnSource> SharedPtr = MakeSource(FText::FromString(SelectedSource->Name), FText::FromString(SelectedSource->MachineName), SelectedSource->Address, SelectedSource->MachineTimeOffset);
	if (SharedPtr)
	{
		FString ConnectionString = FString::Printf(TEXT("Name=\"%s\""), *(SelectedSource->Name));
		InOnLiveLinkSourceCreated.ExecuteIfBound(SharedPtr, MoveTemp(ConnectionString));
	}
}

#undef LOCTEXT_NAMESPACE
