// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnMetadataService.h"

#define LOCTEXT_NAMESPACE "FLiveLinkMvnMetadataService"

FLiveLinkMvnMetadataService& FLiveLinkMvnMetadataService::getInstance() 
{
	static FLiveLinkMvnMetadataService instance;
	return instance;
}

FLiveLinkMvnMetadataService::FLiveLinkMvnMetadataService()
{

}

void FLiveLinkMvnMetadataService::SetSegmentNames(FGuid SourceId, const FLiveLinkSubjectName& AvatarId, const TArray<FString>& SegmentNames)
{
	TArray<FText> TextNames;
	for (auto Name : SegmentNames)
	{
		TextNames.Add(FText::AsCultureInvariant(Name));
	}

	EnsureMetadata(SourceId, AvatarId);
	Metadata[SourceId][AvatarId].SegmentNames = TextNames;
}

void FLiveLinkMvnMetadataService::SetSegmentCount(FGuid SourceId, const FLiveLinkSubjectName& AvatarId, int SegmentsCount)
{
	EnsureMetadata(SourceId, AvatarId);
	Metadata[SourceId][AvatarId].SegmentsCount = SegmentsCount;
}

int32 FLiveLinkMvnMetadataService::GetSegmentsCount(FGuid SourceId, const FLiveLinkSubjectName& AvatarId)
{
	if (!HasSegmentNames(SourceId, AvatarId))
	{
		return 0;
	}
	return Metadata[SourceId][AvatarId].SegmentsCount;

}

bool FLiveLinkMvnMetadataService::GetSegmentNames(FGuid SourceId, const FLiveLinkSubjectName& AvatarId, TArray<FText>& OutSegmentNames) const
{
	if (!HasSegmentNames(SourceId, AvatarId))
	{
		return false;
	}
	OutSegmentNames.Append(Metadata[SourceId][AvatarId].SegmentNames);
	return true;
}

bool FLiveLinkMvnMetadataService::HasSegmentNames(FGuid SourceId, const FLiveLinkSubjectName& AvatarId) const
{
	return Metadata.Contains(SourceId) && Metadata[SourceId].Contains(AvatarId);
}

void FLiveLinkMvnMetadataService::EnsureMetadata(FGuid SourceId, const FLiveLinkSubjectName& AvatarId)
{
	if (Metadata.Contains(SourceId))
	{
		FSourceMetadata& Existing = Metadata[SourceId];
		if (!Existing.Contains(AvatarId))
		{
			FSubjectMetadata NewSubjectMetadata;
			Existing.Add(AvatarId, NewSubjectMetadata);
		}
	}
	else
	{
		FSourceMetadata NewSourceMetadata;
		FSubjectMetadata NewSubjectMetadata;
		NewSourceMetadata.Add(AvatarId, NewSubjectMetadata);
		Metadata.Add(SourceId, NewSourceMetadata);
	}
}

#undef LOCTEXT_NAMESPACE