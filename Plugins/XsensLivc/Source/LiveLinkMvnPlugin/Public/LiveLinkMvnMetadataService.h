// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Misc/Guid.h"
#include "LiveLinkTypes.h"

class LIVELINKMVNPLUGIN_API FLiveLinkMvnMetadataService {

public:

	struct FSubjectMetadata
	{
		int32 SegmentsCount = 0;
		TArray<FText> SegmentNames;
	};

	static FLiveLinkMvnMetadataService& getInstance();

	void SetSegmentCount(FGuid SourceId, const FLiveLinkSubjectName& AvatarId, int SegmentsCount);
	void SetSegmentNames(FGuid SourceId, const FLiveLinkSubjectName& AvatarId, const TArray<FString>& SegmentNames);

	int32 GetSegmentsCount(FGuid SourceId, const FLiveLinkSubjectName& AvatarId);
	bool GetSegmentNames(FGuid SourceId, const FLiveLinkSubjectName& AvatarId, TArray<FText>& OutSegmentNames) const;
	bool HasSegmentNames(FGuid SourceId, const FLiveLinkSubjectName& AvatarId) const;

protected:

	FLiveLinkMvnMetadataService();

	typedef TMap<FLiveLinkSubjectName, FSubjectMetadata> FSourceMetadata;
	TMap<FGuid, FSourceMetadata> Metadata;

	void EnsureMetadata(FGuid SourceId, const FLiveLinkSubjectName& AvatarId);
};