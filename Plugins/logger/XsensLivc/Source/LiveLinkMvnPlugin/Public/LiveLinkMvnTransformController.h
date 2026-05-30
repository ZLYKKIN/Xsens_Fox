// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "LiveLinkControllerBase.h"

#include "LiveLinkMvnTransformController.generated.h"

UCLASS()
class LIVELINKMVNPLUGIN_API ULiveLinkMvnTransformController : public ULiveLinkControllerBase
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, Category = "LiveLinkMVN", meta = (ShowOnlyInnerProperties))
	int32 SegmentIndex = 0;

public:
	//~Begin ULiveLinkControllerBase interface
	//virtual void OnEvaluateRegistered() override;
	virtual void Tick(float DeltaTime, const FLiveLinkSubjectFrameData& SubjectData) override;
	virtual bool IsRoleSupported(const TSubclassOf<ULiveLinkRole>& RoleToSupport) override;
	virtual TSubclassOf<UActorComponent> GetDesiredComponentClass() const override;
	//virtual void SetAttachedComponent(UActorComponent* ActorComponent) override;
	//~End ULiveLinkControllerBase interface
};