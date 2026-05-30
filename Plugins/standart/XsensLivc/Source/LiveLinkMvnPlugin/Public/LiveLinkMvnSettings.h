

#pragma once

#include "CoreMinimal.h"
#include "LiveLinkMvnSettings.generated.h"

/**
 * 
 */
UCLASS(config = Game, defaultconfig)
class LIVELINKMVNPLUGIN_API ULiveLinkMvnSettings : public UObject
{
	GENERATED_BODY()
	
public:

	ULiveLinkMvnSettings();

	UPROPERTY(config, EditAnywhere, AdvancedDisplay, Category = "LiveLink", meta = (ForceUnits = s))
	double MessageBusTimeBeforeRemovingInactiveSourceOverride;

	double GetMessageBusTimeBeforeRemovingDeadSourceOverride() const { return MessageBusTimeBeforeRemovingInactiveSourceOverride; }
	
};
