// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkGameInstance.h"
#include "Features/IModularFeatures.h"
#include "LiveLinkClient.h"
#include "LiveLinkMvnSource.h"
#include "..\Public\LiveLinkGameInstance.h"
//#include "XSensLLWrapper\MVNWrapper.h"

//static ULiveLinkGameInstance* LLInstance;

static void SetPortNumber(const TArray< FString >& Args);

//FAutoConsoleCommand TestMvnCommand(
//	TEXT("mvn.Test"),
//	TEXT("test making connection"),
//	FConsoleCommandWithArgsDelegate::CreateStatic(&SetPortNumber));

static void SetPortNumber(const TArray< FString >& Args)
{
	uint16 NewPortNumber = 9763;
	if (Args.Num() > 0 && Args[0].Len() > 0)
	{
		NewPortNumber = FCString::Atoi(*Args[0]);
	}
	FLiveLinkClient* Client = &IModularFeatures::Get().GetModularFeature<FLiveLinkClient>(FLiveLinkClient::ModularFeatureName);
	check(Client);
	//Client->RemoveSource(LLInstance->CurrentConnetion);
	//LLInstance->CurrentConnetion = Client->AddSource(MakeShared<FLiveLinkMvnSource>(NewPortNumber, true));
	//LLInstance->PortNumber = NewPortNumber;
}

ULiveLinkGameInstance::ULiveLinkGameInstance() : PortNumber(9763)
{
	//LLInstance = this;
}

void ULiveLinkGameInstance::OnStart()
{
	Super::OnStart();
	//MvnWrapper::Start();

#if WITH_EDITOR
	return;
#endif

	//SetPortNumber({ FString::FromInt(PortNumber) });
}

void ULiveLinkGameInstance::Shutdown()
{
	Super::Shutdown();

	//MvnWrapper::ShutDown();
	return;
}

