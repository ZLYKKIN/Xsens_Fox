// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "LiveLinkMvnEditorModule.h"

#include "AssetToolsModule.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyEditorDelegates.h"
#include "IAssetTools.h"
#include "ISettingsModule.h"
#include "ToolMenus.h"

#include "LiveLinkMvnRetargetAsset.h"
#include "LiveLinkMvnSettings.h"
#include "Customizations/LiveLinkMvnRetargetAssetCustomization.h"
#include "Customizations/XsensRemapAction.h"
#include "Customizations/LiveLinkMvnEditorUtilities.h"
#include "LevelEditor.h"
#include "Framework/MultiBox/MultiBoxBuilder.h"
#include "Engine/Selection.h"
#include "Styling/SlateStyle.h"
#include "WorkspaceMenuStructure.h"
#include "WorkspaceMenuStructureModule.h"
#include "RemoteControl/MvnRemoteControlClientPanel.h"
#include "MvnRemoteControlManager.h"
#include "Styling/SlateStyle.h"
#include "Styling/SlateStyleRegistry.h"
#include "Widgets/Docking/SDockTab.h"


#define LOCTEXT_NAMESPACE "FLiveLinkMvnPluginEditorModule"


static const FName MvnRemoteControlTabName(TEXT("Mvn Remote Control"));

namespace LiveLinkMvnEditorUtils
{
    /// spawn tab for mvn remote control
    static TSharedRef<SDockTab> SpawnRemoteControlTab( const FSpawnTabArgs& SpawnTabArgs, TSharedPtr<FSlateStyleSet> InStyleSet, MvnRemoteControlManager* _remoteControlManager );
}

void FLiveLinkMvnPluginEditorModule::StartupModule()
{
	MvnRemoteControlUtils::InitStyleSet();

	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked< FPropertyEditorModule >(TEXT("PropertyEditor"));
	PropertyModule.RegisterCustomClassLayout(ULiveLinkMvnRetargetAsset::StaticClass()->GetFName(), FOnGetDetailCustomizationInstance::CreateStatic(&LiveLinkMvnRetargetAssetCustomization::MakeInstance));

	// Registering our custom Asset Action.
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

	TSharedRef<FXsensRemapAction> RemapAction = MakeShareable(new FXsensRemapAction());
	// We need to grab an original Action menu and set it to our override, so that we don't completely override it.
	TWeakPtr<IAssetTypeActions> AssetActions = AssetTools.GetAssetTypeActionsForClass(USkeleton::StaticClass());
	if (AssetActions.IsValid())
	{
		RemapAction->SetParent(AssetActions.Pin());
	}
	AssetTools.RegisterAssetTypeActions(RemapAction);
	CreatedAssetTypeActions.Add(RemapAction);

	UToolMenus* ToolMenus = UToolMenus::Get();
	if (ToolMenus)
	{
		UToolMenu* ActorContextMenu = ToolMenus->ExtendMenu("LevelEditor.ActorContextMenu");
		if (ActorContextMenu != nullptr)
		{
			ActorContextMenu->AddDynamicSection("XsensActorContextMenuDynamic", FNewToolMenuDelegate::CreateLambda([](UToolMenu* InMenu)
			{
				TArray<AActor*> SelectedActors;
				GEditor->GetSelectedActors()->GetSelectedObjects<AActor>(SelectedActors);
				if (SelectedActors.Num() == 1) {
					FToolMenuSection& Section = InMenu->AddSection("XsensActorControls", LOCTEXT("XsensActorHeading", "Xsens Actor Actions"));

					Section.AddSubMenu(
						"XsensSourceSubMenu",
						LOCTEXT("XsensAttachActor", "Attach to LiveLink Mvn"),
						FText::GetEmpty(),
						FNewToolMenuDelegate::CreateStatic(&FLiveLinkMvnEditorUtilities::AddLiveLinkSourcePicker));
				}
			}));
		}
	}

	RemoteControlManager = new MvnRemoteControlManager();
	RegisterRemoteControlTabSpawner();
	RegisterSettings();
}

void FLiveLinkMvnPluginEditorModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
	UnregisterRemoteControlTabSpawner();

	if (FModuleManager::Get().IsModuleLoaded("AssetTools"))
	{
		// Unregister skeleton assets actions from the AssetTools
		IAssetTools& AssetTools = FModuleManager::GetModuleChecked<FAssetToolsModule>("AssetTools").Get();
		for (int32 i = 0; i < CreatedAssetTypeActions.Num(); ++i)
		{
			AssetTools.UnregisterAssetTypeActions(CreatedAssetTypeActions[i].ToSharedRef());
		}
	}

	CreatedAssetTypeActions.Empty();
	delete RemoteControlManager;
}

void FLiveLinkMvnPluginEditorModule::RegisterRemoteControlTabSpawner()
{
    if ( bHasRegisteredRemoteControlTabSpawner )
    {
        UnregisterRemoteControlTabSpawner();
    }


    FTabSpawnerEntry& SpawnerEntry = FGlobalTabmanager::Get()->RegisterNomadTabSpawner( MvnRemoteControlTabName, FOnSpawnTab::CreateStatic( &LiveLinkMvnEditorUtils::SpawnRemoteControlTab, MvnRemoteControlUtils::GetStyleSet(), RemoteControlManager ) )
        .SetDisplayName( LOCTEXT( "MvnRemoteControlTitle", "Mvn Remote Control" ) )
        .SetTooltipText( LOCTEXT( "MvnRemoteControlTooltipText", "Open the Mvn Remote control tab." ))
		.SetIcon(FSlateIcon(MvnRemoteControlUtils::GetStyleSet()->GetStyleSetName(), "Xsens.SmallIcon"));

    const IWorkspaceMenuStructure& MenuStructure = WorkspaceMenu::GetMenuStructure();
    SpawnerEntry.SetGroup( MenuStructure.GetLevelEditorCategory() );

    bHasRegisteredRemoteControlTabSpawner = true;
}

void FLiveLinkMvnPluginEditorModule::RegisterSettings()
{
	if (ISettingsModule* SettingsModule = FModuleManager::GetModulePtr<ISettingsModule>("Settings"))
	{
		SettingsModule->RegisterSettings("Project", "Plugins", "LiveLinkMvnPlugin",
			LOCTEXT("EditorSettingsName", "LiveLinkMvnPlugin"),
			LOCTEXT("EditorSettingsDescription", "Configure LiveLinkMvnPlugin."),
			GetMutableDefault<ULiveLinkMvnSettings>()
		);
	}
}

void FLiveLinkMvnPluginEditorModule::UnregisterSettings()
{
	// unregister settings
	ISettingsModule* SettingsModule = FModuleManager::GetModulePtr<ISettingsModule>("Settings");
	if (SettingsModule != nullptr)
	{
		SettingsModule->UnregisterSettings("Project", "Plugins", "LiveLinkMvnPlugin");
	}
}

void FLiveLinkMvnPluginEditorModule::UnregisterRemoteControlTabSpawner()
{
    FGlobalTabmanager::Get()->UnregisterNomadTabSpawner( MvnRemoteControlTabName );
    bHasRegisteredRemoteControlTabSpawner = false;
}


TSharedRef<SDockTab> LiveLinkMvnEditorUtils::SpawnRemoteControlTab( const FSpawnTabArgs& SpawnTabArgs, TSharedPtr<FSlateStyleSet> InStyleSet, MvnRemoteControlManager* _remoteControlManager )
{
    const TSharedRef<SDockTab> RemoteControlTab =
        SNew( SDockTab )
        .TabRole( ETabRole::NomadTab );

	RemoteControlTab->SetContent(SNew(SMvnRemoteControlClientPanel, _remoteControlManager));

    return RemoteControlTab;
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FLiveLinkMvnPluginEditorModule, LiveLinkMvnPluginEditor)