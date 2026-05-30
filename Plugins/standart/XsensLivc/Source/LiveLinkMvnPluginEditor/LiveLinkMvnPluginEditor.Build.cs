// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

using UnrealBuildTool;

public class LiveLinkMvnPluginEditor : ModuleRules
{
    public LiveLinkMvnPluginEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        PrivateIncludePaths.AddRange(
            new string[] {
                "LiveLinkMvnPluginEditor/Private",
                // ... add other private include paths required here ...
            }
            );


        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                // ... add other public dependencies that you statically link with here ...
                "CoreUObject",
                "Engine",
                "Projects",
                "Networking",
                "DetailCustomizations",
                "LiveLinkMvnPlugin",
                "LiveLinkInterface",
                "BlueprintGraph",
                "AnimGraph",
                "Persona",
                "AssetTools",
                "UnrealEd",
                "ToolMenus",
                "LiveLinkInterface",
                "LiveLinkMessageBusFramework",
                "LiveLink",
                "LiveLinkComponents",
                "LiveLinkEditor",
            }
            );


        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "WorkspaceMenuStructure",
                "EditorStyle",
                "PropertyEditor",
                "SlateCore",
                "Slate",
                "InputCore",
                "AssetTools",
                "UnrealEd",
                "ToolMenus",
                "LiveLinkComponents",
                "LiveLinkEditor",
            }
            );


        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
                // ... add any modules that your module loads dynamically here ...
            }
            );
    }
}
