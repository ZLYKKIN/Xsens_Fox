// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

using UnrealBuildTool;

public class XSensLLWrapper : ModuleRules
{
	public XSensLLWrapper(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// ... add other public dependencies that you statically link with here ...
				"Networking",
				"Sockets",
				"XmlParser",
				"LiveLinkInterface",
				"LiveLinkMessageBusFramework",
                "LiveLink",
				"LiveLinkComponents",
				"Networking",
				"LiveLinkMvnPlugin",
#if UE_5_0_OR_LATER
				"LiveLinkAnimationCore",
#endif
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
                "LiveLink",
				"LiveLinkComponents",
				// ... add private dependencies that you statically link with here ...	
			}
            );
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
		// possible fix packaging issue (XUU-113)
		PrecompileForTargets = PrecompileTargetsType.Any;
	}
}
