// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "RemoteControl/MvnRemoteControlUtils.h"
#include "Styling/SlateStyle.h"
#include "Styling/SlateStyleRegistry.h"
#include "Interfaces/IPluginManager.h"

#define IMAGE_PLUGIN_BRUSH( RelativePath, ... ) FSlateImageBrush( MvnRemoteControlUtils::InPluginContent( RelativePath, ".png" ), __VA_ARGS__ )

namespace MvnRemoteControlUtils
{
    static TSharedPtr<FSlateStyleSet> StyleSet;
};

TSharedPtr<FSlateStyleSet> MvnRemoteControlUtils::GetStyleSet()
{
    return StyleSet;
}

FString MvnRemoteControlUtils::InPluginContent( const FString& RelativePath, const ANSICHAR* Extension )
{
    static FString ContentDir = IPluginManager::Get().FindPlugin( TEXT( "LiveLinkMvnPlugin" ) )->GetContentDir();
    return ( ContentDir / RelativePath ) + Extension;
}

void MvnRemoteControlUtils::InitStyleSet()
{
    static FName LiveLinkMvnStyle( TEXT( "LiveLinkMvnStyle" ) );
    StyleSet = MakeShared<FSlateStyleSet>( LiveLinkMvnStyle );

    const FVector2D Icon16x16( 16.0f, 16.0f );
    const FVector2D Icon64x64( 64.0f, 64.0f );
    const FVector2D Icon48x48( 48.0f, 48.0f );

    StyleSet->SetContentRoot( FPaths::EngineContentDir() / TEXT( "Editor/Slate" ) );
    StyleSet->SetCoreContentRoot( FPaths::EngineContentDir() / TEXT( "Slate" ) );
    StyleSet->Set( "Xsens.SmallIcon", new IMAGE_PLUGIN_BRUSH( TEXT( "xsens_16x16" ), Icon16x16 ) );
    StyleSet->Set( "Xsens.LargeIcon", new IMAGE_PLUGIN_BRUSH( TEXT( "xsens_64x64" ), Icon64x64 ) );

    FCheckBoxStyle RecordButton = FCheckBoxStyle()
        .SetUncheckedImage( IMAGE_PLUGIN_BRUSH( TEXT( "RecordButton_Idle" ), Icon48x48 ) )
        .SetUncheckedHoveredImage( IMAGE_PLUGIN_BRUSH( TEXT( "RecordButton_Hovered" ), Icon48x48 ) )
        .SetUncheckedPressedImage( IMAGE_PLUGIN_BRUSH( TEXT( "RecordButton_Pressed" ), Icon48x48 ) )
        .SetCheckedImage( IMAGE_PLUGIN_BRUSH( TEXT( "StopButton_Idle" ), Icon48x48 ) )
        .SetCheckedHoveredImage( IMAGE_PLUGIN_BRUSH( TEXT( "StopButton_Hovered" ), Icon48x48 ) )
        .SetCheckedPressedImage( IMAGE_PLUGIN_BRUSH( TEXT( "StopButton_Pressed" ), Icon48x48 ) )
        .SetPadding( FMargin( 0, 0, 0, 1 ) );

    StyleSet->Set( "Xsens.RecordButton", RecordButton );
    

    FSlateStyleRegistry::RegisterSlateStyle( *MvnRemoteControlUtils::StyleSet.Get() );
}


