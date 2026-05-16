// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Widgets/SCompoundWidget.h"

#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/Views/SListView.h"
#include "Widgets/Views/STreeView.h"
#include "EditorUndoClient.h"

class STableViewBase;
class FSlateStyleSet;

namespace MvnRemoteControlUtils
{
    FString InPluginContent( const FString& RelativePath, const ANSICHAR* Extension );

    void InitStyleSet();

    TSharedPtr<FSlateStyleSet> GetStyleSet();
}
