// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once
#include "XsensMappingEnum.generated.h"

UENUM(BlueprintType)
enum class EXsensMapping : uint8 {
	Root = 0		UMETA(DisplayName = "Root"),
	Pelvis			UMETA(DisplayName = "Pelvis"),
	L5				UMETA(DisplayName = "L5"),
	L3				UMETA(DisplayName = "L3"),
	T12				UMETA(DisplayName = "T12"),
	T8				UMETA(DisplayName = "T8"),
	Neck			UMETA(DisplayName = "Neck"),
	Head			UMETA(DisplayName = "Head"),
	RightShoulder	UMETA(DisplayName = "RightShoulder"),
	RightUpperArm	UMETA(DisplayName = "RightUpperArm"),
	RightForeArm	UMETA(DisplayName = "RightForeArm"),
	RightHand		UMETA(DisplayName = "RightHand"),
	LeftShoulder    UMETA(DisplayName = "LeftShoulder"),
	LeftUpperArm    UMETA(DisplayName = "LeftUpperArm"),
	LeftForeArm		UMETA(DisplayName = "LeftForeArm"),
	LeftHand		UMETA(DisplayName = "LeftHand"),
	RightUpperLeg   UMETA(DisplayName = "RightUpperLeg"),
	RightLowerLeg   UMETA(DisplayName = "RightLowerLeg"),
	RightFoot		UMETA(DisplayName = "RightFoot"),
	RightToe		UMETA(DisplayName = "RightToe"),
	LeftUpperLeg    UMETA(DisplayName = "LeftUpperLeg"),
	LeftLowerLeg    UMETA(DisplayName = "LeftLowerLeg"),
	LeftFoot		UMETA(DisplayName = "LeftFoot"),
	LeftToe			UMETA(DisplayName = "LeftToe"),
	Prop1			UMETA(DisplayName = "Prop1"),
	Prop2			UMETA(DisplayName = "Prop2"),
	Prop3			UMETA(DisplayName = "Prop3"),
	Prop4			UMETA(DisplayName = "Prop4"),
	LeftCarpus		UMETA(DisplayName = "LeftCarpus"),
	LeftFirstMC		UMETA(DisplayName = "LeftFirstMC"),
	LeftFirstPP		UMETA(DisplayName = "LeftFirstPP"),
	LeftFirstDP		UMETA(DisplayName = "LeftFirstDP"),
	LeftSecondMC    UMETA(DisplayName = "LeftSecondMC"),
	LeftSecondPP    UMETA(DisplayName = "LeftSecondPP"),
	LeftSecondMP    UMETA(DisplayName = "LeftSecondMP"),
	LeftSecondDP    UMETA(DisplayName = "LeftSecondDP"),
	LeftThirdMC		UMETA(DisplayName = "LeftThirdMC"),
	LeftThirdPP		UMETA(DisplayName = "LeftThirdPP"),
	LeftThirdMP		UMETA(DisplayName = "LeftThirdMP"),
	LeftThirdDP		UMETA(DisplayName = "LeftThirdDP"),
	LeftFourthMC    UMETA(DisplayName = "LeftFourthMC"),
	LeftFourthPP    UMETA(DisplayName = "LeftFourthPP"),
	LeftFourthMP    UMETA(DisplayName = "LeftFourthMP"),
	LeftFourthDP    UMETA(DisplayName = "LeftFourthDP"),
	LeftFifthMC		UMETA(DisplayName = "LeftFifthMC"),
	LeftFifthPP		UMETA(DisplayName = "LeftFifthPP"),
	LeftFifthMP		UMETA(DisplayName = "LeftFifthMP"),
	LeftFifthDP		UMETA(DisplayName = "LeftFifthDP"),
	RightCarpus		UMETA(DisplayName = "RightCarpus"),
	RightFirstMC    UMETA(DisplayName = "RightFirstMC"),
	RightFirstPP    UMETA(DisplayName = "RightFirstPP"),
	RightFirstDP    UMETA(DisplayName = "RightFirstDP"),
	RightSecondMC   UMETA(DisplayName = "RightSecondMC"),
	RightSecondPP   UMETA(DisplayName = "RightSecondPP"),
	RightSecondMP   UMETA(DisplayName = "RightSecondMP"),
	RightSecondDP   UMETA(DisplayName = "RightSecondDP"),
	RightThirdMC    UMETA(DisplayName = "RightThirdMC"),
	RightThirdPP    UMETA(DisplayName = "RightThirdPP"),
	RightThirdMP    UMETA(DisplayName = "RightThirdMP"),
	RightThirdDP    UMETA(DisplayName = "RightThirdDP"),
	RightFourthMC   UMETA(DisplayName = "RightFourthMC"),
	RightFourthPP   UMETA(DisplayName = "RightFourthPP"),
	RightFourthMP   UMETA(DisplayName = "RightFourthMP"),
	RightFourthDP   UMETA(DisplayName = "RightFourthDP"),
	RightFifthMC    UMETA(DisplayName = "RightFifthMC"),
	RightFifthPP    UMETA(DisplayName = "RightFifthPP"),
	RightFifthMP    UMETA(DisplayName = "RightFifthMP"),
	RightFifthDP	UMETA(DisplayName = "RightFifthDP"),

	Count
};



UENUM()
enum class EXsensRetargetNamingConvention : uint8 {
	// Default bone map used in Unreal engine and Metahuman
	Default = 0		UMETA( DisplayName = "Unreal/Metahuman" ),
	// Use custom Blueprint callback to compute bone names
	Custom   		UMETA( DisplayName = "Custom" ),
	// Use Maya bone name convention
	Maya			UMETA( DisplayName = "Maya" ),
	// Use Xsens bone name convention
	XSens			UMETA( DisplayName = "Xsens" ),
	// Manually set bone naming map in "Remapping Assets: table
	Manual			UMETA( DisplayName = "Manual" )
};