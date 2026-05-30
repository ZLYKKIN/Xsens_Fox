// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#ifndef SEGMENTINFORMATION_H
#define SEGMENTINFORMATION_H

#include "Containers/Array.h"
#include "UObject/NameTypes.h"

namespace SegmentInformation
{
	const TArray<int32> parentIndex = { -1, 0, 1, 2, 3, 4, 5, 6, 5, 8, 9, 10, 5, 12, 13, 14, 1, 16, 17, 18, 1, 20, 21, 22, 0, 0, 0, 0,
															14, 28, 29, 30, 28, 32, 33, 34, 28, 36, 37, 38, 28, 40, 41, 42, 28, 44, 45, 46,		//left hand
															10, 48, 49, 50, 48, 52, 53, 54, 48, 56, 57, 58, 48, 60, 61, 62, 48, 64, 65, 66 };	//right hand

	const TArray<FName> SegmentBoneNames = {
											FName("Root"),
											FName("Pelvis"),
											FName("L5"),
											FName("L3"),
											FName("T12"),
											FName("T8"),
											FName("Neck"),
											FName("Head"),
											FName("RightShoulder"),
											FName("RightUpperArm"),
											FName("RightForearm"),
											FName("RightHand"),
											FName("LeftShoulder"),
											FName("LeftUpperArm"),
											FName("LeftForearm"),
											FName("LeftHand"),
											FName("RightUpperLeg"),
											FName("RightLowerLeg"),
											FName("RightFoot"),
											FName("RightToe"),
											FName("LeftUpperLeg"),
											FName("LeftLowerLeg"),
											FName("LeftFoot"),
											FName("LeftToe"),
											FName("Prop1"),
											FName("Prop2"),
											FName("Prop3"),
											FName("Prop4"),
											FName("LeftCarpus"),
											FName("LeftFirstMC"),
											FName("LeftFirstPP"),
											FName("LeftFirstDP"),
											FName("LeftSecondMC"),
											FName("LeftSecondPP"),
											FName("LeftSecondMP"),
											FName("LeftSecondDP"),
											FName("LeftThirdMC"),
											FName("LeftThirdPP"),
											FName("LeftThirdMP"),
											FName("LeftThirdDP"),
											FName("LeftFourthMC"),
											FName("LeftFourthPP"),
											FName("LeftFourthMP"),
											FName("LeftFourthDP"),
											FName("LeftFifthMC"),
											FName("LeftFifthPP"),
											FName("LeftFifthMP"),
											FName("LeftFifthDP"),
											FName("RightCarpus"),
											FName("RightFirstMC"),
											FName("RightFirstPP"),
											FName("RightFirstDP"),
											FName("RightSecondMC"),
											FName("RightSecondPP"),
											FName("RightSecondMP"),
											FName("RightSecondDP"),
											FName("RightThirdMC"),
											FName("RightThirdPP"),
											FName("RightThirdMP"),
											FName("RightThirdDP"),
											FName("RightFourthMC"),
											FName("RightFourthPP"),
											FName("RightFourthMP"),
											FName("RightFourthDP"),
											FName("RightFifthMC"),
											FName("RightFifthPP"),
											FName("RightFifthMP"),
											FName("RightFifthDP"),
	};
};

#endif
