#pragma once
#include "SegmentIndices.generated.h"

UENUM()
enum SegmentIndexes : uint8
{
	Pelvis = 0,
	L5 = 1,
	L3 = 2,
	T12 = 3,
	T8 = 4,
	Neck = 5,
	Head = 6,
	RightShoulder = 7,
	RightUpperArm = 8,
	RightForearm = 9,
	RightHand = 10,
	LeftShoulder = 11,
	LeftUpperArm = 12,
	LeftForearm = 13,
	LeftHand = 14,
	RightUpperLeg = 15,
	RightLowerLeg = 16,
	RightFoot = 17,
	RightToe = 18,
	LeftUpperLeg = 19,
	LeftLowerLeg = 20,
	LeftFoot = 21,
	LeftToe = 22,
	Prop1 = 23,
	Prop2 = 24,
	Prop3 = 25,
	Prop4 = 26,

	XS_SEG_NUM = 27, //number of segments
	XS_SEG_NUM_FINGERS = 67//number of segments with fingers enabled
};