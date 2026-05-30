// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "ParserManager.h"

#include "QuaternionDatagram.h"
#include "ScaleDatagram.h"
#include "MetaDatagram.h"

ParserManager::ParserManager()
{ 
}

/*! Destructor */
ParserManager::~ParserManager()
{
}

Datagram* ParserManager::createDgram(StreamingProtocol proto)
{
	switch (proto)
	{
		case SPPoseQuaternion:
			return (Datagram *)new QuaternionDatagram;
//		case SPPoseUnity3D:
//			return (Datagram *)new Unity3DDatagram;
//		case SPPoseEuler:
//			return (Datagram *)new EulerDatagram;
//		case SPPosePositions:
//			return (Datagram *)new PositionDatagram;
		case SPMetaScaling:
			return (Datagram *)new ScaleDatagram;
		case SPMetaMoreMeta:
			return (Datagram *)new MetaDatagram;
//		case SPJointAngles:
//			return (Datagram *)new JointAnglesDatagram;
//		case SPLinearSegmentKinematics:
//			return (Datagram *)new LinearSegmentKinematicsDatagram;
//		case SPAngularSegmentKinematics:
//			return (Datagram *)new AngularSegmentKinematicsDatagram;
//		case SPTrackerKinematics:
//			return (Datagram *)new TrackerKinematicsDatagram;
//		case SPCenterOfMass:
//			return (Datagram *)new CenterOfMassDatagram;
//		case SPTimeCode:
//			return (Datagram *)new TimeCodeDatagram;
		
		default:
//			UE_LOG(LogTemp, Error, TEXT("Unknown datagram: %d"), (int)proto);
			return nullptr;
	}	
}

/*! Read single datagram from the incoming stream */
Datagram* ParserManager::readDatagram(const TArray<uint8_t>& data)
{
	StreamingProtocol type = static_cast<StreamingProtocol>(Datagram::messageType(data));
	Datagram *datagram = createDgram(type);

	if (datagram != nullptr) 
	{
		datagram->deserialize(data);
		//log to console
//		datagram->printHeader();
//		datagram->printData();
		return datagram;
	}
	return nullptr;
}
