// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#ifndef QUATERNIONDATAGRAM_H
#define QUATERNIONDATAGRAM_H

#include "Datagram.h"

class QuaternionDatagram : public Datagram 
{
public:
	QuaternionDatagram();
	virtual ~QuaternionDatagram();
	virtual void printData() const override;

protected:
	virtual void deserializeData(Streamer &inputStreamer) override;
	
public:
	struct Kinematics 
	{
		int segmentId;
		float sensorPos[3];
		float quatRotation[4];
	};
	std::vector<Kinematics> m_data;
};

#endif
