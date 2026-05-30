// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#ifndef SCALEDATAGRAM_H
#define SCALEDATAGRAM_H

#include "Datagram.h"

class ScaleDatagram : public Datagram 
{
public:
	ScaleDatagram();
	virtual ~ScaleDatagram();
	virtual void printData() const override;

protected:
	virtual void deserializeData(Streamer &inputStreamer) override;
	
public:
	struct Scaling 
	{
		int segmentNameLength;
		std::string segmentName;
		float segmentOriginPos[3];
	};
	int m_segmentCount;
	std::vector<Scaling> m_data;
};

#endif
