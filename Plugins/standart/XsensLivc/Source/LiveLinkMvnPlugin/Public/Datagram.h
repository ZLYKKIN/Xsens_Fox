// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#ifndef DATAGRAM_H
#define DATAGRAM_H

#include <memory>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>

#include "Streamer.h"

enum StreamingProtocol {
	SPPoseEuler = 0x01,
	SPPoseQuaternion = 0x02,
	SPPosePositions = 0x03,
	SPJackProcessSimulate = 0x04,
	SPPoseUnity3D = 0x05,

	// meta data
	//SPMetaScaling = 0x10,		// obsolete protocol
	//SPMetaPropInfo = 0x11,	// obsolete protocol
	SPMetaMoreMeta = 0x12,
	SPMetaScaling = 0x13,

	// additional information
	SPJointAngles = 0x20,
	SPLinearSegmentKinematics = 0x21,
	SPAngularSegmentKinematics = 0x22,
	SPTrackerKinematics = 0x23,
	SPCenterOfMass = 0x24,
	SPTimeCode = 0x25,
};

class LIVELINKMVNPLUGIN_API Datagram
{
public:
	Datagram();
	virtual ~Datagram();

	bool deserialize(const TArray<uint8_t>& arr);
	void setDataCount(uint8_t c);
	void setType(StreamingProtocol proto);
	int32_t messageType() const;
	int32_t sampleCounter() const;
	int32_t frameTime() const;
	uint8_t avatarId() const;
	uint8_t dataCount() const;
	uint8_t datagramCounter() const;
	uint8_t bodySegmentCount() const;
	uint8_t propCount() const;
	uint8_t fingerTrackingSegmentCount() const;

	static int messageType(const TArray<uint8_t>& arr);
	std::string decode(StreamingProtocol proto) const;

	void convertFromYupToZup(float *vector) const;

	void printHeader() const;
	virtual void printData() const = 0;
	
protected:
	virtual void deserializeData(Streamer &inputStreamer) = 0;
	static const float EULERPOSITIONSCALE;

private:
	std::string m_header;
	int32_t m_sampleCounter;
	int32_t m_frameTime;
	uint8_t m_avatarId;
	uint8_t m_dataCount;
	uint8_t m_dgramCounter;
	uint8_t m_bodySegmentCount;
	uint8_t m_propCount;
	uint8_t m_fingerTrackingSegmentCount;
	int m_dataSize;

	int getDataSize() const;
	void initMap(std::map<int, std::string> &map);
	std::map<int, std::string> m_packetsName;
};

#endif
