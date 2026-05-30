// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "ScaleDatagram.h"

#ifndef M_PI
	#define M_PI 3.14159265359f
#endif
#define rad2deg(rad) ((rad) * 180 / M_PI)

/*! \class QuaternionDatagram
  \brief a Position & Quaternion orientation pose datagram (type 02)

  Information about each segment is sent as follows.

  4 bytes segment ID, in the range 1-30
  4 bytes xñcoordinate of sensor position
  4 bytes yñcoordinate of sensor position
  4 bytes zñcoordinate of sensor position
  4 bytes q1 rotation ñ sensor rotation quaternion component 1 (re)
  4 bytes q2 rotation ñ sensor rotation quaternion component 1 (i)
  4 bytes q3 rotation ñ sensor rotation quaternion component 1 (j)
  4 bytes q4 rotation ñ sensor rotation quaternion component 1 (k)

  Total: 32 bytes per segment

  The coordinates use a Z-Up, right-handed coordinate system.
 */

/*! Constructor */
ScaleDatagram::ScaleDatagram()
	: Datagram()
{
	setType(SPMetaScaling);
}

/*! Destructor */
ScaleDatagram::~ScaleDatagram()
{

}

/*! Deserialize the data from \a arr
	\sa serializeData
*/
void ScaleDatagram::deserializeData(Streamer &inputStreamer)
{
	Streamer* streamer = &inputStreamer;

	streamer->read(m_segmentCount);
	for (int i = 0; i < m_segmentCount; i++)
	{
		Scaling scale;

		// 4 byte
		streamer->read(scale.segmentNameLength);
		streamer->read(scale.segmentName, scale.segmentNameLength);

		// Store the Sensor Position in a Vector -> 12 byte	(3 x 4 byte)
		// The coordinates use a Z-Up, right-handed coordinate system.
		for (int k = 0; k < 3; k++)
			streamer->read(scale.segmentOriginPos[k]);

		// Store the Quaternion Rotation in a vector -> 16 byte	(4 x 4 byte) 
	/*	for (int k = 0; k < 4; k++)
			streamer->read(kin.quatRotation[k]);*/

		m_data.push_back(scale);
	}
	if (!m_data.empty())
	{
		FString spook = m_data.at(0).segmentName.c_str();
		//read points if needed
		//UE_LOG(LogTemp, Warning, TEXT("SEG[%d]: %s"), m_data.at(0).segmentNameLength, *spook);// m_data.at(0).segmentName);// , m_data.at(0).segmentOriginPos[0].toString()); // Pos:%s   Rot:%s , m_data.at(i).sensorPos.ToString(), m_data.at(i).quatRotation.ToString()
	}
}

/*! Print Data datagram in a formated why
*/
void ScaleDatagram::printData() const
{
	std::cout << "*********************** DATA CONTENT ***********************" <<  std::endl <<  std::endl;

	//for (int i = 0; i < m_data.size(); i++)
	//{
	//	std::cout << "Segment ID: " << m_data.at(i).segmentId << std::endl;
	//	// Sensor Position
	//	std::cout << "Segment Position: " << "(";
	//	std::cout << "x: " << m_data.at(i).sensorPos[0] << ", ";
	//	std::cout << "y: " << m_data.at(i).sensorPos[1] << ", ";
	//	std::cout << "z: " << m_data.at(i).sensorPos[2] << ")"<< std::endl;

	//	// Quaternion Rotation
	//	std::cout << "Quaternion Rotation: " << "(";
	//	std::cout << "re: " << m_data.at(i).quatRotation[0] << ", ";
	//	std::cout << "i: " << m_data.at(i).quatRotation[1] << ", ";
	//	std::cout << "j: " << m_data.at(i).quatRotation[2] << ", ";
	//	std::cout << "k: " << m_data.at(i).quatRotation[3] << ")"<< std::endl << std::endl;
	//}
}
