// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "Streamer.h"

/*!
	Intel processors use "Little Endian" byte order:

	Base_Address+0 Byte0
	Base_Address+1 Byte1
	Base_Address+2 Byte2
	Base_Address+3 Byte3

	-------------------------
	"Big Endian" :

	Base_Address+0 Byte3
	Base_Address+1 Byte2
	Base_Address+2 Byte1
	Base_Address+3 Byte0
*/

Streamer::Streamer(const TArray<uint8_t>& arr)
	: m_offset(0)
{
	// reference to the real data
	m_array = &arr;

	// check Endianess
	int num = 1;
	if(*(char *)&num == 1)
	{
		m_endianess = Little_Endian;
	}
	else
	{
		m_endianess = Big_Endian;
	}
}

/*! Destructor */
Streamer::~Streamer() 
{
}

/*! Extract 4 byte from the ByteArray and Store the value into a int32_t (4 byte) variable */
void Streamer::read(int32_t &destination)
{
	char byte_array[4];

	// copy part of the original byte array into a temp array 
	memcpy(byte_array, m_array->GetData()+m_offset, sizeof(destination));

	if (m_endianess == Little_Endian)
		destination = byte_array[0] << 24 | (byte_array[1] & 0xff) << 16 | (byte_array[2] & 0xff) << 8| (byte_array[3] & 0xff);
	else
		destination = byte_array[3] << 24 | (byte_array[2] & 0xff) << 16 | (byte_array[1] & 0xff) << 8| (byte_array[0] & 0xff);

	// increase the index
	m_offset += 4;
}

/*! Extract 1 byte from the ByteArray and Store the value into a int8_t (1 byte) variable */
void Streamer::read(int8_t &destination)
{
	int8_t res;

	memcpy(&res, m_array->GetData()+m_offset, 1);
	destination = (int)res & 0xFF;

	// increase the index
	m_offset++;
}

/*! Extract 2 byte from the ByteArray and Store the value into a int16_t (2 byte) variable */
void Streamer::read(int16_t &destination)
{
	char byte_array[2];

	// copy part of the original byte array into a temp array 
	memcpy(byte_array, m_array->GetData()+m_offset, 2);

	if (m_endianess == Little_Endian)
		destination = 0x00 << 24 | 0x00 << 16 | (byte_array[0] & 0xff) << 8 | (byte_array[1] & 0xff);
	else
		destination = 0x00 << 24 | 0x00 << 16 | (byte_array[1] & 0xff) << 8 | (byte_array[0] & 0xff);
	
	// increase the index
	m_offset += 2;
}

/*! Extract 1 byte from the ByteArray and Store the value into a uint8_t (1 byte) variable */
void Streamer::read(uint8_t &destination)
{
	uint8_t res;

	memcpy(&res, m_array->GetData()+m_offset, 1);
	destination = (int)res & 0xFF;

	// increase the index
	m_offset++;
}

/*! Extract 4 byte from the ByteArray and Store the value into a float (4 byte) variable */
void Streamer::read(float &destination)
{
	float output = 0.0;

	unsigned char byte_array[4];
	// copy part of the original byte array into a temp array 
	memcpy(&byte_array, m_array->GetData()+m_offset, sizeof(destination));

	if (m_endianess == Little_Endian)
	{
		*((unsigned char*)(&output) + 3) = byte_array[0];
		*((unsigned char*)(&output) + 2) = byte_array[1];
		*((unsigned char*)(&output) + 1) = byte_array[2];
		*((unsigned char*)(&output) + 0) = byte_array[3];
	}
	else
	{
		*((unsigned char*)(&output) + 3) = byte_array[3];
		*((unsigned char*)(&output) + 2) = byte_array[2];
		*((unsigned char*)(&output) + 1) = byte_array[1];
		*((unsigned char*)(&output) + 0) = byte_array[0];
	}

	destination = output; 

	// increase the index
	m_offset += 4;
}

/*! Extract "size" byte from the ByteArray and Store the value into a string variable */
void Streamer::read(std::string& str, int numChars)
{
	char * buffer = new char[numChars + 1];

	memcpy(buffer, m_array->GetData()+m_offset, numChars);
	buffer[numChars] = '\0';

	str = buffer;
	delete[] buffer;

	// increase the index
	m_offset += numChars;
}
