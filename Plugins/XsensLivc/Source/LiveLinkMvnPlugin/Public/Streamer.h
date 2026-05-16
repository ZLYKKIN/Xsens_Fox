// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#ifndef STREAMER_H
#define STREAMER_H

#include "CoreMinimal.h"

#include <memory>
#include <iostream>
#include <sstream>

class Streamer
{
public:
	Streamer(const TArray<uint8_t>& arr);
	~Streamer();

	void read(int32_t &destination);
	void read(int8_t &destination);
	void read(int16_t &destination);
	void read(uint8_t &destination);
	void read(float &destination);
	void read(std::string& str, int numChars);

private:

	enum Endianess  {
		Little_Endian,
		Big_Endian
	};

	const TArray<uint8_t>* m_array;
	int m_offset;
	Endianess m_endianess;
};

#endif
