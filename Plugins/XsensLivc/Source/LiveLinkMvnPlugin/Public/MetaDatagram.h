// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#pragma once

#include "Datagram.h"

class MetaDatagram : public Datagram
{
public:
	MetaDatagram();
	virtual ~MetaDatagram();
	virtual void printData() const override;

protected:
	virtual void deserializeData(Streamer& inputStreamer) override;

public:

	std::string m_Name;
};
