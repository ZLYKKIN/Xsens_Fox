// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#ifndef PARSERMANAGER_H
#define PARSERMANAGER_H

#include "Datagram.h"

class LIVELINKMVNPLUGIN_API ParserManager
{
public:
//	const bool UseGlobalSource = true;
	
	ParserManager();
	~ParserManager();
	Datagram* readDatagram(const TArray<uint8_t>& data);

private:
	Datagram* createDgram(StreamingProtocol proto);
};

#endif
