// Copyright 2018-2025 Movella Technologies B.V., Inc. All Rights Reserved.

#include "MetaDatagram.h"

 /*! Constructor */
MetaDatagram::MetaDatagram()
	: Datagram()
{
	setType(SPMetaMoreMeta);
}

/*! Destructor */
MetaDatagram::~MetaDatagram()
{

}

/*! Deserialize the data from \a arr
	\sa serializeData
*/
void MetaDatagram::deserializeData(Streamer& inputStreamer)
{
	Streamer* streamer = &inputStreamer;

	int textLength;
	streamer->read(textLength);

	std::string text;
	streamer->read(text, textLength);

	// since we're interested only in the name, no need to parse the whole string
	static const std::string nameTag = "name:";
	size_t foundPos = text.find(nameTag);
	if (foundPos != std::string::npos)
	{
		size_t endPos = text.find("\n", foundPos);
		m_Name = text.substr(foundPos + nameTag.length(), endPos - foundPos - nameTag.length());
	}
}

/*! Print Data datagram in a formated why
*/
void MetaDatagram::printData() const
{
	std::cout << "*********************** DATA CONTENT ***********************" << std::endl << std::endl;
}
