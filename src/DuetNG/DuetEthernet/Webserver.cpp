/*
 * Webserver.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "RepRapFirmware.h"

Webserver::Webserver(Platform* p, Network *n)
	: seq(0)
{

}

void Webserver::Init()
{

}

void Webserver::Spin()
{

}

void Webserver::Exit()
{

}

void Webserver::Diagnostics(MessageType mtype)
{

}

bool Webserver::GCodeAvailable(const WebSource source) const
{
	return false;
}

char Webserver::ReadGCode(const WebSource source)
{
	return 0;
}

void Webserver::HandleGCodeReply(const WebSource source, OutputBuffer *reply)
{

}

void Webserver::HandleGCodeReply(const WebSource source, const char *reply)
{

}

// End
