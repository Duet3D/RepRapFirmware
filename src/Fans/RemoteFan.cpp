/*
 * RemoteFan.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "RemoteFan.h"

#if SUPPORT_CAN_EXPANSION

RemoteFan::RemoteFan(unsigned int fanNum)
	: Fan(fanNum),
	  lastRpm(-1), whenLastRpmReceived(0)
{
}

RemoteFan::~RemoteFan()
{
	//TODO release remote fan
}

bool RemoteFan::Check()
{
	//TODO
	return false;
}

bool RemoteFan::IsEnabled() const
{
	return true;
}

void RemoteFan::SetPwmFrequency(PwmFrequency freq)
{
	//TODO
}

int32_t RemoteFan::GetRPM()
{
	if (millis() - whenLastRpmReceived > RpmReadingTimeout)
	{
		lastRpm = -1;
	}
	return lastRpm;
}

void RemoteFan::AppendPortDetails(const StringRef& str) const
{
	//TODO
	str.cat(" port details not available");
}

void RemoteFan::UpdateFanConfiguration()
{
	//TODO
}

void RemoteFan::Refresh()
{
	//TODO
}

#endif

// End
