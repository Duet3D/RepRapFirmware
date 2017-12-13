/*
 * ZProbeProgrammer.cpp
 *
 *  Created on: 22 Jul 2017
 *      Author: David
 */

#include "ZProbeProgrammer.h"
#include "RepRap.h"
#include "Platform.h"

/*static*/ bool ZProbeProgrammer::TimerInterrupt(void *param, uint32_t& when)
{
	return static_cast<ZProbeProgrammer*>(param)->Interrupt(when);
}

ZProbeProgrammer::ZProbeProgrammer()
{
}

// Kick off sending some program bytes
void ZProbeProgrammer::SendProgram(const uint32_t zProbeProgram[], size_t len)
{
	timer.CancelCallback();										// make quite certain that this timer isn't already pending

	for (size_t i = 0; i < len; ++i)
	{
		progBytes[i] = (uint8_t)zProbeProgram[i];
	}
	numBytes = len;
	bytesSent = 0;
	bitsSent = 0;
	bitTime = SoftTimer::GetTickRate()/bitsPerSecond;

	reprap.GetPlatform().SetZProbeModState(false);				// start with 2 bits of zero
	startTime = SoftTimer::GetTimerTicksNow();
	timer.ScheduleCallback(startTime + 2 * bitTime, ZProbeProgrammer::TimerInterrupt, static_cast<void*>(this));
}

bool ZProbeProgrammer::Interrupt(uint32_t& when)
{
	// The data format is:
	// [0 0 1 0 b7 b6 b5 b4 /b4 b3 b2 b1 b0 /b0] repeated for each byte, where /b4 = inverse of b4, /b0 = inverse of b0
	// After the last byte the line returns to 0
	bool nextBit;
	switch(bitsSent++)
	{
	case 0:		// We sent 00, now send 1
		nextBit = true;
		break;

	case 1:	// We sent 001, now send 0
	default:
		nextBit = false;
		break;

	case 2:
	case 3:
	case 4:
	case 5:
		nextBit = (((progBytes[bytesSent] >> (10 - bitsSent)) & 1) != 0);
		break;

	case 6:
		nextBit = (((progBytes[bytesSent] >> 4) & 1) == 0);
		break;

	case 7:
	case 8:
	case 9:
	case 10:
		nextBit = (((progBytes[bytesSent] >> (11 - bitsSent)) & 1) != 0);
		break;

	case 11:
		nextBit = ((progBytes[bytesSent] & 1) == 0);
		break;

	case 12:		// We sent 0010 + 10 data bits, now send 0
		nextBit = false;
		bitsSent = 0;
		++bytesSent;
		break;
	}

	reprap.GetPlatform().SetZProbeModState(nextBit);
	if (bytesSent < numBytes)
	{
		when = startTime + ((bytesSent * 14) + bitsSent + 2) * bitTime;
		return true;
	}

	bytesSent = numBytes = 0;
	return false;
}

// End
