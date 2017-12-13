/*
 * ZProbeProgrammer.h
 *
 *  Created on: 22 Jul 2017
 *      Author: David
 */

#ifndef SRC_ZPROBEPROGRAMMER_H_
#define SRC_ZPROBEPROGRAMMER_H_

#include "SoftTimer.h"

class ZProbeProgrammer
{
public:
	ZProbeProgrammer();

	void SendProgram(const uint32_t zProbeProgram[], size_t len);

private:
	static bool TimerInterrupt(void *param, uint32_t& when);
	bool Interrupt(uint32_t& when);

	SoftTimer timer;
	uint8_t progBytes[MaxZProbeProgramBytes];
	size_t numBytes;
	size_t bytesSent;
	unsigned int bitsSent;
	SoftTimer::Ticks bitTime;
	SoftTimer::Ticks startTime;

	static const unsigned int bitsPerSecond = 1000;
};

#endif /* SRC_ZPROBEPROGRAMMER_H_ */
