/*
 * GCodeChannel.h
 *
 *  Created on: 04 Jul 2019
 *      Author: Christian
 */

#include <RepRapFirmware.h>

#ifndef SRC_GCODES_GCODEBUFFER_CODECHANNEL_H_
#define SRC_GCODES_GCODEBUFFER_CODECHANNEL_H_

// If you add to this list, remember to update the gcodeChannelName array too!
enum class GCodeChannel : uint8_t
{
	http = 0,
	telnet = 1,
	file = 2,
	usb = 3,
	aux = 4,
	trigger = 5,
	queue = 6,
	lcd = 7,
	spi = 8,
	daemon = 9,
	autopause = 10
};

constexpr size_t NumGCodeChannels = 11;

extern const char * const gcodeChannelName[];

#endif
