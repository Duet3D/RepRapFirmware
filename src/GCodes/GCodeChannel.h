/*
 * GCodeChannel.h
 *
 *  Created on: 04 Jul 2019
 *      Author: Christian
 */

#include <cstdint>

#ifndef SRC_GCODES_GCODEBUFFER_CODECHANNEL_H_
#define SRC_GCODES_GCODEBUFFER_CODECHANNEL_H_

enum class GCodeChannel : uint8_t
{
	http = 0,
	telnet = 1,
	file = 2,
	usb = 3,
	aux = 4,
	daemon = 5,
	queue = 6,
	lcd = 7,
	spi = 8,
	autopause = 9
};

constexpr size_t NumGCodeChannels = 10;

extern const char * const gcodeChannelName[];

#endif
