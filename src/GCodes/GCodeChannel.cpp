/*
 * GCodeChannel.cpp
 *
 *  Created on: 4 Jul 2019
 *      Author: Christian
 */

#include "GCodeChannel.h"

extern const char * const gcodeChannelName[] =
{
	"http",
	"telnet",
	"file",
	"serial",
	"aux",
	"trigger",
	"queue",
	"lcd",
	"spi",
	"daemon",
	"autopause"
};

static_assert(ARRAY_SIZE(gcodeChannelName) == NumGCodeChannels);

// End
