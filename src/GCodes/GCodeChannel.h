/*
 * GCodeChannel.h
 *
 *  Created on: 04 Jul 2019
 *      Author: Christian
 */

#ifndef SRC_GCODES_GCODEBUFFER_CODECHANNEL_H_
#define SRC_GCODES_GCODEBUFFER_CODECHANNEL_H_

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>

// The Microchip device library for SAME5x defines USB as the USB peripheral.
// We can't change the channel name to something else because it breaks compatibility with DSF, so #undef it here
#ifdef USB
# undef USB
#endif

NamedEnum(GCodeChannel, uint8_t, HTTP, Telnet, File, USB, Aux, Trigger, Queue, LCD, SBC, Daemon, Aux2, Autopause);

constexpr size_t NumGCodeChannels = GCodeChannel::NumValues;

#endif
