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

NamedEnum(GCodeChannel, uint8_t, HTTP, Telnet, File, USB, Aux, Trigger, Queue, LCD, SBC, Daemon, Aux2, Autopause);

constexpr size_t NumGCodeChannels = GCodeChannel::NumValues;

#endif
