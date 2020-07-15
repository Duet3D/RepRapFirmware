/*
 * Lcd7567.cpp
 *
 *  Created on: 15 Jul 2020
 *      Author: David
 */

#include "Lcd7567.h"

#if SUPPORT_12864_LCD

Lcd7567::Lcd7567(const LcdFont * const fnts[], size_t nFonts) noexcept
	: Lcd(128, 64, fnts, nFonts)
{
}

// Flush just some data, returning true if this needs to be called again
bool Lcd7567::FlushSome() noexcept
{
	//TODO
	return true;
}

void Lcd7567::HardwareInit() noexcept
{
	pinMode(a0Pin, OUTPUT_HIGH);
	//TODO
}

void Lcd7567::CommandDelay() noexcept
{
	//TODO
}

void Lcd7567::DataDelay() noexcept
{
	//TODO
}

void Lcd7567::SendLcdCommand(uint8_t byteToSend) noexcept
{
	digitalWrite(a0Pin, false);
	uint8_t data[1] = { byteToSend };
	device.TransceivePacket(data, nullptr, 1);
}

void Lcd7567::SendLcdData(uint8_t byteToSend) noexcept
{
	digitalWrite(a0Pin, true);
	uint8_t data[1] = { byteToSend };
	device.TransceivePacket(data, nullptr, 1);
}

#endif

// End
