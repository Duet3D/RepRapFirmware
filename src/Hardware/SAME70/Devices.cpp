/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"
#include <RepRapFirmware.h>
#include <AnalogIn.h>
#include <AnalogOut.h>

AsyncSerial Serial(UART2, UART2_IRQn, ID_UART2, 512, 512, 		[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
USARTClass Serial1(USART2, USART2_IRQn, ID_USART2, 512, 512,	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
SerialCDC SerialUSB;

void UART2_Handler(void) noexcept
{
	Serial.IrqHandler();
}

void USART2_Handler(void) noexcept
{
	Serial1.IrqHandler();
}

void SerialInit() noexcept
{
	SetPinFunction(APIN_Serial0_RXD, Serial0PinFunction);
	SetPinFunction(APIN_Serial0_TXD, Serial0PinFunction);
	SetPullup(APIN_Serial0_RXD, true);

	SetPinFunction(APIN_Serial1_RXD, Serial1PinFunction);
	SetPinFunction(APIN_Serial1_TXD, Serial1PinFunction);
	SetPullup(APIN_Serial1_RXD, true);
}

void SdhcInit() noexcept
{
	SetPinFunction(HsmciMclkPin, HsmciMclkPinFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciOtherPinsFunction);
	}
}

// Device initialisation
void DeviceInit() noexcept
{
	SerialInit();
	SdhcInit();

	LegacyAnalogIn::AnalogInInit();
	AnalogOut::Init();
}

void StopAnalogTask() noexcept
{
}

// End
