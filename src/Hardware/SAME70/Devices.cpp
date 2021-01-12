/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"

AsyncSerial serialUart0(UART2, UART2_IRQn, ID_UART2, 512, 512, [](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

constexpr Pin APIN_Serial0_RXD = PortDPin(25);
constexpr Pin APIN_Serial0_TXD = PortDPin(26);
constexpr auto Serial0PinFunction = GpioPinFunction::C;

constexpr Pin HcmciMclkPin = PortAPin(25);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::D;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherkPinsFunction = GpioPinFunction::C;

void UART2_Handler(void)
{
	serialUart0.IrqHandler();
}


void SerialInit() noexcept
{
	SetPinFunction(APIN_Serial0_RXD, Serial0PinFunction);
	SetPinFunction(APIN_Serial0_TXD, Serial0PinFunction);
	SetPullup(APIN_Serial0_RXD, true);
}

void SdhcInit() noexcept
{
	SetPinFunction(HcmciMclkPin, HsmciMclkPinFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciOtherkPinsFunction);
	}
}


// Device initialisation
void DeviceInit() noexcept
{
	SerialInit();
	SdhcInit();
}

// End
