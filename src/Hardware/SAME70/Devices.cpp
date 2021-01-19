/*
 * Devices.cpp
 *
 *  Created on: 11 Aug 2020
 *      Author: David
 */

#include "Devices.h"

AsyncSerial Serial(UART2, UART2_IRQn, ID_UART2, 512, 512, 		[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
USARTClass Serial1(USART2, USART2_IRQn, ID_USART2, 512, 512,	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
SerialCDC SerialUSB;

constexpr Pin APIN_Serial0_RXD = PortDPin(25);
constexpr Pin APIN_Serial0_TXD = PortDPin(26);
constexpr auto Serial0PinFunction = GpioPinFunction::C;
constexpr Pin APIN_Serial1_RXD = PortDPin(18);
constexpr Pin APIN_Serial1_TXD = PortDPin(19);
constexpr auto Serial1PinFunction = GpioPinFunction::C;

constexpr Pin HcmciMclkPin = PortAPin(25);
constexpr auto HsmciMclkPinFunction = GpioPinFunction::D;
constexpr Pin HsmciOtherPins[] = { PortAPin(26), PortAPin(27), PortAPin(28), PortAPin(30), PortAPin(31) };
constexpr auto HsmciOtherkPinsFunction = GpioPinFunction::C;

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
