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
#include <matrix/matrix.h>

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
	EnablePullup(APIN_Serial0_RXD);

	SetPinFunction(APIN_Serial1_RXD, Serial1PinFunction);
	SetPinFunction(APIN_Serial1_TXD, Serial1PinFunction);
	EnablePullup(APIN_Serial1_RXD);
}

void SdhcInit() noexcept
{
	SetPinFunction(HsmciMclkPin, HsmciMclkPinFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciOtherPinsFunction);
		EnablePullup(p);
	}
}

void EthernetInit() noexcept
{
	// Initialize Ethernet pins
	pinMode(EthernetPhyInterruptPin, INPUT_PULLUP);
	for (Pin p : EthernetPhyOtherPins)
	{
		SetPinFunction(p, EthernetPhyOtherPinsFunction);
	}
}

// Device initialisation
void DeviceInit() noexcept
{
	LegacyAnalogIn::AnalogInInit();
	AnalogOut::Init();

	SerialInit();
	SdhcInit();
	EthernetInit();

	// Set up PB4..PB5 as normal I/O, not JTAG
	matrix_set_system_io(CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5);
}

void StopAnalogTask() noexcept
{
}

// End
