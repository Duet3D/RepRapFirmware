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
#include <pmc/pmc.h>

AsyncSerial Serial (UART0, UART0_IRQn, ID_UART0, 512, 512, 	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
AsyncSerial Serial1(UART1, UART1_IRQn, ID_UART1, 512, 512,	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
SerialCDC SerialUSB;

void UART0_Handler(void) noexcept
{
	Serial.IrqHandler();
}

void UART1_Handler(void) noexcept
{
	Serial1.IrqHandler();
}

void SerialInit() noexcept
{
	SetPinFunction(APIN_Serial0_RXD, Serial0PeriphMode);
	SetPinFunction(APIN_Serial0_TXD, Serial0PeriphMode);
	EnablePullup(APIN_Serial0_RXD);

	SetPinFunction(APIN_Serial1_RXD, Serial1PeriphMode);
	SetPinFunction(APIN_Serial1_TXD, Serial1PeriphMode);
	EnablePullup(APIN_Serial1_RXD);
}

void SdhcInit() noexcept
{
	SetPinFunction(HsmciClockPin, HsmciPinsFunction);
	for (Pin p : HsmciOtherPins)
	{
		SetPinFunction(p, HsmciPinsFunction);
		EnablePullup(p);
	}
}

void WireInit() noexcept
{
	pmc_enable_periph_clk(WIRE_INTERFACE_ID);
	SetPinFunction(TWI_Data, TWIPeriphMode);
	SetPinFunction(TWI_CK, TWIPeriphMode);

	NVIC_DisableIRQ(WIRE_ISR_ID);
	NVIC_ClearPendingIRQ(WIRE_ISR_ID);
}

TwoWire Wire(WIRE_INTERFACE, WireInit);


// Device initialisation
void DeviceInit() noexcept
{
	LegacyAnalogIn::AnalogInInit();
	AnalogOut::Init();

	SerialInit();
	SdhcInit();
}

void StopAnalogTask() noexcept
{
}

// End
