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
#include <matrix/matrix.h>

#ifndef PCCB
AsyncSerial Serial (UART1, UART1_IRQn, ID_UART1, 512, 512, 	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });

void UART1_Handler(void) noexcept
{
	Serial.IrqHandler();
}

void SerialInit() noexcept
{
	SetPinFunction(APIN_Serial0_RXD, Serial0PeriphMode);
	SetPinFunction(APIN_Serial0_TXD, Serial0PeriphMode);
	EnablePullup(APIN_Serial0_RXD);
}
#endif

SerialCDC SerialUSB;

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

#ifndef PCCB
	SerialInit();
#endif
	SdhcInit();

#ifndef PCCB
	// Set up PB4..PB7 as normal I/O, not JTAG
	matrix_set_system_io(CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5 | CCFG_SYSIO_SYSIO6 | CCFG_SYSIO_SYSIO7);
#endif
}

void StopAnalogTask() noexcept
{
}

// End
