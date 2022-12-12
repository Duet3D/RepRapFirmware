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

AsyncSerial serialUart(UART0, UART0_IRQn, ID_UART0, 512, 512, 	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
AsyncSerial serialWiFi(UART1, UART1_IRQn, ID_UART1, 512, 512,	[](AsyncSerial*) noexcept { }, [](AsyncSerial*) noexcept { });
SerialCDC serialUSB;

void UART0_Handler(void) noexcept
{
	serialUart.IrqHandler();
}

void UART1_Handler(void) noexcept
{
	serialWiFi.IrqHandler();
}

void SerialInit() noexcept
{
	SetPinFunction(APIN_Serial0_RXD, Serial0PeriphMode);
	SetPinFunction(APIN_Serial0_TXD, Serial0PeriphMode);
	EnablePullup(APIN_Serial0_RXD);

	SetPinFunction(APIN_SerialWiFi_RXD, SerialWiFiPeriphMode);
	SetPinFunction(APIN_SerialWiFi_TXD, SerialWiFiPeriphMode);
	EnablePullup(APIN_SerialWiFi_RXD);
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

void StopUsbTask() noexcept
{
}

// End
