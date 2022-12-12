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

#if CORE_USES_TINYUSB
# include <TinyUsbInterface.h>
# include <Platform/TaskPriorities.h>
#endif

AsyncSerial serialUart1(UART2, UART2_IRQn, ID_UART2, 512, 512,
					[](AsyncSerial*) noexcept
					{
						SetPinFunction(APIN_Serial0_RXD, Serial0PinFunction);
						SetPinFunction(APIN_Serial0_TXD, Serial0PinFunction);
					},
					[](AsyncSerial*) noexcept
					{
						ClearPinFunction(APIN_Serial0_RXD);
						ClearPinFunction(APIN_Serial0_TXD);
					}
				);

USARTClass serialUart2(USART2, USART2_IRQn, ID_USART2, 512, 512,
					[](AsyncSerial*) noexcept
					{
						SetPinFunction(APIN_Serial1_RXD, Serial1PinFunction);
						SetPinFunction(APIN_Serial1_TXD, Serial1PinFunction);
					},
					[](AsyncSerial*) noexcept
					{
						ClearPinFunction(APIN_Serial1_RXD);
						ClearPinFunction(APIN_Serial1_TXD);
					}
				);

#if defined(DUET3_MB6HC)
AsyncSerial serialWiFi(UART4, UART4_IRQn, ID_UART4, 512, 512,
					[](AsyncSerial*) noexcept
					{
						SetPinFunction(APIN_SerialWiFi_RXD, SerialWiFiPeriphMode);
						SetPinFunction(APIN_SerialWiFi_TXD, SerialWiFiPeriphMode);
					},
					[](AsyncSerial*) noexcept
					{
						ClearPinFunction(APIN_SerialWiFi_RXD);
						ClearPinFunction(APIN_SerialWiFi_TXD);
					}
				);
#endif

void UART2_Handler(void) noexcept
{
	serialUart1.IrqHandler();
}

void USART2_Handler(void) noexcept
{
	serialUart2.IrqHandler();
}

#if defined(DUET3_MB6HC)
void UART4_Handler(void) noexcept
{
	serialWiFi.IrqHandler();
}
#endif

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
	for (Pin p : EthernetPhyOtherPins)
	{
		SetPinFunction(p, EthernetPhyOtherPinsFunction);
	}
}

#if CORE_USES_TINYUSB

constexpr size_t UsbDeviceTaskStackWords = 200;
static Task<UsbDeviceTaskStackWords> usbDeviceTask;

#endif

SerialCDC serialUSB;

// Device initialisation
void DeviceInit() noexcept
{
	LegacyAnalogIn::AnalogInInit();
	AnalogOut::Init();

	SdhcInit();
	EthernetInit();

#if defined(DUET3_MB6HC) || defined(DUET3_MB6XD)
# ifdef DEBUG
	// Set up PB4..PB5 as normal I/O, not JTAG. Leave PB6/7 pins as SWD. STATUS and ACT LEDs will not work.
	matrix_set_system_io(CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5);
# else
	// Set up PB4..PB7 as normal I/O, not JTAG or SWD
	matrix_set_system_io(CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5 | CCFG_SYSIO_SYSIO6 | CCFG_SYSIO_SYSIO7);
# endif
#endif

#if CORE_USES_TINYUSB
	CoreUsbInit(NvicPriorityUSB);
	usbDeviceTask.Create(CoreUsbDeviceTask, "USBD", nullptr, TaskPriority::UsbPriority);
#endif
}

void StopAnalogTask() noexcept
{
}

void StopUsbTask() noexcept
{
#if CORE_USES_TINYUSB
	usbDeviceTask.TerminateAndUnlink();
#endif
}

// End
