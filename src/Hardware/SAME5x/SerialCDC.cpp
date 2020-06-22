/*
 * UsbSerial.cpp
 *
 *  Created on: 19 Jun 2020
 *      Author: David
 */

#include "SerialCDC.h"

extern "C" {
#include <AtmelStart_SAME5x/usb/class/cdc/device/cdcdf_acm.h>
#include <AtmelStart_SAME5x/usb/class/cdc/device/cdcdf_acm_desc.h>
}

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

static SerialCDC *device;
static bool sending = false, receiving = false;
static usb_cdc_control_signal_t cdcState;

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

// Buffers to receive and send data. I am not sure that they need to be aligned.
alignas(4) static uint8_t rxTempBuffer[64];
alignas(4) static uint8_t txTempBuffer[64];

/**
 * \brief Callback invoked when bulk data received
 */
static bool usb_device_cb_bulk_rx(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	device->DataReceived(count);
	return false;						// no error
}

/**
 * \brief Callback invoked when bulk data has been sent
 */
static bool usb_device_cb_bulk_tx(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	sending = false;
	device->StartSending();
	return false;						// no error
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	cdcState = state;
	if (state.rs232.DTR)
	{
		// Callbacks must be registered after endpoint allocation
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_rx);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_tx);

		// Start receive
		device->StartReceiving();
	}

	/* No error. */
	return false;
}

SerialCDC::SerialCDC(Pin p, size_t numTxSlots, size_t numRxSlots) noexcept : vbusPin(p)
{
	txBuffer.Init(numTxSlots);
	rxBuffer.Init(numRxSlots);
}

void SerialCDC::Start() noexcept
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();

	//TODO not sure whether this just waits for installation to complete or waits for a connection
	while (!cdcdf_acm_is_enabled())
	{
		// wait cdc acm to be installed
	}

	device = this;
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
}

void SerialCDC::end() noexcept
{
	cdcdf_acm_deinit();
	usbdc_deinit();
}

bool SerialCDC::IsConnected() const noexcept
{
	return cdcState.rs232.DTR;
}

// Overridden virtual functions

// Non-blocking read, return 0 if no character available
int SerialCDC::read() noexcept
{
	uint8_t c;
	if (rxBuffer.GetItem(c))
	{
		StartReceiving();
		return c;
	}
	return -1;
}

int SerialCDC::available() noexcept
{
	return rxBuffer.ItemsPresent();
}

void SerialCDC::flush() noexcept
{
	while (!txBuffer.IsEmpty()) { }
	//TODO wait until no data in the USB buffer
}

size_t SerialCDC::canWrite() const noexcept
{
	return txBuffer.SpaceLeft();
}

// Write single character, blocking
size_t SerialCDC::write(uint8_t c) noexcept
{
	for (;;)
	{
		if (txBuffer.PutItem(c))
		{
			StartSending();
			break;
		}
		txWaitingTask = RTOSIface::GetCurrentTask();
		StartSending();
		TaskBase::Take(50);
	}
	return 1;
}

// Blocking write block
size_t SerialCDC::write(const uint8_t *buffer, size_t buflen) noexcept
{
	const size_t ret = buflen;
	for (;;)
	{
		buflen -= txBuffer.PutBlock(buffer, buflen);
		if (buflen == 0)
		{
			StartSending();
			break;
		}
		txWaitingTask = RTOSIface::GetCurrentTask();
		StartSending();
		TaskBase::Take(50);
	}
	return ret;
}

void SerialCDC::StartSending() noexcept
{
	if (!sending)
	{
		sending = true;
		const size_t count = txBuffer.GetBlock(txTempBuffer, sizeof(txTempBuffer));
		if (count == 0)
		{
			sending = false;
		}
		else
		{
			cdcdf_acm_write(txTempBuffer, count);
		}
	}
}

void SerialCDC::StartReceiving() noexcept
{
	if (!receiving && rxBuffer.SpaceLeft() > sizeof(rxTempBuffer))
	{
		receiving = true;
		cdcdf_acm_read(rxTempBuffer, sizeof(rxTempBuffer));
	}
}

void SerialCDC::DataReceived(uint32_t count) noexcept
{
	rxBuffer.PutBlock(rxTempBuffer, count);
	receiving = false;
	StartReceiving();
}

// End
