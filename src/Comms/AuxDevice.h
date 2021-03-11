/*
 * AuxDevice.h
 *
 *  Created on: 3 Sep 2020
 *      Author: David
 */

#ifndef SRC_COMMS_AUXDEVICE_H_
#define SRC_COMMS_AUXDEVICE_H_

#include <RepRapFirmware.h>
#if HAS_AUX_DEVICES
#include <Platform/OutputMemory.h>
#include <RTOSIface/RTOSIface.h>

class AuxDevice
{
public:
	AuxDevice() noexcept;

	void Init(UARTClass *p_uart) noexcept;
	bool IsEnabled() const noexcept { return enabled; }
	void Enable(uint32_t baudRate) noexcept;
	void Disable() noexcept;

	bool IsRaw() const noexcept { return raw; }
	void SetRaw(bool p_raw) { raw = p_raw; }

	void SendPanelDueMessage(const char* msg) noexcept;
	void AppendAuxReply(const char *msg, bool rawMessage) noexcept;
	void AppendAuxReply(OutputBuffer *reply, bool rawMessage) noexcept;
	bool Flush() noexcept;

	void Diagnostics(MessageType mt, unsigned int index) noexcept;

private:
	UARTClass *uart;
	volatile OutputStack outStack;
	Mutex mutex;
	uint32_t seq;							// sequence number for output
	bool enabled;							// is it initialised and running?
	bool raw;								// true if device is in raw mode
};

#endif

#endif /* SRC_COMMS_AUXDEVICE_H_ */
