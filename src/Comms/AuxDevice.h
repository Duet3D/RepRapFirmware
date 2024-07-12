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
	enum class AuxMode : uint8_t
	{
		disabled, raw, panelDue,
#if SUPPORT_MODBUS_RTU
		modbus_rtu,
#endif
	};

	AuxDevice() noexcept;

	void Init(AsyncSerial *p_uart) noexcept;
	bool IsEnabledForGCodeIo() const noexcept { return mode == AuxMode::raw || mode == AuxMode::panelDue; }
	void SetMode(AuxMode p_mode, uint32_t baudRate) noexcept;
	void Disable() noexcept;

	AuxMode GetMode() const noexcept { return mode; }
	bool IsRaw() const noexcept { return mode == AuxMode::raw; }

	void SendPanelDueMessage(const char* msg) noexcept;
	void AppendAuxReply(const char *msg, bool rawMessage) noexcept;
	void AppendAuxReply(OutputBuffer *reply, bool rawMessage) noexcept;
	bool Flush() noexcept;

	void Diagnostics(MessageType mt, unsigned int index) noexcept;

#if SUPPORT_MODBUS_RTU
	GCodeResult SendModbusRegisters(uint8_t slaveAddress, uint16_t startRegister, uint16_t numRegisters, const uint16_t *data) noexcept;
	GCodeResult ReadModbusRegisters(uint8_t slaveAddress, uint16_t startRegister, uint16_t numRegisters, uint16_t *data) noexcept;
	GCodeResult CheckModbusResult(bool& success) noexcept;
#endif

private:

	AsyncSerial *uart;						// the underlying serial device
	Mutex mutex;
	volatile OutputStack outStack;			// output stack for use in raw or PanelDue mode
	uint32_t seq;							// sequence number for output in PanelDue mode
	AuxMode mode;							// whether disabled, raw, PanelDue mode or Modbus RTU mode

#if SUPPORT_MODBUS_RTU
	//TODO extra variables for modbus send/receive state
#endif
};

#endif

#endif /* SRC_COMMS_AUXDEVICE_H_ */
