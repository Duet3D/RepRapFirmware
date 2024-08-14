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

#if SUPPORT_MODBUS_RTU
# include <Storage/CRC16.h>
# include <Hardware/IoPorts.h>
# include "Modbus.h"
#endif

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

	void Init(AsyncSerial *p_uart, uint32_t p_baudRate) noexcept;
	bool IsEnabledForGCodeIo() const noexcept { return mode == AuxMode::raw || mode == AuxMode::panelDue; }
	void SetMode(AuxMode p_mode) noexcept;
	void SetBaudRate(uint32_t p_baudRate) noexcept { baudRate = p_baudRate; }			// must call SetMode after calling this to actually change the baud rate
	void Disable() noexcept;
	AuxMode GetMode() const noexcept { return mode; }
	uint32_t GetBaudRate() const noexcept { return baudRate; }
	bool IsRaw() const noexcept { return mode == AuxMode::raw; }

	void SendPanelDueMessage(const char* msg) noexcept;
	void AppendAuxReply(const char *msg, bool rawMessage) noexcept;
	void AppendAuxReply(OutputBuffer *reply, bool rawMessage) noexcept;
	bool Flush() noexcept;

	void Diagnostics(MessageType mt, unsigned int index) noexcept;

#if SUPPORT_MODBUS_RTU
	bool ConfigureDirectionPort(const char *pinName, const StringRef& reply) THROWS(GCodeException);
	void AppendDirectionPortName(const StringRef& reply) const noexcept;

	GCodeResult SendModbusRegisters(uint8_t p_slaveAddress, uint8_t p_function, uint16_t p_startRegister, uint16_t p_numRegisters, const uint8_t *data) noexcept;
	GCodeResult ReadModbusRegisters(uint8_t p_slaveAddress, uint8_t p_function, uint16_t p_startRegister, uint16_t p_numRegisters, uint8_t *data) noexcept
		pre(function == 3 || function == 4);
	GCodeResult CheckModbusResult() noexcept;

	void TxEndedCallback() noexcept;
#endif

private:

#if SUPPORT_MODBUS_RTU
	void ModbusWriteByte(uint8_t b) noexcept;
	void ModbusWriteWord(uint16_t w) noexcept;
	uint8_t ModbusReadByte() noexcept;
	uint16_t ModbusReadWord() noexcept;
	GCodeResult ReleaseMutexAndCheckCrc() noexcept;
	uint32_t CalcTransmissionTime(unsigned int numChars) const noexcept;	// calculate the time in milliseconds to send or received the specified number of characters

	static void GlobalTxEndedCallback(CallbackParameter cp) noexcept;

	static constexpr uint32_t ModbusBusAvailableTimeout = 50;				// how many milliseconds we wait for the device to become available
	static constexpr uint32_t ModbusResponseTimeout = 140;					// how many milliseconds we give the device time to respond, excluding transmission time
	static constexpr uint16_t MaxModbusRegisters = 100;						// the maximum number of registers we send or receive
	static constexpr uint16_t ModbusCrcInit = 0xFFFF;
#endif

	AsyncSerial *uart;						// the underlying serial device
	Mutex mutex;
	volatile OutputStack outStack;			// output stack for use in raw or PanelDue mode
	uint32_t seq;							// sequence number for output in PanelDue mode
	uint32_t baudRate;
	AuxMode mode = AuxMode::disabled;		// whether disabled, raw, PanelDue mode or Modbus RTU mode

#if SUPPORT_MODBUS_RTU
	IoPort txNotRx;							// port used to switch the RS485 port between transmit and receive
	uint8_t *receivedData;
	uint32_t whenStartedTransmitting;
	CRC16 crc;
	uint16_t bytesTransmitted;
	uint16_t bytesExpected;
	uint16_t startRegister;
	uint16_t numRegisters;
	uint8_t slaveAddress;
	ModbusFunction function;
#endif
};

#endif

#endif /* SRC_COMMS_AUXDEVICE_H_ */
