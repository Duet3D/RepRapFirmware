/*
 * AuxDevice.h
 *
 *  Created on: 3 Sep 2020
 *      Author: David
 */

#ifndef SRC_COMMS_AUXDEVICE_H_
#define SRC_COMMS_AUXDEVICE_H_

#include <RepRapFirmware.h>

enum class AuxMode : uint8_t
{
	disabled, raw, panelDue, device,
};

#if HAS_AUX_DEVICES

#include <Platform/OutputMemory.h>
#include <RTOSIface/RTOSIface.h>

#if SUPPORT_MODBUS_RTU
# include <Storage/CRC16.h>
# include <Hardware/IoPorts.h>
# include "Modbus.h"
#endif

class AsyncSerial;

class AuxDevice
{
public:
	AuxDevice() noexcept;

	void Init(AsyncSerial *p_uart, uint32_t p_baudRate) noexcept;
	bool IsEnabledForGCodeIo() const noexcept { return mode == AuxMode::raw || mode == AuxMode::panelDue; }
	void SetMode(AuxMode p_mode) noexcept;
	void SetBaudRate(uint32_t p_baudRate) noexcept { baudRate = p_baudRate; }			// must call SetMode after calling this to actually change the baud rate
	void Disable() noexcept;
	AuxMode GetMode() const noexcept { return mode; }
	uint32_t GetBaudRate() const noexcept { return baudRate; }
	bool IsRaw() const noexcept { return mode == AuxMode::raw; }

	void SendPanelDueMessage(const char *_ecv_array msg) noexcept;
	void AppendAuxReply(const char *_ecv_array msg, bool rawMessage) noexcept;
	void AppendAuxReply(OutputBuffer *_ecv_null reply, bool rawMessage) noexcept;
	bool Flush() noexcept;

	void Diagnostics(MessageType mt, unsigned int index) noexcept;

#if SUPPORT_MODBUS_RTU
	bool ConfigureDirectionPort(const char *_ecv_array pinName, const StringRef& reply) THROWS(GCodeException);
	void AppendDirectionPortName(const StringRef& reply) const noexcept;

	GCodeResult SendModbusRegisters(uint8_t p_slaveAddress, uint8_t p_function, uint16_t p_startRegister, uint16_t p_numRegisters, const uint8_t *_ecv_array data) noexcept;
	GCodeResult ReadModbusRegisters(uint8_t p_slaveAddress, uint8_t p_function, uint16_t p_startRegister, uint16_t p_numRegisters, uint8_t *_ecv_array data) noexcept
		pre(function == 3 || function == 4);
	GCodeResult ModbusRawTransaction(uint8_t p_slaveAddress, const uint8_t *_ecv_array rawDataOut, size_t numOut, uint8_t *_ecv_array rawDataIn, size_t numIn) noexcept;
	GCodeResult CheckModbusResult() noexcept;

	void TxEndedCallback() noexcept;
#endif

	GCodeResult SendUartData(const uint8_t *_ecv_array data, size_t len) noexcept;
	GCodeResult ReadUartData(uint8_t *_ecv_array data, size_t bytesToRead) noexcept;

private:
	uint32_t CalcTransmissionTime(unsigned int numChars) const noexcept;	// calculate the time in milliseconds to send or received the specified number of characters

#if SUPPORT_MODBUS_RTU
	void ModbusWriteByte(uint8_t b) noexcept;
	void ModbusWriteWord(uint16_t w) noexcept;
	uint8_t ModbusReadByte() noexcept;
	uint16_t ModbusReadWord() noexcept;
	GCodeResult ReleaseMutexAndCheckCrc() noexcept;

	static void GlobalTxEndedCallback(CallbackParameter cp) noexcept;

	static constexpr uint32_t ModbusResponseTimeout = 140;					// how many milliseconds we give the device time to respond, excluding transmission time
	static constexpr uint16_t MaxModbusRegisters = 100;						// the maximum number of registers we send or receive
	static constexpr uint16_t ModbusCrcInit = 0xFFFF;
#endif
	static constexpr uint32_t BusAvailableTimeout = 50;				// how many milliseconds we wait for the device to become available
	static constexpr uint32_t UartResponseTimeout = 200;			// how many milliseconds we wait for the device to respond, excluding transmission time

	AsyncSerial *_ecv_null uart;			// the underlying serial device
	Mutex mutex;
	volatile OutputStack outStack;			// output stack for use in raw or PanelDue mode
	uint32_t seq;							// sequence number for output in PanelDue mode
	uint32_t baudRate;
	AuxMode mode = AuxMode::disabled;		// whether disabled, raw, PanelDue mode or Modbus RTU mode

#if SUPPORT_MODBUS_RTU
	IoPort txNotRx;							// port used to switch the RS485 port between transmit and receive
	uint8_t *_ecv_array receivedData;
	uint32_t whenStartedTransmitting;
	CRC16 crc;
	uint16_t bytesTransmitted;
	uint16_t bytesExpected;
	uint16_t startRegister;
	uint16_t numRegistersOrDataWord;
	uint8_t slaveAddress;
	ModbusFunction function;
#endif
};

#endif

#endif /* SRC_COMMS_AUXDEVICE_H_ */
