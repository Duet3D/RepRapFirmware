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
	GCodeResult SendModbusRegisters(uint8_t p_slaveAddress, uint16_t p_startRegister, uint16_t p_numRegisters, const uint16_t *data) noexcept;
	GCodeResult ReadModbusRegisters(uint8_t p_slaveAddress, uint16_t p_startRegister, uint16_t p_numRegisters, uint16_t *data) noexcept;
	GCodeResult CheckModbusResult() noexcept;
#endif

private:

#if SUPPORT_MODBUS_RTU
	void ModbusWriteByte(uint8_t b) noexcept;
	void ModbusWriteWord(uint16_t w) noexcept;
	uint8_t ModbusReadByte() noexcept;
	uint16_t ModbusReadWord() noexcept;

	static constexpr uint32_t ModbusBusAvailableTimeout = 50;			// how many milliseconds we wait for the device to become available
	static constexpr uint32_t ModbusResponseTimeout = 20;				// how many milliseconds we give the device time to respond, excluding transmission time
	static constexpr uint16_t MaxModbusRegisters = 100;					// the maximum number of registers we send or receive
	static constexpr uint16_t ModbusCrcInit = 0xFFFF;

	enum class ModbusFunction : uint8_t
	{
		readCoils = 0x01,
		readDiscreteInputs = 0x02,
		readHoldingRegisters = 0x03,
		readInputRegisters = 0x04,
		writeSingleCoil = 0x05,
		writeSingleRegister = 0x06,
		writeMultipleCoils = 0x0F,
		writeMultipleRegisters = 0x10,
		readDeviceId1 = 0x0E,
		readDeviceId2 = 0x2B
	};

#endif

	AsyncSerial *uart;						// the underlying serial device
	Mutex mutex;
	volatile OutputStack outStack;			// output stack for use in raw or PanelDue mode
	uint32_t seq;							// sequence number for output in PanelDue mode
	AuxMode mode;							// whether disabled, raw, PanelDue mode or Modbus RTU mode

#if SUPPORT_MODBUS_RTU
	uint16_t *receivedRegisters;
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
