/*
 * Modbus.h
 *
 *  Created on: 14 Aug 2024
 *      Author: David
 */

#ifndef SRC_COMMS_MODBUS_H_
#define SRC_COMMS_MODBUS_H_

#include <cstdint>

// Modbus RTU function codes
enum class ModbusFunction : uint16_t
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
	readDeviceId2 = 0x2B,

	generic = 0x100
};

#endif /* SRC_COMMS_MODBUS_H_ */
