/*
 * AuxDevice.cpp
 *
 *  Created on: 3 Sep 2020
 *      Author: David
 */

#include "AuxDevice.h"

#if HAS_AUX_DEVICES
#include <Platform/RepRap.h>
#include <Platform/Platform.h>

AuxDevice::AuxDevice() noexcept : uart(nullptr), seq(0), mode(AuxMode::disabled)
{
}

void AuxDevice::Init(AsyncSerial *p_uart, uint32_t p_baudRate) noexcept
{
	uart = p_uart;
#if SAME5x
	uart->setInterruptPriority(NvicPriorityAuxUartRx, NvicPriorityAuxUartTx);
#else
	uart->setInterruptPriority(NvicPriorityAuxUart);
#endif
	baudRate = p_baudRate;
	mutex.Create("Aux");
}

void AuxDevice::SetMode(AuxMode p_mode) noexcept
{
	if (uart != nullptr)
	{
		if (mode == AuxMode::disabled)
		{
			Disable();
		}
		else
		{
			uart->begin(baudRate);
			mode = p_mode;
		}
	}
}

void AuxDevice::Disable() noexcept
{
	if (mode != AuxMode::disabled)
	{
		uart->end();
		outStack.ReleaseAll();
		mode = AuxMode::disabled;
	}
}

void AuxDevice::SendPanelDueMessage(const char* msg) noexcept
{
	if (mode == AuxMode::panelDue)
	{
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			buf->printf("{\"message\":\"%.s\"}\n", msg);
			outStack.Push(buf);
			Flush();
		}
	}
}

void AuxDevice::AppendAuxReply(const char *msg, bool rawMessage) noexcept
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (msg[0] != 0 && IsEnabledForGCodeIo())
	{
		MutexLocker lock(mutex);
		OutputBuffer *buf;
		if (OutputBuffer::Allocate(buf))
		{
			if (rawMessage || mode == AuxMode::raw)
			{
				buf->copy(msg);
			}
			else
			{
				seq++;
				buf->printf("{\"seq\":%" PRIu32 ",\"resp\":\"%.s\"}\n", seq, msg);
			}
			outStack.Push(buf);
		}
	}
}

void AuxDevice::AppendAuxReply(OutputBuffer *reply, bool rawMessage) noexcept
{
	// Discard this response if either no aux device is attached or if the response is empty
	if (reply == nullptr || reply->Length() == 0 || !IsEnabledForGCodeIo())
	{
		OutputBuffer::ReleaseAll(reply);
	}
	else
	{
		MutexLocker lock(mutex);
		if (rawMessage || mode == AuxMode::raw)
		{
			outStack.Push(reply);
		}
		else
		{
			OutputBuffer *buf;
			if (OutputBuffer::Allocate(buf))
			{
				seq++;
				buf->printf("{\"seq\":%" PRIu32 ",\"resp\":", seq);
				buf->EncodeReply(reply);
				buf->cat("}\n");
				outStack.Push(buf);
			}
			else
			{
				OutputBuffer::ReleaseAll(reply);
			}
		}
	}
}

bool AuxDevice::Flush() noexcept
{
	bool hasMore = !outStack.IsEmpty();
	if (hasMore)
	{
		MutexLocker lock(mutex);
		OutputBuffer *auxOutputBuffer = outStack.GetFirstItem();
		if (auxOutputBuffer == nullptr)
		{
			(void)outStack.Pop();
		}
		else if (!IsEnabledForGCodeIo())
		{
			OutputBuffer::ReleaseAll(auxOutputBuffer);
			(void)outStack.Pop();
		}
		else
		{
			const size_t bytesToWrite = min<size_t>(uart->canWrite(), auxOutputBuffer->BytesLeft());
			if (bytesToWrite > 0)
			{
				uart->print(auxOutputBuffer->Read(bytesToWrite), bytesToWrite);
			}

			if (auxOutputBuffer->BytesLeft() == 0)
			{
				outStack.ReleaseFirstItem();
			}
		}
		hasMore = !outStack.IsEmpty();
	}
	return hasMore;
}

void AuxDevice::Diagnostics(MessageType mt, unsigned int index) noexcept
{
	if (mode != AuxMode::disabled)
	{
		const AsyncSerial::Errors errs = uart->GetAndClearErrors();
		reprap.GetPlatform().MessageF(mt, "Aux%u errors %u,%u,%u\n", index, (unsigned int)errs.uartOverrun, (unsigned int)errs.bufferOverrun, (unsigned int)errs.framing);
	}
}

#if SUPPORT_MODBUS_RTU

// Configure the Tx/!Rx port returning true if success
bool AuxDevice::ConfigureDirectionPort(const char *pinName, GCodeBuffer& gb, const StringRef& reply) noexcept
{
	return txNotRx.AssignPort(gb, reply, PinUsedBy::gpout, PinAccess::write0);
}

// Send some Modbus registers. May return GCodeResult notFinished if the buffer had insufficient room  but may have enough later, or the port was busy.
GCodeResult AuxDevice::SendModbusRegisters(uint8_t p_slaveAddress, uint16_t p_startRegister, uint16_t p_numRegisters, const uint16_t *data) noexcept
{
	if (numRegisters == 0 || numRegisters > MaxModbusRegisters)
	{
		return GCodeResult::badOrMissingParameter;
	}

	if (!mutex.Take(ModbusBusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	uart->ClearTransmitBuffer();
	uart->DisableTransmit();
	crc.Reset(ModbusCrcInit);
	bytesTransmitted = 0;

	slaveAddress = p_slaveAddress;
	ModbusWriteByte(slaveAddress);
	function = ModbusFunction::writeMultipleRegisters;
	ModbusWriteByte((uint8_t)function);
	startRegister = p_startRegister;
	ModbusWriteWord(startRegister);
	numRegisters = p_numRegisters;
	ModbusWriteWord(numRegisters);
	ModbusWriteByte((uint8_t)(2 * numRegisters));
	for (size_t i = 0; i < numRegisters; ++i)
	{
		ModbusWriteWord(data[i]);
	}

	uart->write((uint8_t)crc.Get());						// CRC is sent low byte first
	uart->write((uint8_t)(crc.Get() >> 8));

	txNotRx.WriteDigital(true);								// set RS485 direction to transmit
	delay(CalcTransmissionTime(4));							// Modbus specifies a 3.5 character interval
	uart->ClearReceiveBuffer();
	uart->EnableTransmit();
	whenStartedTransmitting = millis();

	bytesExpected = 8;
	return GCodeResult::ok;
}

// Read some Modbus registers. May return GCodeResult notFinished if the buffer had insufficient room  but may have enough later, or the port was busy.
GCodeResult AuxDevice::ReadModbusRegisters(uint8_t p_slaveAddress, uint16_t p_startRegister, uint16_t p_numRegisters, uint16_t *data) noexcept
{
	if (numRegisters == 0 || numRegisters > MaxModbusRegisters)
	{
		return GCodeResult::badOrMissingParameter;
	}

	if (!mutex.Take(ModbusBusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	uart->ClearTransmitBuffer();
	uart->DisableTransmit();
	crc.Reset(ModbusCrcInit);
	bytesTransmitted = 0;

	slaveAddress = p_slaveAddress;
	ModbusWriteByte(slaveAddress);
	function = ModbusFunction::readInputRegisters;
	ModbusWriteByte((uint8_t)function);
	startRegister = p_startRegister;
	ModbusWriteWord(startRegister);
	numRegisters = p_numRegisters;
	ModbusWriteWord(numRegisters);
	uart->write((uint8_t)crc.Get());						// CRC is sent low byte first
	uart->write((uint8_t)(crc.Get() >> 8));

	txNotRx.WriteDigital(true);								// set port to transmit
	delay(CalcTransmissionTime(4));							// Modbus specifies a 3.5 character interval
	uart->ClearReceiveBuffer();
	uart->EnableTransmit();
	whenStartedTransmitting = millis();

	bytesExpected = 5 + 2 * numRegisters;
	receivedRegisters = data;
	return GCodeResult::ok;
}

// Check whether the Modbus operation completed.
GCodeResult AuxDevice::CheckModbusResult() noexcept
{
	if (mutex.GetHolder() != TaskBase::GetCallerTaskHandle())
	{
		return GCodeResult::error;
	}

	if (uart->available() < bytesExpected)
	{
		// Check whether we should time out
		const uint32_t expectedCommsTime = CalcTransmissionTime(bytesTransmitted + bytesExpected);
		if (millis() - whenStartedTransmitting < expectedCommsTime + ModbusResponseTimeout)
		{
			return GCodeResult::notFinished;
		}

		mutex.Release();
		return GCodeResult::error;					// timed out
	}

	// If we get here then we received sufficient bytes for a valid reply
	crc.Reset(ModbusCrcInit);
	if (ModbusReadByte() == slaveAddress && ModbusReadByte() == (uint8_t)function)
	{
		switch(function)
		{
		case ModbusFunction::writeMultipleRegisters:
			if (ModbusReadWord() == startRegister && ModbusReadWord() == numRegisters)
			{
				const uint16_t crcLo = (uint16_t)uart->read();
				const uint16_t recdCrc = ((uint16_t)uart->read() << 8) | crcLo;
				mutex.Release();
				return (recdCrc == crc.Get()) ? GCodeResult::ok : GCodeResult::error;
			}
			break;

		case ModbusFunction::readInputRegisters:
			if (ModbusReadWord() == startRegister && ModbusReadWord() == numRegisters && ModbusReadByte() == 2 * numRegisters)
			{
				while (numRegisters != 0)
				{
					*receivedRegisters++ = ModbusReadWord();
					--numRegisters;
				}
				const uint16_t crcLo = (uint16_t)uart->read();
				const uint16_t recdCrc = ((uint16_t)uart->read() << 8) | crcLo;
				mutex.Release();
				return (recdCrc == crc.Get()) ? GCodeResult::ok : GCodeResult::error;
			}
			break;

		default:
			break;
		}
	}

	mutex.Release();
	return GCodeResult::error;
}

void AuxDevice::ModbusWriteByte(uint8_t b) noexcept
{
	crc.UpdateModbus(b);
	uart->write(b);
}

void AuxDevice::ModbusWriteWord(uint16_t w) noexcept
{
	ModbusWriteByte((uint8_t)(w >> 8));
	ModbusWriteByte((uint8_t)w);
}

uint8_t AuxDevice::ModbusReadByte() noexcept
{
	const uint8_t b = uart->read();
	crc.UpdateModbus(b);
	return b;
}

uint16_t AuxDevice::ModbusReadWord() noexcept
{
	const uint16_t hi = (uint16_t)ModbusReadByte() << 8;
	return hi | (uint16_t)ModbusReadByte();
}

// Calculate the time in milliseconds to send or received the specified number of characters
uint32_t AuxDevice::CalcTransmissionTime(unsigned int numChars) const noexcept
{
	return (numChars * 11000)/baudRate + 1;						// Modbus specifies 2 stop bits if parity not used, therefore 11 bits/character
}

#endif

#endif

// End
