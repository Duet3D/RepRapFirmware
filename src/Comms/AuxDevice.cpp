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
#include <AsyncSerial.h>

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
		if (p_mode == AuxMode::disabled)
		{
			Disable();
		}
		else
		{
#if SUPPORT_MODBUS_RTU
			uart->SetOnTxEndedCallback((p_mode == AuxMode::device) ? GlobalTxEndedCallback : nullptr, CallbackParameter(this));
#endif
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

void AuxDevice::SendPanelDueMessage(const char *_ecv_array msg) noexcept
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

void AuxDevice::AppendAuxReply(const char *_ecv_array msg, bool rawMessage) noexcept
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

void AuxDevice::AppendAuxReply(OutputBuffer *_ecv_null reply, bool rawMessage) noexcept
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
		OutputBuffer *_ecv_null auxOutputBuffer = outStack.GetFirstItem();
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
bool AuxDevice::ConfigureDirectionPort(const char *pinName, const StringRef& reply) THROWS(GCodeException)
{
	return txNotRx.AssignPort(pinName, reply, PinUsedBy::gpout, PinAccess::write0);
}

void AuxDevice::AppendDirectionPortName(const StringRef& reply) const noexcept
{
	txNotRx.AppendPinName(reply);
}

// Send some Modbus registers. Returns GCodeResult::error if we failed to acquire the mutex, GCodeResult::ok if we sent the command.
// After receiving the GCodeResult::ok response the caller must call CheckModbusResult until it doesn't return GCodeResult::notFinished.
// If the function code requires sending 16-bit data then 'data' must be 16-bit aligned
GCodeResult AuxDevice::SendModbusRegisters(uint8_t p_slaveAddress, uint8_t p_function, uint16_t p_startRegister, uint16_t p_numRegisters, const uint8_t *data) noexcept
{
	if (p_numRegisters == 0 || p_numRegisters > MaxModbusRegisters)
	{
		return GCodeResult::badOrMissingParameter;
	}

	if (!mutex.Take(BusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	uart->ClearTransmitBuffer();
	uart->DisableTransmit();
	crc.Reset(ModbusCrcInit);
	bytesTransmitted = 0;

	slaveAddress = p_slaveAddress;
	ModbusWriteByte(slaveAddress);
	function = (ModbusFunction)p_function;
	ModbusWriteByte((uint8_t)function);
	startRegister = p_startRegister;
	ModbusWriteWord(startRegister);
	switch (function)
	{
	case ModbusFunction::writeSingleCoil:
	case ModbusFunction::writeSingleRegister:
		numRegistersOrDataWord = ((const uint16_t*)data)[0];
		ModbusWriteWord(numRegistersOrDataWord);
		bytesExpected = 8;
		break;

	case ModbusFunction::writeMultipleCoils:
		numRegistersOrDataWord = p_numRegisters;
		ModbusWriteWord(numRegistersOrDataWord);
		ModbusWriteByte((uint8_t)((numRegistersOrDataWord + 7u)/8u));
		for (size_t i = 0; i < (numRegistersOrDataWord + 7u)/8u; ++i)
		{
			ModbusWriteByte(data[i]);
		}
		bytesExpected = 8;
		break;

	case ModbusFunction::writeMultipleRegisters:
	default:
		numRegistersOrDataWord = p_numRegisters;
		ModbusWriteWord(numRegistersOrDataWord);
		ModbusWriteByte((uint8_t)(2 * numRegistersOrDataWord));
		for (size_t i = 0; i < numRegistersOrDataWord; ++i)
		{
			ModbusWriteWord(((const uint16_t*)data)[i]);
		}
		bytesExpected = 8;
		break;
	}

	uart->write((uint8_t)crc.Get());
	uart->write((uint8_t)(crc.Get() >> 8));

	txNotRx.WriteDigital(true);								// set RS485 direction to transmit
	delay(CalcTransmissionTime(4));							// Modbus specifies a 3.5 character interval
	uart->ClearReceiveBuffer();
	uart->EnableTransmit();
	whenStartedTransmitting = millis();

	return GCodeResult::ok;
}

// Read some Modbus registers. Returns GCodeResult::error if we failed to acquire the mutex, GCodeResult::ok if we sent the command.
// After receiving the GCodeResult::ok response the caller must call CheckModbusResult until it doesn't return GCodeResult::notFinished.
// If the function code calls for receiving word data then 'data' must be aligned on a 16-bit boundary
GCodeResult AuxDevice::ReadModbusRegisters(uint8_t p_slaveAddress, uint8_t p_function, uint16_t p_startRegister, uint16_t p_numRegisters, uint8_t *data) noexcept
{
	if (   p_numRegisters == 0
		|| p_numRegisters > MaxModbusRegisters
		|| (   p_function != (uint8_t)ModbusFunction::readHoldingRegisters
			&& p_function != (uint8_t)ModbusFunction::readInputRegisters
			&& p_function != (uint8_t)ModbusFunction::readCoils
			&& p_function != (uint8_t)ModbusFunction::readDiscreteInputs
		   )
	   )
	{
		return GCodeResult::badOrMissingParameter;
	}

	if (!mutex.Take(BusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	uart->ClearTransmitBuffer();
	uart->DisableTransmit();
	crc.Reset(ModbusCrcInit);
	bytesTransmitted = 0;

	slaveAddress = p_slaveAddress;
	ModbusWriteByte(slaveAddress);
	function = (ModbusFunction)p_function;
	ModbusWriteByte((uint8_t)function);
	startRegister = p_startRegister;
	ModbusWriteWord(startRegister);
	numRegistersOrDataWord = p_numRegisters;
	ModbusWriteWord(numRegistersOrDataWord);
	uart->write((uint8_t)crc.Get());
	uart->write((uint8_t)(crc.Get() >> 8));

	txNotRx.WriteDigital(true);								// set port to transmit
	delay(CalcTransmissionTime(4));							// Modbus specifies a 3.5 character interval
	uart->ClearReceiveBuffer();
	uart->EnableTransmit();
	whenStartedTransmitting = millis();

	switch (function)
	{
	case ModbusFunction::readCoils:
	case ModbusFunction::readDiscreteInputs:
		bytesExpected = 5 + (numRegistersOrDataWord + 7)/8;
		break;

	case ModbusFunction::readHoldingRegisters:
	case ModbusFunction::readInputRegisters:
	default:
		bytesExpected = 5 + 2 * numRegistersOrDataWord;
		break;
	}
	receivedData = data;
	return GCodeResult::ok;
}

GCodeResult AuxDevice::ModbusRawTransaction(uint8_t p_slaveAddress, const uint8_t *_ecv_array rawDataOut, size_t numOut, uint8_t *_ecv_array rawDataIn, size_t numIn) noexcept
{
	if (!mutex.Take(BusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	uart->ClearTransmitBuffer();
	uart->DisableTransmit();
	crc.Reset(ModbusCrcInit);
	bytesTransmitted = 0;

	slaveAddress = p_slaveAddress;
	ModbusWriteByte(slaveAddress);
	function = ModbusFunction::generic;
	while (numOut != 0)
	{
		ModbusWriteByte(*rawDataOut++);
		--numOut;
	}
	uart->write((uint8_t)crc.Get());
	uart->write((uint8_t)(crc.Get() >> 8));

	txNotRx.WriteDigital(true);								// set port to transmit
	delay(CalcTransmissionTime(4));							// Modbus specifies a 3.5 character interval
	uart->ClearReceiveBuffer();
	uart->EnableTransmit();
	whenStartedTransmitting = millis();

	bytesExpected = numIn + 3;
	receivedData = rawDataIn;
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
	if (ModbusReadByte() == slaveAddress)
	{
		if (function == ModbusFunction::generic)
		{
			for (size_t i = 0; i + 3 < bytesExpected; ++i)
			{
				*receivedData++ = ModbusReadByte();
			}
			return ReleaseMutexAndCheckCrc();
		}
		else if (ModbusReadByte() == (uint8_t)function)
		{
			switch(function)
			{
			case ModbusFunction::writeSingleCoil:
			case ModbusFunction::writeSingleRegister:
			case ModbusFunction::writeMultipleCoils:
			case ModbusFunction::writeMultipleRegisters:
				if (ModbusReadWord() == startRegister && ModbusReadWord() == numRegistersOrDataWord)
				{
					return ReleaseMutexAndCheckCrc();
				}
				break;

			case ModbusFunction::readCoils:
			case ModbusFunction::readDiscreteInputs:
				if (ModbusReadByte() == (numRegistersOrDataWord + 7u)/8u)
				{
					for (size_t i = 0; i < (numRegistersOrDataWord + 7u)/8u; ++i)
					{
						*receivedData++ = ModbusReadByte();
					}
					return ReleaseMutexAndCheckCrc();
				}
				break;

			case ModbusFunction::readInputRegisters:
			case ModbusFunction::readHoldingRegisters:
				if (ModbusReadByte() == 2 * numRegistersOrDataWord)
				{
					while (numRegistersOrDataWord != 0)
					{
						*(uint16_t*)receivedData = ModbusReadWord();
						receivedData += sizeof(uint16_t);
						--numRegistersOrDataWord;
					}
					return ReleaseMutexAndCheckCrc();
				}
				break;

			default:
				break;
			}
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
#if 0
	debugPrintf("Modbus Rx %02x\n", b);
#endif
	return b;
}

uint16_t AuxDevice::ModbusReadWord() noexcept
{
	const uint16_t hi = (uint16_t)ModbusReadByte() << 8;
	return hi | (uint16_t)ModbusReadByte();
}

GCodeResult AuxDevice::ReleaseMutexAndCheckCrc() noexcept
{
	const uint16_t crcLo = (uint16_t)uart->read();
	const uint16_t recdCrc = ((uint16_t)uart->read() << 8) | crcLo;
	mutex.Release();
	return (recdCrc == crc.Get()) ? GCodeResult::ok : GCodeResult::error;
}

// Callback for transmission ended
/*static*/ void AuxDevice::GlobalTxEndedCallback(CallbackParameter cp) noexcept
{
	((AuxDevice*)cp.vp)->TxEndedCallback();
}

// Callback for transmission ended
void AuxDevice::TxEndedCallback() noexcept
{
	uart->DisableTransmit();
	txNotRx.WriteDigital(false);
}

#endif

// Calculate the time in milliseconds to delay to allow time to send or received the specified number of characters
uint32_t AuxDevice::CalcTransmissionTime(unsigned int numChars) const noexcept
{
	// In the following we need to do +1 to round up the a whole number of milliseconds, and another +1 because a call to delay() may delay up to 1 tick less than we requested
	return (numChars * 11000)/baudRate + 2;						// Modbus specifies 2 stop bits if parity not used, therefore 11 bits/character
}

// Send some data to the Uart. Returns GCodeResult::error if we failed to acquire the mutex, GCodeResult::ok if we sent the data.
GCodeResult AuxDevice::SendUartData(const uint8_t *_ecv_array data, size_t len) noexcept
{
	if (!mutex.Take(BusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	uart->ClearTransmitBuffer();
	uart->DisableTransmit();

	for (size_t i = 0; i < len; i++)
	{
		uart->write((const uint8_t)data[i]);
	}

#if SUPPORT_MODBUS_RTU
	txNotRx.WriteDigital(true);								// set RS485 direction to transmit
	delay(CalcTransmissionTime(4));							// Modbus specifies a 3.5 character interval
#endif
	uart->ClearReceiveBuffer();
	uart->EnableTransmit();

	mutex.Release();

	return GCodeResult::ok;
}

GCodeResult AuxDevice::ReadUartData(uint8_t *_ecv_array data, size_t bytesToRead) noexcept
{
	if (!mutex.Take(BusAvailableTimeout))
	{
		return GCodeResult::error;
	}

	if (bytesToRead == 0)
	{
		uart->ClearReceiveBuffer();
		return GCodeResult::ok;
	}


	const uint32_t start = millis();
	const uint32_t expectedCommsTime = CalcTransmissionTime(bytesToRead);
	while (uart->available() < (int)bytesToRead)
	{
		// Check whether we should time out
		if (millis() - start < expectedCommsTime + UartResponseTimeout)
		{
			delay(2);
			continue;
		}
		mutex.Release();
		return GCodeResult::error;					// timed out
	}

	// If we get here then we received sufficient bytes for a valid reply
	for (size_t i = 0; i < bytesToRead; i++)
	{
		data[i] = (uint8_t)uart->read();
	}

	mutex.Release();
	return GCodeResult::ok;
}

#endif

// End
