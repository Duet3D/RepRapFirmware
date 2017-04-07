/*
 * TransactionBuffer.h
 *
 *  Created on: 17 May 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_TRANSACTIONBUFFER_H_
#define SRC_DUETNG_TRANSACTIONBUFFER_H_

#include <cstdint>

//-------------------------------------------------------------------------------------------
// Transaction buffer class
// ***** This must be kept in step with the corresponding class in the WiFi module code *****
const uint32_t maxSpiDataLength = 2048;

//---------------------------------------------------
// SPI transaction type field bits
// Byte 3 (MSB) is the packet type.
// Byte 2 holds flags
// Byte 1 is currently unused
// Byte 0 is the opcode if the packet is a request or info message, or the error code if it is a response.

// Packet types
const uint32_t trTypeRequest = 0x3A000000;          // this is a request
const uint32_t trTypeResponse = 0xB6000000;         // this is a response to a request
const uint32_t trTypeInfo = 0x93000000;             // this is an informational message that does not require a response

// Flags
const uint32_t ttDataTaken = 0x00010000;            // indicates to the SAM the the ESP8266 has read its data, or vice verse

// Opcodes for requests from web sever to Duet
const uint32_t ttRr = 0x01;                         // any request starting with "rr_"

// Opcodes for info messages from web server to Duet
const uint32_t ttNetworkInfo = 0x71;				// pass network info to Duet, e.g. when first connected
const uint32_t ttNetworkInfoOld = 0x70;				// pass network info to Duet, e.g. when first connected (old format)

// Opcodes for requests and info from Duet to web server
const uint32_t ttNetworkConfig = 0x80;              // set network configuration (SSID, password etc.)
const uint32_t ttNetworkEnable = 0x81;              // enable WiFi
const uint32_t ttGetNetworkInfo = 0x83;             // get IP address etc.

// Opcodes for info messages from Duet to server
const uint32_t ttMachineConfigChanged = 0x82;       // notify server that the machine configuration has changed significantly

// Last fragment bit
const uint32_t lastFragment = 0x80000000;

class TransactionBuffer
{
    uint32_t trType;					// type of transaction, and flags
    uint32_t seq;						// sequence number of the request
    uint32_t ip;						// requesting IP address
    uint32_t fragment;					// fragment number of this packet, top bit set if last fragment
    uint32_t dataLength;				// number of bytes of data following the header
    uint32_t data[maxSpiDataLength/4];  // the actual data, if needed
    uint32_t dummy;						// extra so that we can append a null and to allow overrun to be detected

public:
	static const uint32_t headerDwords = 5;

	static const uint32_t MaxTransferBytes = maxSpiDataLength + (4 * headerDwords);

	// Initialise
	void Init();

	// Mark this buffer empty
	void Clear();

	// When we send this packet, tell the ESP that we have taken its data
	void SetDataTaken() { trType |= ttDataTaken; }

	// Clear the DataTaken flag
	void ClearDataTaken() { trType &= ~ttDataTaken; }

	// Say whether data was taken
	bool DataWasTaken() const { return (trType & ttDataTaken) != 0; }

	// Return true if this buffer contains data
	bool IsReady() const
	{
		return trType != 0;
	}

	bool IsValid() const
	{
		return dataLength <= maxSpiDataLength;
	}

	bool IsEmpty() const
	{
		return trType == 0;
	}

	uint32_t GetOpcode() const
	{
		return trType;
	}

	uint32_t GetSeq() const
	{
		return seq;
	}

	uint32_t GetIp() const
	{
		return ip;
	}

	uint32_t GetFragment() const
	{
		return fragment;
	}

	const char* GetData() const
	{
		return reinterpret_cast<const char*>(data);
	}

	uint32_t GetLength() const
	{
		return dataLength;
	}

	const void* GetData(size_t& length) const
	{
		length = dataLength;
		return data;
	}

	// Append a null to a received buffer.
	// The buffer size is larger than the maximum allowed data length, so this is safe even if the message has the maximum allowed size.
	void AppendNull()
	{
		if (IsReady())
		{
			*((char*)data + dataLength) = 0;
		}
	}

	// Ensure there is a null at the specified data position.
	// Used to make sure that received fields that should be null terminated really are.
	void EnsureNull(size_t offset)
	pre(offset < dataLength)
	{
		*((char*)data + offset) = 0;
	}

	// Get the total SPI packet length in dwords
	uint32_t PacketLength() const
	{
		return (IsReady()) ? (dataLength + 3)/4 + headerDwords : headerDwords;
	}

	// Set up a message header in this buffer
	bool SetMessage(uint32_t tt, uint32_t p_ip, uint32_t frag);

	// Append data to the message in the output buffer, returning the length appended
	size_t AppendData(const void* dataToAppend, uint32_t length);

	// Append a single 32-bit integer to the output buffer
	size_t AppendU32(uint32_t val)
	pre(dataLength + sizeof(uint32_t) <= maxSpiDataLength)
	{
		return AppendData(&val, sizeof(val));
	}

	// Flag this buffer as the last fragment of the current message
	void SetLastFragment()
	{
		fragment |= lastFragment;
	}

    // Get the address and remaining size to write data into directly
    char *GetBuffer(size_t& length);

    // Tell the buffer that we have appended some data. Call this after writing data directly.
    void DataAppended(size_t length)
    pre(dataLength + length <= maxSpiDataLength)
    ;
};

#endif /* SRC_DUETNG_TRANSACTIONBUFFER_H_ */
