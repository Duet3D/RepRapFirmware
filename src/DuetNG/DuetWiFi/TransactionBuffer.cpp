/*
 * TransactionBuffer.cpp
 *
 *  Created on: 17 May 2016
 *      Author: David
 */

#include "Core.h"
#include "TransactionBuffer.h"
#include <cstring>

void TransactionBuffer::Clear()
{
	trType = 0;
	ip = 0;
	seq = 0;
	fragment = 0;
	dataLength = 0;
}

bool TransactionBuffer::SetMessage(uint32_t tt, uint32_t p_ip, uint32_t frag)
{
	if (IsReady())
	{
		return false;
	}
	trType = tt;
	ip = p_ip;
	seq = 0;
	fragment = frag;
	dataLength = 0;
	return true;
}

// Append data to the message in the output buffer, returning the length appended
size_t TransactionBuffer::AppendData(const void* dataToAppend, uint32_t length)
{
	uint32_t spaceLeft = maxSpiDataLength - dataLength;
	uint32_t bytesToCopy = min<size_t>(length, spaceLeft);
	memcpy((char*)data + dataLength, dataToAppend, bytesToCopy);
	dataLength += bytesToCopy;
	return bytesToCopy;
}

// Get the address and size to write data into
char *TransactionBuffer::GetBuffer(size_t& length)
{
	length = maxSpiDataLength - dataLength;
	return reinterpret_cast<char*>(data) + dataLength;
}

// Say we have appended some data
void TransactionBuffer::DataAppended(size_t length)
{
	dataLength += length;
}

// End
