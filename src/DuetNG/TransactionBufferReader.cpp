/*
 * TransactionBufferReader.cpp
 *
 *  Created on: 7 Jul 2016
 *      Author: David
 */

#include "TransactionBufferReader.h"

TransactionBufferReader::TransactionBufferReader(TransactionBuffer& tb)
	: buf(tb), offset(0), ok(true)
{
}

const char* TransactionBufferReader::GetString(size_t fieldWidth)
{
	if (ok)
	{
		if (offset + fieldWidth <= buf.GetLength())
		{
			buf.EnsureNull(offset + fieldWidth - 1);
			const char* rslt = buf.GetData() + offset;
			offset += fieldWidth;
			return rslt;
		}
		ok = false;
	}
	return "**ERROR**";
}

// End
