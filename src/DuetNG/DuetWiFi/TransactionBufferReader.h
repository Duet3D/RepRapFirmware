/*
 * TransactionBufferReader.h
 *
 *  Created on: 7 Jul 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_TRANSACTIONBUFFERREADER_H_
#define SRC_DUETNG_TRANSACTIONBUFFERREADER_H_

#include "Core.h"
#include "TransactionBuffer.h"

// Helper class to extract values in sequence from a TransactionBuffer
class TransactionBufferReader
{
public:
	TransactionBufferReader(TransactionBuffer& tb);

	// Read a value of a primitive type
	template<class T> T GetPrimitive();

	// Read an array of primitive types
	template<class T> void GetArray(T * arr, size_t numElems);

	// Read a string
	const char* GetString(size_t fieldWidth);

	// Test whether we ran off the end
	bool IsOk() const { return ok; }

private:
	TransactionBuffer& buf;
	uint32_t offset;
	bool ok;
};

template <class T> T TransactionBufferReader::GetPrimitive()
{
	if (ok)
	{
		if (offset + sizeof(T) <= buf.GetLength())
		{
			T rslt = *reinterpret_cast<const T*>(buf.GetData() + offset);
			offset += sizeof(T);
			return rslt;
		}
		ok = false;
	}
	return static_cast<T>(0);
}

template<class T> void TransactionBufferReader::GetArray(T * arr, size_t numElems)
{
	for (size_t i = 0; i < numElems; ++i)
	{
		arr[i] = GetPrimitive<T>();
	}
}

#endif /* SRC_DUETNG_TRANSACTIONBUFFERREADER_H_ */
