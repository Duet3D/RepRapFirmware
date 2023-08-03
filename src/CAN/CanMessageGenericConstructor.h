/*
 * CanMessageGenericConstructor.h
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#ifndef SRC_CANMESSAGEGENERICCONSTRUCTOR_H_
#define SRC_CANMESSAGEGENERICCONSTRUCTOR_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

#include <CanMessageFormats.h>
#include <GCodes/GCodeException.h>
#include <CanMessageGenericTableFormat.h>

class GCodeBuffer;

class CanMessageGenericConstructor
{
public:
	CanMessageGenericConstructor(const ParamDescriptor *p_param) noexcept;

	// Populate from a GCode message. Throws if an error occurs.
	void PopulateFromCommand(GCodeBuffer& gb) THROWS(GCodeException);

	// Methods to add parameters
	void AddU64Param(char c, uint64_t v) THROWS(GCodeException);
	void AddUParam(char c, uint32_t v) THROWS(GCodeException);
	void AddIParam(char c, int32_t v) THROWS(GCodeException);
	void AddFParam(char c, float v) THROWS(GCodeException);
	void AddCharParam(char c, char v) THROWS(GCodeException);
	void AddStringParam(char c, const char* v) THROWS(GCodeException);
	void AddDriverIdParam(char c, DriverId did) THROWS(GCodeException);
	void AddFloatArrayParam(char c, const float *v, size_t numV) THROWS(GCodeException);

	GCodeResult SendAndGetResponse(CanMessageType msgType, CanAddress dest, const StringRef& reply, uint8_t *extra = nullptr) const noexcept;

private:
	// Return the correct position in the data to insert a parameter. If successful, add the bit to the parameter map and pass back the expected parameter type and size; else throw.
	unsigned int FindInsertPoint(char c, ParamDescriptor::ParamType& t, size_t &sz) THROWS(GCodeException);

	// Append a value to the data, returning true if it wouldn't fit
	void StoreValue(const void *vp, size_t sz) THROWS(GCodeException);

	// Append a value to the data, returning true if it wouldn't fit
	template<class T> void StoreValue(const T& val) THROWS(GCodeException) { StoreValue(&val, sizeof(T)); }

	// Insert a value in the data, returning true if it wouldn't fit
	void InsertValue(const void *vp, size_t sz, size_t pos) THROWS(GCodeException);

	const ParamDescriptor * const paramTable;
	size_t dataLen;
	CanMessageGeneric msg;
};

#endif

#endif /* SRC_CANMESSAGEGENERICCONSTRUCTOR_H_ */
