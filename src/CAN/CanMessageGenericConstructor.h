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

#include "CanMessageFormats.h"
#include "GCodes/GCodeResult.h"
#include "GCodes/GCodeBuffer/ParseException.h"

class GCodeBuffer;

class CanMessageGenericConstructor
{
public:
	CanMessageGenericConstructor(const ParamDescriptor *p_param) noexcept;

	// Populate from a GCode message. Throws if an error occurs.
	void PopulateFromCommand(GCodeBuffer& gb) THROWS_PARSE_ERROR;

	// Methods to add parameters
	void AddU64Param(char c, uint64_t v) THROWS_PARSE_ERROR;
	void AddUParam(char c, uint32_t v) THROWS_PARSE_ERROR;
	void AddIParam(char c, int32_t v) THROWS_PARSE_ERROR;
	void AddFParam(char c, float v) THROWS_PARSE_ERROR;
	void AddCharParam(char c, char v) THROWS_PARSE_ERROR;
	void AddStringParam(char c, const char* v) THROWS_PARSE_ERROR;

	GCodeResult SendAndGetResponse(CanMessageType msgType, CanAddress dest, const StringRef& reply) noexcept;

private:
	// Append a value to the data, returning true if it wouldn't fit
	void StoreValue(const void *vp, size_t sz) THROWS_PARSE_ERROR;

	// Append a value to the data, returning true if it wouldn't fit
	template<class T> void StoreValue(const T& val) THROWS_PARSE_ERROR { StoreValue(&val, sizeof(T)); }

	// Insert a value in the data, returning true if it wouldn't fit
	void InsertValue(const void *vp, size_t sz, size_t pos) THROWS_PARSE_ERROR;

	static ParseException ConstructParseException(const char *msg) noexcept
	{
		return ParseException(-1, -1, msg);
	}

	static ParseException ConstructParseException(const char *msg, uint32_t param) noexcept
	{
		return ParseException(-1, -1, msg, param);
	}

	const ParamDescriptor * const paramTable;
	size_t dataLen;
	CanMessageGeneric msg;
};

#endif

#endif /* SRC_CANMESSAGEGENERICCONSTRUCTOR_H_ */
