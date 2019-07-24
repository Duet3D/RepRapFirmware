/*
 * CanMessageGenericConstructo.h
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#ifndef SRC_CANMESSAGEGENERICCONSTRUCTOR_H_
#define SRC_CANMESSAGEGENERICCONSTRUCTOR_H_

#include "CanMessageFormats.h"
#include "GCodes/GCodeResult.h"

class GCodeBuffer;

class CanMessageGenericConstructor
{
public:
	CanMessageGenericConstructor(const ParamDescriptor *p_param);

	// Populate from a GCode message  returning true if successful. If an error occurs, write the message to 'reply' and return false.
	bool PopulateFromCommand(GCodeBuffer& gb, const StringRef& reply);

	// Methods to add parameters
	void AddUint32Param(char c, uint32_t v)		{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::uint32, sizeof(uint32_t)); }
	void AddInt32Param(char c, int32_t v)		{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::int32, sizeof(int32_t)); }
	void AddFloatParam(char c, float v)			{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::float_p, sizeof(float)); }
	void AddUint16Param(char c, uint16_t v)		{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::uint16, sizeof(uint16_t)); }
	void AddInt16Param(char c, int16_t v)		{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::int16, sizeof(int16_t)); }
	void AddUint8Param(char c, uint8_t v)		{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::uint8, sizeof(uint8_t)); }
	void AddInt8Param(char c, int8_t v)			{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::int8, sizeof(int8_t)); }
	void AddCharParam(char c, char v)			{ AddParam(c, (const char*)&v, ParamDescriptor::ParamType::char_p, sizeof(char)); }
	void AddStringParam(char c, const char* v)	{ AddParam(c, v, ParamDescriptor::ParamType::string, strlen(v) + 1); }

	const char *GetErrorMessage() const { return err; }

	GCodeResult SendAndGetResponse(CanMessageType msgType, uint16_t dest, const StringRef& reply);

private:
	void AddParam(char c, const char *v, ParamDescriptor::ParamType expectedType, size_t length);

	// Append a value to the data, returning true if it wouldn't fit
	bool StoreValue(const void *vp, size_t sz);

	// Append a value to the data, returning true if it wouldn't fit
	template<class T> bool StoreValue(const T& val) { return StoreValue(&val, sizeof(T)); }

	const ParamDescriptor * const paramTable;
	size_t dataLen;
	const char *err;
	CanMessageGeneric msg;
};

#endif /* SRC_CANMESSAGEGENERICCONSTRUCTOR_H_ */
