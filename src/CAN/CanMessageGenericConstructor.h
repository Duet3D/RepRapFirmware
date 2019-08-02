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

class GCodeBuffer;

class CanMessageGenericConstructor
{
public:
	CanMessageGenericConstructor(const ParamDescriptor *p_param);

	// Populate from a GCode message  returning true if successful. If an error occurs, write the message to 'reply' and return false.
	bool PopulateFromCommand(GCodeBuffer& gb, const StringRef& reply);

	// Methods to add parameters
	void AddUParam(char c, uint32_t v);
	void AddIParam(char c, int32_t v);
	void AddFParam(char c, float v);
	void AddCharParam(char c, char v);
	void AddStringParam(char c, const char* v);

	const char *GetErrorMessage() const { return err; }

	GCodeResult SendAndGetResponse(CanMessageType msgType, CanAddress dest, const StringRef& reply);

private:
	// Append a value to the data, returning true if it wouldn't fit
	bool StoreValue(const void *vp, size_t sz);

	// Append a value to the data, returning true if it wouldn't fit
	template<class T> bool StoreValue(const T& val) { return StoreValue(&val, sizeof(T)); }

	// Insert a value in the data, returning true if it wouldn't fit
	bool InsertValue(const void *vp, size_t sz, size_t pos);

	const ParamDescriptor * const paramTable;
	size_t dataLen;
	const char *err;
	CanMessageGeneric msg;
};

#endif

#endif /* SRC_CANMESSAGEGENERICCONSTRUCTOR_H_ */
