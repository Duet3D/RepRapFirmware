/*
 * ParseException.h
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEEXCEPTION_H_
#define SRC_GCODES_GCODEEXCEPTION_H_

#include <RepRapFirmware.h>

namespace StackUsage
{
	constexpr uint32_t Throw = 1050;					// how much stack we need to throw an exception
	constexpr uint32_t Margin = 300;					// the margin we allow for calls to non-recursive functions that can throw
}

// This class is mostly used to throw exceptions when processing GCode. It is also used to store error messages that need to be retrieved later.
// Field "message" should always point to a constant string in flash memory, or be null.
// The error message may have a string, int32_t or uint32_t parameter
class GCodeException
{
public:
	GCodeException() noexcept : line(-1), column(-1), message(nullptr) { }
	explicit GCodeException(const char *_ecv_array msg) noexcept: line(-1), column(-1), message(msg) { }

	GCodeException(int lin, int col, const char *_ecv_array msg) noexcept : line(lin), column(col), message(msg)  { }

	GCodeException(int lin, int col, const char *_ecv_array msg, const char *_ecv_array sparam) noexcept : line(lin), column(col), message(msg)
	{
		stringParam.copy(sparam);
	}

	GCodeException(int lin, int col, const char *_ecv_array msg, uint32_t uparam) noexcept : line(lin), column(col), message(msg)
	{
		param.u = uparam;
	}

	GCodeException(int lin, int col, const char *_ecv_array msg, int32_t iparam) noexcept : line(lin), column(col), message(msg)
	{
		param.i = iparam;
	}

	void GetMessage(const StringRef& reply, const GCodeBuffer *null gb) const noexcept;

	bool IsNull() const noexcept { return message == nullptr; }

private:
	int line;
	int column;
	const char *_ecv_array _ecv_null message;
	union
	{
		int32_t i;
		uint32_t u;
	} param;
	String<StringLength50> stringParam;
};

#endif /* SRC_GCODES_GCODEEXCEPTION_H_ */
