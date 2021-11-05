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

class GCodeException
{
public:
	explicit GCodeException(const char *msg) noexcept: line(-1), column(-1), message(msg), haveStringParam(false) { }

	GCodeException(int lin, int col, const char *msg) noexcept : line(lin), column(col), message(msg), haveStringParam(false)  { }

	GCodeException(int lin, int col, const char *msg, const char *sparam) noexcept : line(lin), column(col), message(msg), haveStringParam(true)
	{
		stringParam.copy(sparam);
	}

	GCodeException(int lin, int col, const char *msg, uint32_t uparam) noexcept : line(lin), column(col), message(msg), haveStringParam(false)
	{
		param.u = uparam;
	}

	GCodeException(int lin, int col, const char *msg, int32_t iparam) noexcept : line(lin), column(col), message(msg), haveStringParam(false)
	{
		param.i = iparam;
	}

	void GetMessage(const StringRef& reply, const GCodeBuffer *gb) const noexcept;

private:
	int line;
	int column;
	const char *message;
	union
	{
		int32_t i;
		uint32_t u;
	} param;
	bool haveStringParam;
	String<StringLength50> stringParam;
};

#endif /* SRC_GCODES_GCODEEXCEPTION_H_ */
