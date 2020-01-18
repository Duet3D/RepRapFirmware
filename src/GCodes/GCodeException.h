/*
 * ParseException.h
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEEXCEPTION_H_
#define SRC_GCODES_GCODEEXCEPTION_H_

#include <cstdint>

class StringRef;
class GCodeBuffer;

class GCodeException
{
public:
	GCodeException(int lin, int col, const char *msg) noexcept : line(lin), column(col), message(msg)  { }

	GCodeException(int lin, int col, const char *msg, const char *sparam) noexcept : line(lin), column(col), message(msg)
	{
		param.s = sparam;
	}

	GCodeException(int lin, int col, const char *msg, uint32_t uparam) noexcept : line(lin), column(col), message(msg)
	{
		param.u = uparam;
	}

	GCodeException(int lin, int col, const char *msg, int32_t iparam) noexcept : line(lin), column(col), message(msg)
	{
		param.i = iparam;
	}

	void GetMessage(const StringRef& reply, const GCodeBuffer& gb) const noexcept;

private:
	int line;
	int column;
	const char *message;
	union
	{
		int32_t i;
		uint32_t u;
		const char *s;
	} param;
};

#endif /* SRC_GCODES_GCODEEXCEPTION_H_ */
