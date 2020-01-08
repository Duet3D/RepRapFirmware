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
	GCodeException(int lin, int col, const char *msg) : line(lin), column(col), message(msg)  { }

	GCodeException(int lin, int col, const char *msg, const char *sparam) : line(lin), column(col), message(msg)
	{
		param.s = sparam;
	}

	GCodeException(int lin, int col, const char *msg, uint32_t uparam) : line(lin), column(col), message(msg)
	{
		param.u = uparam;
	}

	GCodeException(int lin, int col, const char *msg, int32_t iparam) : line(lin), column(col), message(msg)
	{
		param.i = iparam;
	}

	void GetMessage(const StringRef& reply, const GCodeBuffer& gb) const;

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
