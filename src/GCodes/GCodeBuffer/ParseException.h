/*
 * ParseException.h
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#ifndef SRC_GCODES_PARSEEXCEPTION_H_
#define SRC_GCODES_PARSEEXCEPTION_H_

#include <cstdint>

class StringRef;

class ParseException
{
public:
	ParseException(int col, const char *msg) : column(col), message(msg)  { }

	ParseException(int col, const char *msg, const char *sparam) : column(col), message(msg)
	{
		param.s = sparam;
	}

	ParseException(int col, const char *msg, uint32_t uparam) : column(col), message(msg)
	{
		param.u = uparam;
	}

	ParseException(unsigned int col, const char *msg, int32_t iparam) : column(col), message(msg)
	{
		param.i = iparam;
	}

	void GetMessage(const StringRef& reply) const;

private:
	int column;
	const char *message;
	union
	{
		int32_t i;
		uint32_t u;
		const char *s;
	} param;
};

#endif /* SRC_GCODES_PARSEEXCEPTION_H_ */
