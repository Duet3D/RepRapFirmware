/*
 * ParseException.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include "ParseException.h"

#include <General/StringRef.h>

void ParseException::GetMessage(const StringRef &reply) const
{
	reply.copy("Parsing error");
	if (column >= 0)
	{
		reply.catf(" at column %d", column + 1);
	}
	reply.cat(": ");
	reply.printf(message, param.u);
}

// End
