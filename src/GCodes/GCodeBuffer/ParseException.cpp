/*
 * ParseException.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include "ParseException.h"

#include <General/StringRef.h>
#include "GCodeBuffer.h"

// Construct the error message. This will be prefixed with "Error: " when it is returned to the user.
void ParseException::GetMessage(const StringRef &reply, const GCodeBuffer& gb) const
{
	reply.copy((gb.IsDoingFileMacro()) ? "in file macro" : "in GCode file");
	if (line >= 0 && column >= 0)
	{
		reply.catf(", line %d column %d", line, column + 1);
	}
	reply.cat(": ");
	reply.catf(message, param.u);
}

// End
