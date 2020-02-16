/*
 * ParseException.cpp
 *
 *  Created on: 21 Dec 2019
 *      Author: David
 */

#include "GCodeException.h"

#include <General/StringRef.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

// Construct the error message. This will be prefixed with "Error: " when it is returned to the user.
void GCodeException::GetMessage(const StringRef &reply, const GCodeBuffer *gb) const noexcept
{
	const char *context = (gb != nullptr && gb->IsDoingFileMacro()) ? "in file macro"
							: (gb != nullptr && gb->IsDoingFile()) ? "in GCode file"
								: "while executing command";
	reply.copy(context);
	if (line >= 0 && column >= 0 && gb != nullptr && gb->IsDoingFile())
	{
		reply.catf(", line %d column %d", line, column + 1);
	}
	reply.cat(": ");
	reply.catf(message, param.u);
}

// End
