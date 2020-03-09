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
	const bool inFile = gb != nullptr && gb->IsDoingFile();
	if (inFile)
	{
		reply.copy((gb->IsDoingFileMacro()) ? "in file macro": "in GCode file");
		if (line >= 0)
		{
			reply.catf(" line %d", line);
			if (column >= 0)
			{
				reply.catf(" column %d", column + 1);
			}
		}
		reply.cat(": ");
	}
	if (gb != nullptr && gb->HasCommandNumber())
	{
		reply.catf("%c%u: ", gb->GetCommandLetter(), gb->GetCommandNumber());
	}
	reply.catf(message, param.u);
}

// End
