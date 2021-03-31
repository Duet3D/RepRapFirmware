/*
 * GlobalVariables.cpp
 *
 *  Created on: 31 Mar 2021
 *      Author: David
 */

#include "GlobalVariables.h"
#include <Platform/Outputmemory.h>

// This function is not used in this class
const ObjectModelClassDescriptor *GlobalVariables::GetObjectModelClassDescriptor() const noexcept { return nullptr; }

// Construct a JSON representation of those parts of the object model requested by the user
// This overrides the standard definition because the variable names are not fixed
// We ignore any remaining key or flags and just report all the variables
void GlobalVariables::ReportAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, uint8_t tableNumber, const char *filter) const noexcept
		THROWS(GCodeException)
{
	buf->cat('{');
	if (context.IncreaseDepth())
	{
		ReadLocker locker(lock);
		vars.ListAsJson(buf);
	}
	buf->cat('}');
}

ReadLockedPointer<const VariableSet> GlobalVariables::GetForReading() noexcept
{
	ReadLocker locker(lock);
	return ReadLockedPointer<const VariableSet>(locker, &vars);
}

WriteLockedPointer<VariableSet> GlobalVariables::GetForWriting() noexcept
{
	WriteLocker locker(lock);
	return WriteLockedPointer<VariableSet>(locker, &vars);
}

// End
