/*
 * GlobalVariables.cpp
 *
 *  Created on: 31 Mar 2021
 *      Author: David
 */

#include "GlobalVariables.h"
#include <Platform/OutputMemory.h>

// This function is not used in this class
const ObjectModelClassDescriptor *GlobalVariables::GetObjectModelClassDescriptor() const noexcept { return nullptr; }

// Construct a JSON representation of those parts of the object model requested by the user
// This overrides the standard definition because the variable names are not fixed
// We ignore any remaining key or flags and just report all the variables
void GlobalVariables::ReportAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, uint8_t tableNumber, const char *filter) const THROWS(GCodeException)
{
	if (*filter == 0)
	{
		// Report all global variables
		buf->cat('{');
		if (context.IncreaseDepth())
		{
			{
				ReadLocker locker(lock);			// make sure that no other task modifies the list while we are traversing it
				vars.IterateWhile([this, buf, &context, classDescriptor, filter](unsigned int index, const Variable& v) noexcept -> bool
									{
										buf->catf((index != 0) ? ",\"%s\":" : "\"%s\":", v.GetName().Ptr());
										ReportItemAsJson(buf, context, classDescriptor, v.GetValue(), filter);
										return true;
									}
								 );
			}
			context.DecreaseDepth();
		}
		buf->cat('}');
	}
	else
	{
		// Report a specific global variable, or part of one
		const char *pos = GetNextElement(filter);				// find the end of the variable name
		const Variable *const var = vars.Lookup(filter, pos - filter, false);
		if (var == nullptr)
		{
			buf->cat("null");
		}
		else if (context.IncreaseDepth())
		{
			ReportItemAsJson(buf, context, nullptr, var->GetValue(), pos);
		}
		else
		{
			buf->cat("{}");
		}
	}
}

ReadLockedPointer<const VariableSet> GlobalVariables::GetForReading() noexcept
{
	return ReadLockedPointer<const VariableSet>(lock, &vars);
}

WriteLockedPointer<VariableSet> GlobalVariables::GetForWriting() noexcept
{
	return WriteLockedPointer<VariableSet>(lock, &vars);
}

// End
