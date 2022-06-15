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
										ReportItemAsJsonFull(buf, context, classDescriptor, v.GetValue(), filter);
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
		const Variable *const var = vars.Lookup(filter, pos - filter);
		if (var == nullptr)
		{
			buf->cat("null");
		}
		else if (context.IncreaseDepth())
		{
#if 1
			// Currently a variable cannot hold an object or an object model array, so it doesn't matter what object we pass as the this-parameter to ReportItemAsJsonFull
			ReportItemAsJsonFull(buf, context, nullptr, var->GetValue(), pos);
#else
			// The following code doesn't work anyway. For type ObjectModel_tc it reports the requested member name but not the value.
			const ExpressionValue val = var->GetValue();
			switch (val.GetType())
			{
			case TypeCode::ObjectModel_tc:
			case TypeCode::ObjectModelArray:
				val.omVal->ReportItemAsJsonFull(buf, context, nullptr, val, pos);
				break;

			default:
				ReportItemAsJsonFull(buf, context, nullptr, val, pos);
				break;
			}
#endif
		}
		else
		{
			buf->cat("{}");
		}
	}
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
