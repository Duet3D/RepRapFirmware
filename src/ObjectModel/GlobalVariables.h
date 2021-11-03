/*
 * GlobalVariables.h
 *
 *  Created on: 31 Mar 2021
 *      Author: David
 */

#ifndef SRC_OBJECTMODEL_GLOBALVARIABLES_H_
#define SRC_OBJECTMODEL_GLOBALVARIABLES_H_

#include "ObjectModel.h"
#include "Variable.h"
#include <RTOSIface/RTOSIface.h>

class GlobalVariables INHERIT_OBJECT_MODEL
{
public:
	GlobalVariables() noexcept { }

	ReadLockedPointer<const VariableSet> GetForReading() noexcept;
	WriteLockedPointer<VariableSet> GetForWriting() noexcept;

	VariableSet& GetVariables() noexcept { return vars; }

protected:
	DECLARE_OBJECT_MODEL

	// Construct a JSON representation of those parts of the object model requested by the user
	// This overrides the standard definition because the variable names are not fixed
	void ReportAsJson(OutputBuffer *buf, ObjectExplorationContext& context, const ObjectModelClassDescriptor * null classDescriptor, uint8_t tableNumber, const char *_ecv_array filter) const noexcept override
			THROWS(GCodeException);

private:
	VariableSet vars;
	mutable ReadWriteLock lock;
};

#endif /* SRC_OBJECTMODEL_GLOBALVARIABLES_H_ */
