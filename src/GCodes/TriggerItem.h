/*
 * Trigger.h
 *
 *  Created on: 6 Jun 2019
 *      Author: David
 */

#ifndef SRC_GCODES_TRIGGERITEM_H_
#define SRC_GCODES_TRIGGERITEM_H_

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <Platform/StringHandle.h>

class TriggerItem
{
public:
	TriggerItem() noexcept;

	void Init() noexcept;

	// Return true if this trigger is unused, i.e. it doesn't watch any pins
	bool IsUnused() const noexcept;

	// Check whether this trigger is active and update the input states
	bool Check(unsigned int number) noexcept;

	// Handle M581 for this trigger
	GCodeResult Configure(unsigned int number, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Handle M582 for this trigger
	bool CheckLevel(unsigned int number) noexcept;

private:
	bool EvaluateExpression() THROWS(GCodeException);
	static void AppendInputNames(AxesBitmap endstops, InputPortsBitmap inputs, const StringRef& reply) noexcept;

	AxesBitmap highLevelEndstops, lowLevelEndstops, endstopStates;
	InputPortsBitmap highLevelInputs, lowLevelInputs, inputStates;
	AutoStringHandle expr;					// the expression to trigger on, or null if we are triggering on inputs/endstops
	int8_t condition;						// the condition specified by the R parameter
	bool exprResult;						// the value of the expression when we last evaluated it
};

#endif /* SRC_GCODES_TRIGGERITEM_H_ */
