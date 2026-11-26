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

// What a trigger does when it fires, set by the M581 K parameter. All but runMacro are hard-wired, so that they do not depend on the file system
enum class TriggerAction : uint8_t { emergencyStop = 0, pause, runMacro, feedHold, feedHoldAndAbort };
constexpr unsigned int NumTriggerActions = 5;

class TriggerItem
{
public:
	TriggerItem() noexcept;

	void Init() noexcept;

	// Return true if this trigger is unused, i.e. it doesn't watch any pins
	bool IsUnused() const noexcept;

	// Return what this trigger does when it fires. Without a K parameter trigger 0 is an emergency stop, trigger 1 pauses and all others run their macro
	TriggerAction GetAction(unsigned int number) const noexcept;

	// Return true if the level of an input, endstop or expression that fires this trigger is active now, as of the last call to Check
	bool IsLevelActive() const noexcept;

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
	int8_t action;							// the action specified by the K parameter, or -1 to use the default for the trigger number
	bool exprResult;						// the value of the expression when we last evaluated it
};

#endif /* SRC_GCODES_TRIGGERITEM_H_ */
