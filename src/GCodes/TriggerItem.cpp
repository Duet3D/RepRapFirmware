/*
 * Trigger.cpp
 *
 *  Created on: 6 Jun 2019
 *      Author: David
 */

#include "TriggerItem.h"
#include <Platform/RepRap.h>
#include "GCodes.h"
#include <PrintMonitor/PrintMonitor.h>
#include "GCodeBuffer/GCodeBuffer.h"
#include "GCodeBuffer/ExpressionParser.h"

TriggerItem::TriggerItem() noexcept : condition(0)
{
}

// Initialise the trigger
void TriggerItem::Init() noexcept
{
	highLevelEndstops.Clear();
	lowLevelEndstops.Clear();
	highLevelInputs.Clear();
	lowLevelInputs.Clear();
	expr.Delete();
	condition = -1;
}

// Return true if this trigger is unused, i.e. it doesn't watch any pins
bool TriggerItem::IsUnused() const noexcept
{
	return (highLevelEndstops | lowLevelEndstops).IsEmpty() && (highLevelInputs | lowLevelInputs).IsEmpty() && expr.IsNull();
}

// Check whether this trigger is active and update the input states. This is called in a polling loop, so it needs to be fast.
bool TriggerItem::Check(unsigned int number) noexcept
{
	if (condition < 0) { return false; }			// don't check if the trigger is disabled

	bool triggered = false;

	if (expr.IsNull())
	{
		Platform& platform = reprap.GetPlatform();

		// Check the endstops
		const AxesBitmap endstopsMonitored = highLevelEndstops | lowLevelEndstops;
		if (endstopsMonitored.IsNonEmpty())
		{
			EndstopsManager& endstops = platform.GetEndstops();
			endstopsMonitored.Iterate([this, &endstops, &triggered](unsigned int axis, unsigned int) noexcept
										{
											const bool stopped = endstops.Stopped(axis);
											if (stopped != endstopStates.IsBitSet(axis))
											{
												if (stopped)
												{
													endstopStates.SetBit(axis);
													if (highLevelEndstops.IsBitSet(axis))
													{
														triggered = true;
													}
												}
												else
												{
													endstopStates.ClearBit(axis);
													if (lowLevelEndstops.IsBitSet(axis))
													{
														triggered = true;
													}
												}
											}
										}
									);
		}

		const InputPortsBitmap portsMonitored = highLevelInputs | lowLevelInputs;
		if (portsMonitored.IsNonEmpty())
		{
			portsMonitored.Iterate([this, &platform, &triggered](unsigned int inPort, unsigned int) noexcept
									{
										const bool isActive = reprap.GetPlatform().GetGpInPort(inPort).GetState();
										if (isActive != inputStates.IsBitSet(inPort))
										{
											if (isActive)
											{
												inputStates.SetBit(inPort);
												if (highLevelInputs.IsBitSet(inPort))
												{
													triggered = true;
												}
											}
											else
											{
												inputStates.ClearBit(inPort);
												if (lowLevelInputs.IsBitSet(inPort))
												{
													triggered = true;
												}
											}

										}
									}
								);
		}
	}
	else
	{
		// We have an expression to evaluate
		try
		{
			const bool oldVal = exprResult;
			exprResult = EvaluateExpression();
			triggered = exprResult && !oldVal;
		}
		catch (const GCodeException& e)
		{
			condition = -1;
			String<StringLength256> errorMessage;
			e.GetMessage(errorMessage.GetRef(), nullptr);
			errorMessage.catf("\nTrigger %u disabled\n", number);
			reprap.GetPlatform().Message(ErrorMessage, errorMessage.c_str());
		}
	}

	return triggered &&
			(   condition == 0
			 || (2 - condition == (int)reprap.GetPrintMonitor().IsPrinting())		// condition == 1 && IsPrinting || condition == 2 && !IsPrinting
			);
}

// Handle M581 and M581.1 for this trigger. We have already checked that gb.GetCommandFraction() returns <= 1.
GCodeResult TriggerItem::Configure(unsigned int number, GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	// We allow the P-1 parameter to be used with both M581 and M581.1
	bool seen = gb.Seen('P');
	if (seen)
	{
		// We need a try..catch block here so that if we pass an array of unsigned or a string we do not abort when trying to read a single integer
		try
		{
			if (gb.GetIValue() == -1)
			{
				Init();						// P-1 deletes the trigger
				condition = -1;
				return GCodeResult::ok;
			}
		} catch (const GCodeException&) { }
	}

	const bool wasUnused = IsUnused();		// save for later

	switch (gb.GetCommandFraction())
	{
	case 1:									// trigger on an expression
		if (gb.Seen('P'))
		{
			highLevelInputs.Clear();
			lowLevelInputs.Clear();
			highLevelEndstops.Clear();
			lowLevelEndstops.Clear();
			String<StringLength256> conditionString;
			gb.GetQuotedString(conditionString.GetRef(), false);		// may throw
			expr.Assign(conditionString.c_str());
		}
		break;

	default:								// trigger on inputs and/or endstops
		{
			const int sParam = (gb.Seen('S')) ? gb.GetIValue() : 1;			// S is ignored if there is no P or axis letter parameter, so don't set 'seen'

			// See if there are inputs to trigger on
			if (gb.Seen('P'))
			{
				expr.Delete();
				uint32_t inputNumbers[MaxGpInPorts];
				size_t numValues = MaxGpInPorts;
				gb.GetUnsignedArray(inputNumbers, numValues, false);
				const InputPortsBitmap portsToWaitFor = InputPortsBitmap::MakeFromArray(inputNumbers, numValues);
				if (sParam < 0)
				{
					highLevelInputs &= ~portsToWaitFor;
					lowLevelInputs &= ~portsToWaitFor;
				}
				else
				{
					((sParam >= 1) ? highLevelInputs : lowLevelInputs) |= portsToWaitFor;
				}
			}

			// See if there are endstops to trigger on
			{
				AxesBitmap endstopsToWaitFor;
				for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
				{
					if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
					{
						seen = true;
						expr.Delete();
						endstopsToWaitFor.SetBit(axis);
					}
				}
				if (endstopsToWaitFor.IsNonEmpty())
				{
					if (sParam < 0)
					{
						highLevelEndstops &= ~endstopsToWaitFor;
						lowLevelEndstops &= ~endstopsToWaitFor;
					}
					else
					{
						((sParam >= 1) ? highLevelEndstops : lowLevelEndstops) |= endstopsToWaitFor;
					}
				}
			}
		}
	}

	if (!IsUnused())
	{
		if (gb.Seen('R'))
		{
			condition = gb.GetIValue();
			seen = true;
		}
		else if (seen && wasUnused)
		{
			condition = 0;										// this is a new trigger, so set no enable condition
		}
	}

	if (seen)
	{
		// If trigger inputs or the enable condition have been changed, determine the initial state
		if (expr.IsNull())
		{

			inputStates.Clear();
			(void)Check(number);								// set up initial input states
		}
		else
		{
			// Get the initial value of the expression
			try
			{
				exprResult = EvaluateExpression();				// may throw
			}
			catch (const GCodeException&)
			{
				Init();											// clear the trigger
				throw;											// report the error
			}
		}
	}
	else
	{
		reply.printf("Trigger %u ", number);
		if (IsUnused())
		{
			reply.cat("is not configured");
		}
		else
		{
			if (condition < 0)
			{
				reply.cat("if enabled would fire");
			}
			else if (condition == 1)
			{
				reply.cat("fires only when printing");
			}
			else if (condition == 2)
			{
				reply.cat("fires only when not printing");
			}
			else
			{
				reply.cat("fires");
			}
			if (expr.IsNull())
			{
				reply.cat(" on a");
				const bool hasHighLevel = !highLevelEndstops.IsEmpty() || !highLevelInputs.IsEmpty();
				if (hasHighLevel)
				{
					reply.cat(" rising edge of endstops/inputs");
					AppendInputNames(highLevelEndstops, highLevelInputs, reply);
				}
				const bool hasLowLevel = !lowLevelEndstops.IsEmpty() || !lowLevelInputs.IsEmpty();
				if (hasLowLevel)
				{
					if (hasHighLevel)
					{
						reply.cat(" or a");
					}
					reply.cat(" falling edge of endstops/inputs");
					AppendInputNames(lowLevelEndstops, lowLevelInputs, reply);
				}
			}
			else
			{
				reply.cat((condition > 0) ? " and" : " when");
				const auto ptr = expr.Get();
				reply.catf(" expression {%s} becomes true", ptr.Ptr());
			}
		}
	}
	return GCodeResult::ok;
}

// Handle M582 for this trigger
bool TriggerItem::CheckLevel(unsigned int number) noexcept
{
	endstopStates = lowLevelEndstops;
	inputStates = lowLevelInputs;
	exprResult = false;
	return Check(number);
}

void TriggerItem::AppendInputNames(AxesBitmap endstops, InputPortsBitmap inputs, const StringRef &reply) noexcept
{
	if (endstops.IsEmpty() && inputs.IsEmpty())
	{
		reply.cat(" (none)");
	}
	else
	{
		const char *_ecv_array const axisLetters = reprap.GetGCodes().GetAxisLetters();
		endstops.Iterate([axisLetters, &reply](unsigned int axis, unsigned int) noexcept { reply.catf(" %c", axisLetters[axis]); } );
		inputs.Iterate([&reply](unsigned int port, unsigned int) noexcept { reply.catf(" %d", port); } );
	}
}

bool TriggerItem::EvaluateExpression() THROWS(GCodeException)
{
	const auto ptr = expr.Get();
	ExpressionParser parser(nullptr, ptr.Ptr());
	const bool ret = parser.ParseBoolean();
	parser.CheckForExtraCharacters();
	return ret;
}

// End
