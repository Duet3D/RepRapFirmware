/****************************************************************************************************

 RepRapFirmware - Tool

 This class implements a tool in the RepRap machine, usually (though not necessarily) an extruder.

 Tools may have zero or more drives associated with them and zero or more heaters.  There are a fixed number
 of tools in a given RepRap, with fixed heaters and drives.  All this is specified on reboot, and cannot
 be altered dynamically.  This restriction may be lifted in the future.  Tool descriptions are stored in
 GCode macros that are loaded on reboot.

 -----------------------------------------------------------------------------------------------------

 Version 0.1

 Created on: Apr 11, 2014

 Adrian Bowyer
 RepRap Professional Ltd
 http://reprappro.com

 Licence: GPL

 ****************************************************************************************************/

#include "Tool.h"

#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Platform.h"
#include "RepRap.h"

Tool * Tool::freelist = nullptr;

/*static*/Tool * Tool::Create(int toolNumber, long d[], size_t dCount, long h[], size_t hCount, uint32_t xMap, uint32_t fanMap)
{
	const size_t numExtruders = reprap.GetGCodes().GetNumExtruders();
	if (dCount > ARRAY_SIZE(Tool::drives))
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: Tool creation: too many drives");
		return nullptr;
	}

	if (hCount > ARRAY_SIZE(Tool::heaters))
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: Tool creation: too many heaters");
		return nullptr;
	}

	// Validate the heater and extruder numbers
	for (size_t i = 0; i < dCount; ++i)
	{
		if (d[i] < 0 || d[i] >= (int)numExtruders)
		{
			reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: Tool creation: bad drive number");
			return nullptr;
		}
	}
	for (size_t i = 0; i < hCount; ++i)
	{
		if (h[i] < 0 || h[i] >= (int)Heaters)
		{
			reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: Tool creation: bad heater number");
			return nullptr;
		}
	}

	Tool *t;
	if (freelist != nullptr)
	{
		t = freelist;
		freelist = t->next;
	}
	else
	{
		t = new Tool;
	}

	t->myNumber = toolNumber;
	t->next = nullptr;
	t->active = false;
	t->driveCount = dCount;
	t->heaterCount = hCount;
	t->xMapping = xMap;
	t->fanMapping = fanMap;
	t->heaterFault = false;
	t->mixing = false;
	t->displayColdExtrudeWarning = false;
	t->virtualExtruderPosition = 0.0;

	for (size_t axis = 0; axis < MaxAxes; axis++)
	{
		t->offset[axis] = 0.0;
	}

	if (t->driveCount > 0)
	{
		const float r = 1.0 / (float) (t->driveCount);
		for (size_t drive = 0; drive < t->driveCount; drive++)
		{
			t->drives[drive] = d[drive];
			t->mix[drive] = r;
		}
	}

	for (size_t heater = 0; heater < t->heaterCount; heater++)
	{
		t->heaters[heater] = h[heater];
		t->activeTemperatures[heater] = ABS_ZERO;
		t->standbyTemperatures[heater] = ABS_ZERO;
	}

	return t;
}

/*static*/void Tool::Delete(Tool *t)
{
	if (t != nullptr)
	{
		t->next = freelist;
		freelist = t;
	}
}

void Tool::Print(StringRef& reply)
{
	reply.printf("Tool %d - drives:", myNumber);
	char sep = ' ';
	for (size_t drive = 0; drive < driveCount; drive++)
	{
		reply.catf("%c%d", sep, drives[drive]);
		sep = ',';
	}

	reply.cat("; heaters (active/standby temps):");
	sep = ' ';
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		reply.catf("%c%d (%.1f/%.1f)", sep, heaters[heater], activeTemperatures[heater], standbyTemperatures[heater]);
		sep = ',';
	}

	reply.cat("; xmap:");
	sep = ' ';
	for (size_t xi = 0; xi < MaxAxes; ++xi)
	{
		if ((xMapping & (1u << xi)) != 0)
		{
			reply.catf("%c%c", sep, GCodes::axisLetters[xi]);
			sep = ',';
		}
	}

	reply.cat("; fans:");
	sep = ' ';
	for (size_t fi = 0; fi < NUM_FANS; ++fi)
	{
		if ((fanMapping & (1u << fi)) != 0)
		{
			reply.catf("%c%u", sep, fi);
			sep = ',';
		}
	}

	reply.catf("; status: %s", active ? "selected" : "standby");
}

float Tool::MaxFeedrate() const
{
	if (driveCount <= 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: Attempt to get maximum feedrate for a tool with no drives.\n");
		return 1.0;
	}
	float result = 0.0;
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t d = 0; d < driveCount; d++)
	{
		const float mf = reprap.GetPlatform().MaxFeedrate(drives[d] + numAxes);
		if (mf > result)
		{
			result = mf;
		}
	}
	return result;
}

float Tool::InstantDv() const
{
	if (driveCount <= 0)
	{
		reprap.GetPlatform().Message(GENERIC_MESSAGE, "Error: Attempt to get InstantDv for a tool with no drives.\n");
		return 1.0;
	}
	float result = FLT_MAX;
	const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t d = 0; d < driveCount; d++)
	{
		const float idv = reprap.GetPlatform().ActualInstantDv(drives[d] + numAxes);
		if (idv < result)
		{
			result = idv;
		}
	}
	return result;
}

// There is a temperature fault on a heater.
// Disable all tools using that heater.
// This function must be called for the first
// entry in the linked list.

void Tool::FlagTemperatureFault(int8_t heater)
{
	Tool* n = this;
	while (n != nullptr)
	{
		n->SetTemperatureFault(heater);
		n = n->Next();
	}
}

void Tool::ClearTemperatureFault(int8_t heater)
{
	Tool* n = this;
	while (n != nullptr)
	{
		n->ResetTemperatureFault(heater);
		n = n->Next();
	}
}

void Tool::SetTemperatureFault(int8_t dudHeater)
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		if (dudHeater == heaters[heater])
		{
			heaterFault = true;
			return;
		}
	}
}

void Tool::ResetTemperatureFault(int8_t wasDudHeater)
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		if (wasDudHeater == heaters[heater])
		{
			heaterFault = false;
			return;
		}
	}
}

bool Tool::AllHeatersAtHighTemperature(bool forExtrusion) const
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		const float temperature = reprap.GetHeat().GetTemperature(heaters[heater]);
		if (temperature < HOT_ENOUGH_TO_RETRACT || (temperature < HOT_ENOUGH_TO_EXTRUDE && forExtrusion))
		{
			return false;
		}
	}
	return true;
}

void Tool::Activate(Tool* currentlyActive)
{
	if (!active)
	{
		if (currentlyActive != nullptr && currentlyActive != this)
		{
			currentlyActive->Standby();
		}
		for (size_t heater = 0; heater < heaterCount; heater++)
		{
			reprap.GetHeat().SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
			reprap.GetHeat().SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			reprap.GetHeat().Activate(heaters[heater]);
		}
		active = true;
	}
}

void Tool::Standby()
{
	if (active)
	{
		for (size_t heater = 0; heater < heaterCount; heater++)
		{
			reprap.GetHeat().SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			reprap.GetHeat().Standby(heaters[heater]);
		}
		active = false;
	}
}

void Tool::SetVariables(const float* standby, const float* active)
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		if (active[heater] < NEARLY_ABS_ZERO && standby[heater] < NEARLY_ABS_ZERO)
		{
			// Temperatures close to ABS_ZERO turn off all associated heaters
			reprap.GetHeat().SwitchOff(heaters[heater]);
		}
		else
		{
			const float temperatureLimit = reprap.GetHeat().GetTemperatureLimit(heaters[heater]);
			const Tool * const currentTool = reprap.GetCurrentTool();
			if (active[heater] < temperatureLimit)
			{
				activeTemperatures[heater] = active[heater];
				if (currentTool == nullptr || currentTool == this)
				{
					reprap.GetHeat().SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
				}
			}
			if (standby[heater] < temperatureLimit)
			{
				standbyTemperatures[heater] = standby[heater];
				if (currentTool == nullptr || currentTool == this)
				{
					reprap.GetHeat().SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
				}
			}
		}
	}
}

void Tool::GetVariables(float* standby, float* active) const
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		active[heater] = activeTemperatures[heater];
		standby[heater] = standbyTemperatures[heater];
	}
}

// May be called from ISR
bool Tool::ToolCanDrive(bool extrude)
{
	if (!heaterFault && AllHeatersAtHighTemperature(extrude))
	{
		return true;
	}

	displayColdExtrudeWarning = true;
	return false;
}

// Update the number of active drives and extruders in use to reflect what this tool uses
void Tool::UpdateExtruderAndHeaterCount(uint16_t &numExtruders, uint16_t &numHeaters) const
{
	for (size_t drive = 0; drive < driveCount; drive++)
	{
		if (drives[drive] >= numExtruders)
		{
			numExtruders = drives[drive] + 1;
		}
	}

	const int8_t bedHeater = reprap.GetHeat().GetBedHeater();
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		if (heaters[heater] != bedHeater && heaters[heater] >= numHeaters)
		{
			numHeaters = heaters[heater] + 1;
		}
	}
}

bool Tool::DisplayColdExtrudeWarning()
{
	bool result = displayColdExtrudeWarning;
	displayColdExtrudeWarning = false;
	return result;
}

void Tool::DefineMix(const float m[])
{
	for(size_t drive = 0; drive < driveCount; drive++)
	{
		mix[drive] = m[drive];
	}
}

// End
