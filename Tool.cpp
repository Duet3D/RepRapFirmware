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

#include "RepRapFirmware.h"

Tool::Tool(int toolNumber, long d[], int dCount, long h[], int hCount)
{
	myNumber = toolNumber;
	next = NULL;
	active = false;
	driveCount = dCount;
	heaterCount = hCount;
	heaterFault = false;
	mixing = false;
	displayColdExtrudeWarning = false;

	for(size_t axis = 0; axis < AXES; axis++)
	{
		offset[axis] = 0.0;
	}

	if (driveCount > 0)
	{
		if (driveCount > DRIVES - AXES)
		{
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Tool creation: attempt to use more drives than there are in the RepRap...");
			driveCount = 0;
			heaterCount = 0;
			return;
		}
		drives = new int[driveCount];
		mix = new float[driveCount];
		float r = 1.0/(float)driveCount;

		for (int drive = 0; drive < driveCount; drive++)
		{
			drives[drive] = d[drive];
			mix[drive] = r;
		}
	}

	if (heaterCount > 0)
	{
		if (heaterCount > HEATERS)
		{
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Tool creation: attempt to use more heaters than there are in the RepRap...");
			driveCount = 0;
			heaterCount = 0;
			return;
		}
		heaters = new int[heaterCount];
		activeTemperatures = new float[heaterCount];
		standbyTemperatures = new float[heaterCount];

		for(int heater = 0; heater < heaterCount; heater++)
		{
			heaters[heater] = h[heater];
			activeTemperatures[heater] = ABS_ZERO;
			standbyTemperatures[heater] = ABS_ZERO;
		}
	}
}

void Tool::Print(StringRef& reply)
{
	reply.printf("Tool %d - drives: ", myNumber);
	char comma = ',';
	for (int drive = 0; drive < driveCount; drive++)
	{
		if (drive >= driveCount - 1)
		{
			comma = ';';
		}
		reply.catf("%d%c", drives[drive], comma);
	}

	reply.cat("heaters (active/standby temps): ");
	comma = ',';
	for (int heater = 0; heater < heaterCount; heater++)
	{
			if (heater >= heaterCount - 1)
			{
				comma = ';';
			}
			reply.catf("%d (%.1f/%.1f)%c", heaters[heater],
					activeTemperatures[heater], standbyTemperatures[heater], comma);
	}

	reply.catf(" status: %s", active ? "selected" : "standby");
}

float Tool::MaxFeedrate() const
{
	if (driveCount <= 0)
	{
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Attempt to get maximum feedrate for a tool with no drives.\n");
		return 1.0;
	}
	float result = 0.0;
	for (int d = 0; d < driveCount; d++)
	{
		float mf = reprap.GetPlatform()->MaxFeedrate(drives[d] + AXES);
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
		reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Attempt to get InstantDv for a tool with no drives.\n");
		return 1.0;
	}
	float result = FLT_MAX;
	for (int d = 0; d < driveCount; d++)
	{
		float idv = reprap.GetPlatform()->ActualInstantDv(drives[d] + AXES);
		if (idv < result)
		{
			result = idv;
		}
	}
	return result;
}

// Add a tool to the end of the linked list.
// (We must already be in it.)

void Tool::AddTool(Tool* tool)
{
	Tool* t = this;
	Tool* last = this;		// initialised to suppress spurious compiler warning
	while(t != NULL)
	{
		if(t->Number() == tool->Number())
		{
			reprap.GetPlatform()->Message(BOTH_ERROR_MESSAGE, "Add tool: tool number already in use.\n");
			return;
		}
		last = t;
		t = t->Next();
	}
	tool->next = NULL; // Defensive...
	last->next = tool;
}

// There is a temperature fault on a heater.
// Disable all tools using that heater.
// This function must be called for the first
// entry in the linked list.

void Tool::FlagTemperatureFault(int8_t heater)
{
	Tool* n = this;
	while(n != NULL)
	{
		n->SetTemperatureFault(heater);
		n = n->Next();
	}
}

void Tool::ClearTemperatureFault(int8_t heater)
{
	Tool* n = this;
	while(n != NULL)
	{
		n->ResetTemperatureFault(heater);
		n = n->Next();
	}
}

void Tool::SetTemperatureFault(int8_t dudHeater)
{
	for (int heater = 0; heater < heaterCount; heater++)
	{
		if(dudHeater == heaters[heater])
		{
			heaterFault = true;
			return;
		}
	}
}

void Tool::ResetTemperatureFault(int8_t wasDudHeater)
{
	for (int heater = 0; heater < heaterCount; heater++)
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
	for (int heater = 0; heater < heaterCount; heater++)
	{
		const float temperature = reprap.GetHeat()->GetTemperature(heaters[heater]);
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
		if (currentlyActive != NULL && currentlyActive != this)
		{
			currentlyActive->Standby();
		}
		for (int heater = 0; heater < heaterCount; heater++)
		{
			reprap.GetHeat()->SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
			reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			reprap.GetHeat()->Activate(heaters[heater]);
		}
		active = true;
	}
}

void Tool::Standby()
{
	if (active)
	{
		for (int heater = 0; heater < heaterCount; heater++)
		{
			reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			reprap.GetHeat()->Standby(heaters[heater]);
		}
		active = false;
	}
}

void Tool::SetVariables(const float* standby, const float* active)
{
	for (int heater = 0; heater < heaterCount; heater++)
	{
		if (active[heater] < NEARLY_ABS_ZERO && standby[heater] < NEARLY_ABS_ZERO)
		{
			// Temperatures close to ABS_ZERO turn off all associated heaters
			reprap.GetHeat()->SwitchOff(heaters[heater]);
		}
		else
		{
			if (active[heater] < BAD_HIGH_TEMPERATURE)
			{
				activeTemperatures[heater] = active[heater];
				reprap.GetHeat()->SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
			}
			if (standby[heater] < BAD_HIGH_TEMPERATURE)
			{
				standbyTemperatures[heater] = standby[heater];
				reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			}
		}
	}
}

void Tool::GetVariables(float* standby, float* active) const
{
	for (int heater = 0; heater < heaterCount; heater++)
	{
		active[heater] = activeTemperatures[heater];
		standby[heater] = standbyTemperatures[heater];
	}
}

// May be called from ISR
bool Tool::ToolCanDrive(bool extrude)
{
	if (heaterFault)
		return false;

	if (reprap.ColdExtrude() || AllHeatersAtHighTemperature(extrude))
		return true;

	displayColdExtrudeWarning = true;
	return false;
}

// Update the number of active drives and extruders in use to reflect what this tool uses
void Tool::UpdateExtruderAndHeaterCount(uint16_t &numExtruders, uint16_t &numHeaters) const
{
	for (int drive = 0; drive < driveCount; drive++)
	{
		if (drives[drive] >= numExtruders)
		{
			numExtruders = drives[drive] + 1;
		}
	}

	for (int heater = 0; heater < heaterCount; heater++)
	{
		if (heaters[heater] != HOT_BED && heaters[heater] >= numHeaters)
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

// End
