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

	if(driveCount > 0)
	{
		if(driveCount > DRIVES - AXES)
		{
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Tool creation: attempt to use more drives than there are in the RepRap...");
			driveCount = 0;
			heaterCount = 0;
			return;
		}
		drives = new int[driveCount];
		for(int8_t drive = 0; drive < driveCount; drive++)
		{
			drives[drive] = d[drive];
		}
	}

	if(heaterCount > 0)
	{
		if(heaterCount > HEATERS)
		{
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Tool creation: attempt to use more heaters than there are in the RepRap...");
			driveCount = 0;
			heaterCount = 0;
			return;
		}
		heaters = new int[heaterCount];
		activeTemperatures = new float[heaterCount];
		standbyTemperatures = new float[heaterCount];
		for(int8_t heater = 0; heater < heaterCount; heater++)
		{
			heaters[heater] = h[heater];
			activeTemperatures[heater] = ABS_ZERO;
			standbyTemperatures[heater] = ABS_ZERO;
		}
	}
}

float Tool::MaxFeedrate()
{
	if(driveCount <= 0)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to get maximum feedrate for a tool with no drives.\n");
		return 1.0;
	}
	float result = 0.0;
	for(int8_t d = 0; d < driveCount; d++)
	{
		float mf = reprap.GetPlatform()->MaxFeedrate(drives[d] + AXES);
		if(mf > result)
		{
			result = mf;
		}
	}
	return result;
}

float Tool::InstantDv()
{
	if(driveCount <= 0)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Attempt to get InstantDv for a tool with no drives.\n");
		return 1.0;
	}
	float result = FLT_MAX;
	for(int8_t d = 0; d < driveCount; d++)
	{
		float idv = reprap.GetPlatform()->InstantDv(drives[d] + AXES);
		if(idv < result)
		{
			result = idv;
		}
	}
	return result;
}

// Add a tool to the end of the linked list.
// (We must already be in it.)

void Tool::AddTool(Tool* t)
{
	Tool* last = this;
	Tool* n = next;
	while(n != NULL)
	{
		last = n;
		n = n->Next();
	}
	t->next = NULL; // Defensive...
	last->next = t;
}

void Tool::Activate(Tool* currentlyActive)
{
	if(active)
		return;
	if(currentlyActive != NULL && currentlyActive != this)
	{
		currentlyActive->Standby();
	}
	for(int8_t heater = 0; heater < heaterCount; heater++)
	{
		reprap.GetHeat()->SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
		reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
		reprap.GetHeat()->Activate(heaters[heater]);
	}
	active = true;
}

void Tool::Standby()
{
	if(!active)
		return;
	for(int8_t heater = 0; heater < heaterCount; heater++)
	{
		reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
		reprap.GetHeat()->Standby(heaters[heater]);
	}
	active = false;
}

void Tool::SetVariables(float* standby, float* active)
{
	for(int8_t heater = 0; heater < heaterCount; heater++)
	{
		activeTemperatures[heater] = active[heater];
		standbyTemperatures[heater] = standby[heater];
		reprap.GetHeat()->SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
		reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
	}
}

void Tool::GetVariables(float* standby, float* active)
{
	for(int8_t heater = 0; heater < heaterCount; heater++)
	{
		active[heater] = activeTemperatures[heater];
		standby[heater] = standbyTemperatures[heater];
	}
}

// Update the number of active drives and extruders in use to reflect what this tool uses
void Tool::UpdateExtrudersAndHeaters(uint16_t &numExtruders, uint16_t &numHeaters)
{
	for(int8_t drive = 0; drive < driveCount; drive++)
	{
		if (drives[drive] >= numExtruders)
		{
			numExtruders = drives[drive] + 1;
		}
	}

	for(int8_t heater = 0; heater < heaterCount; heater++)
	{
		if (heaters[heater] >= numHeaters)
		{
			numHeaters = heaters[heater] + 1;
		}
	}
}

