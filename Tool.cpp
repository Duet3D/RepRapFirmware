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

Tool::Tool(int tNum, int d[], int h[])
{
	myNumber = tNum;
	next = NULL;
	active = false;

	for(driveCount = 0; driveCount < DRIVES; driveCount++)
		if(d[driveCount] < 0)
			break;
	if(driveCount > 0)
	{
		drives = new int[driveCount];
		for(int8_t drive = 0; drive < driveCount; drive++)
			drives[drive] = d[drive];
	}

	for(heaterCount = 0; heaterCount < HEATERS; heaterCount++)
		if(h[heaterCount] < 0)
			break;
	if(heaterCount > 0)
	{
		heaters = new int[heaterCount];
		for(int8_t heater = 0; heater < heaterCount; heater++)
			heaters[heater] = h[heater];
	}

	x = 0.0;
	y = 0.0;
	z = 0.0;
}

// Add a tool to the end of the linked list.
// (We must already be in it.)

void Tool::AddTool(Tool* t)
{
	Tool* last = this;
	Tool* n = next;
	while(n)
	{
		last = n;
		n = Next();
	}
	t->next = NULL; // Defensive...
	last->next = t;
}

void Tool::Activate(Tool* currentlyActive)
{
	if(active)
		return;
	if(currentlyActive)
		currentlyActive->Standby();
	for(int8_t heater = 0; heater < heaterCount; heater++)
		reprap.GetHeat()->Activate(heaters[heater]);
	active = true;
}

void Tool::Standby()
{
	if(!active)
		return;
	for(int8_t heater = 0; heater < heaterCount; heater++)
		reprap.GetHeat()->Standby(heaters[heater]);
	active = false;
}

void Tool::SetVariables(float xx, float yy, float zz, float* standbyTemperatures, float* activeTemperatures)
{
	x = xx;
	y = yy;
	z = zz;
	for(int8_t heater = 0; heater < heaterCount; heater++)
	{
		reprap.GetHeat()->SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
		reprap.GetHeat()->SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
	}
}


