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
#include "Filament.h"

#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "Platform.h"
#include "RepRap.h"

Tool * Tool::freelist = nullptr;

// Create a new tool and return a pointer to it. If an error occurs, put an error message in 'reply' and return nullptr.
/*static*/ Tool *Tool::Create(int toolNumber, const char *name, long d[], size_t dCount, long h[], size_t hCount, AxesBitmap xMap, AxesBitmap yMap, FansBitmap fanMap, StringRef& reply)
{
	const size_t numExtruders = reprap.GetGCodes().GetNumExtruders();
	if (dCount > ARRAY_SIZE(Tool::drives))
	{
		reply.copy("Tool creation: too many drives");
		return nullptr;
	}

	if (hCount > ARRAY_SIZE(Tool::heaters))
	{
		reply.copy("Tool creation: too many heaters");
		return nullptr;
	}

	// Validate the heater and extruder numbers
	for (size_t i = 0; i < dCount; ++i)
	{
		if (d[i] < 0 || d[i] >= (int)numExtruders)
		{
			reply.copy("Tool creation: bad drive number");
			return nullptr;
		}
	}
	for (size_t i = 0; i < hCount; ++i)
	{
		if (h[i] < 0 || h[i] >= (int)Heaters)
		{
			reply.copy("Tool creation: bad heater number");
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

	if (dCount == 1)
	{
		// Create only one Filament instance per extruder drive, and only if this tool is assigned to exactly one extruder
		Filament *filament = Filament::GetFilamentByExtruder(d[0]);
		t->filament = (filament == nullptr) ? new Filament(d[0]) : filament;
	}
	else
	{
		// Don't support filament codes for other tools
		t->filament = nullptr;
	}

	t->myNumber = toolNumber;
	SafeStrncpy(t->name, name, ToolNameLength);
	t->next = nullptr;
	t->state = ToolState::off;
	t->driveCount = dCount;
	t->heaterCount = hCount;
	t->xMapping = xMap;
	t->yMapping = yMap;
	t->fanMapping = fanMap;
	t->heaterFault = false;
	t->displayColdExtrudeWarning = false;

	for (size_t axis = 0; axis < MaxAxes; axis++)
	{
		t->offset[axis] = 0.0;
	}

	for (size_t drive = 0; drive < t->driveCount; drive++)
	{
		t->drives[drive] = d[drive];
		t->mix[drive] = (drive == 0) ? 1.0 : 0.0;		// initial mix ratio is 1:1:0
	}

	for (size_t heater = 0; heater < t->heaterCount; heater++)
	{
		t->heaters[heater] = h[heater];
		t->activeTemperatures[heater] = ABS_ZERO;
		t->standbyTemperatures[heater] = ABS_ZERO;
	}

	if (t->filament != nullptr)
	{
		t->filament->LoadAssignment();
	}

	return t;
}

/*static*/ void Tool::Delete(Tool *t)
{
	if (t != nullptr)
	{
		t->filament = nullptr;
		t->next = freelist;
		freelist = t;
	}
}

void Tool::Print(StringRef& reply)
{
	reply.printf("Tool %d - ", myNumber);
	if (!StringEquals(name, ""))
	{
		reply.catf("name: %s; ", name);
	}

	reply.cat("drives:");
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

	reply.cat("; ymap:");
	sep = ' ';
	for (size_t yi = 0; yi < MaxAxes; ++yi)
	{
		if ((yMapping & (1u << yi)) != 0)
		{
			reply.catf("%c%c", sep, GCodes::axisLetters[yi]);
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

	reply.catf("; status: %s", (state == ToolState::active) ? "selected" : (state == ToolState::standby) ? "standby" : "off");
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
	if (state != ToolState::active)
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
		state = ToolState::active;
	}
}

void Tool::Standby()
{
	if (state != ToolState::standby)
	{
		for (size_t heater = 0; heater < heaterCount; heater++)
		{
			reprap.GetHeat().SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			reprap.GetHeat().Standby(heaters[heater], this);
		}
		state = ToolState::standby;
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
				const Tool *lastStandbyTool = reprap.GetHeat().GetLastStandbyTool(heaters[heater]);
				if (currentTool == nullptr || currentTool == this || lastStandbyTool == nullptr || lastStandbyTool  == this)
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

#ifdef DUET_NG

// Write the tool's settings to file returning true if successful
bool Tool::WriteSettings(FileStore *f) const
{
	char bufSpace[50];
	StringRef buf(bufSpace, ARRAY_SIZE(bufSpace));

	// Set up active and standby heater temperatures
	bool ok = true;
	if (heaterCount != 0)
	{
		buf.copy("G10 ");
		char c = 'S';
		for (size_t i = 0; i < heaterCount; ++i)
		{
			buf.catf("%c%d", c, (int)activeTemperatures[i]);
			c = ':';
		}
		buf.cat(' ');
		c = 'R';
		for (size_t i = 0; i < heaterCount; ++i)
		{
			buf.catf("%c%d", c, (int)standbyTemperatures[i]);
			c = ':';
		}
		buf.cat('\n');
		ok = f->Write(buf.Pointer());
	}

	if (ok && state != ToolState::off)
	{
		// Select tool
		buf.printf("T%d P0\n", myNumber);
		ok = f->Write(buf.Pointer());
	}

	return ok;
}

#endif

// End
