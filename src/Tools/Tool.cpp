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

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Tool, __VA_ARGS__)

constexpr ObjectModelArrayDescriptor Tool::activeTempsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Tool*)self)->heaterCount; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Tool*)self)->activeTemperatures[context.GetLastIndex()], 1); }
};

constexpr ObjectModelArrayDescriptor Tool::standbyTempsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Tool*)self)->heaterCount; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Tool*)self)->standbyTemperatures[context.GetLastIndex()], 1); }
};

constexpr ObjectModelArrayDescriptor Tool::heatersArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Tool*)self)->heaterCount; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue((int32_t)((const Tool*)self)->heaters[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor Tool::extrudersArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Tool*)self)->driveCount; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue((int32_t)((const Tool*)self)->drives[context.GetLastIndex()]); }
};

constexpr ObjectModelArrayDescriptor Tool::mixArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return ((const Tool*)self)->driveCount; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Tool*)self)->mix[context.GetLastIndex()], 2); }
};

constexpr ObjectModelArrayDescriptor Tool::offsetsArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return reprap.GetGCodes().GetVisibleAxes(); },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Tool*)self)->offset[context.GetLastIndex()], 2); }
};

constexpr ObjectModelArrayDescriptor Tool::axesArrayDescriptor =
{
	nullptr,					// no lock needed
	[] (const ObjectModel *self, const ObjectExplorationContext&) noexcept -> size_t { return 2; },
	[] (const ObjectModel *self, ObjectExplorationContext& context) noexcept -> ExpressionValue { return ExpressionValue(((const Tool*)self)->axisMapping[context.GetLastIndex()]); }
};

constexpr ObjectModelTableEntry Tool::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Tool members
	{ "active",		OBJECT_MODEL_FUNC_NOSELF(&activeTempsArrayDescriptor), 			ObjectModelEntryFlags::live },
	{ "axes",		OBJECT_MODEL_FUNC_NOSELF(&axesArrayDescriptor), 				ObjectModelEntryFlags::none },
	{ "extruders",	OBJECT_MODEL_FUNC_NOSELF(&extrudersArrayDescriptor), 			ObjectModelEntryFlags::none },
	{ "fans",		OBJECT_MODEL_FUNC(self->fanMapping), 							ObjectModelEntryFlags::none },
	{ "heaters",	OBJECT_MODEL_FUNC_NOSELF(&heatersArrayDescriptor), 				ObjectModelEntryFlags::none },
	{ "mix",		OBJECT_MODEL_FUNC_NOSELF(&mixArrayDescriptor), 					ObjectModelEntryFlags::none },
	{ "name",		OBJECT_MODEL_FUNC(self->name),						 			ObjectModelEntryFlags::none },
	{ "offsets",	OBJECT_MODEL_FUNC_NOSELF(&offsetsArrayDescriptor), 				ObjectModelEntryFlags::none },
	{ "standby",	OBJECT_MODEL_FUNC_NOSELF(&standbyTempsArrayDescriptor), 		ObjectModelEntryFlags::live },
	{ "state",		OBJECT_MODEL_FUNC(self->state.ToString()), 						ObjectModelEntryFlags::live },
};

constexpr uint8_t Tool::objectModelTableDescriptor[] = { 1, 10 };

DEFINE_GET_OBJECT_MODEL_TABLE(Tool)

#endif

// Create a new tool and return a pointer to it. If an error occurs, put an error message in 'reply' and return nullptr.
/*static*/ Tool *Tool::Create(unsigned int toolNumber, const char *name, int32_t d[], size_t dCount, int32_t h[], size_t hCount, AxesBitmap xMap, AxesBitmap yMap, FansBitmap fanMap, int filamentDrive, const StringRef& reply) noexcept
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
		if (h[i] < 0 || h[i] >= (int)MaxHeaters)
		{
			reply.copy("Tool creation: bad heater number");
			return nullptr;
		}
	}

	Tool * const t = new Tool;

	if (filamentDrive >= 0 && filamentDrive < (int)MaxExtruders)
	{
		// Use exactly only one Filament instance per extruder drive
		Filament * const filament = Filament::GetFilamentByExtruder(filamentDrive);
		t->filament = (filament == nullptr) ? new Filament(d[0]) : filament;
	}
	else
	{
		// Don't support filament codes for other tools
		t->filament = nullptr;
	}

	const size_t nameLength = strlen(name);
	if (nameLength != 0)
	{
		char *toolName = new char[nameLength + 1];
		SafeStrncpy(toolName, name, nameLength + 1);
		t->name = toolName;
	}
	else
	{
		t->name = nullptr;
	}

	t->next = nullptr;
	t->myNumber = (uint16_t)toolNumber;
	t->state = ToolState::off;
	t->driveCount = (uint8_t)dCount;
	t->heaterCount = (uint8_t)hCount;
	t->axisMapping[0] = xMap;
	t->axisMapping[1] = yMap;
	t->fanMapping = fanMap;
	t->heaterFault = false;
	t->axisOffsetsProbed.Clear();
	t->displayColdExtrudeWarning = false;

	for (size_t axis = 0; axis < MaxAxes; axis++)
	{
		t->offset[axis] = 0.0;
	}

	for (size_t drive = 0; drive < t->driveCount; drive++)
	{
		t->drives[drive] = d[drive];
		t->mix[drive] = (drive == 0) ? 1.0 : 0.0;		// initial mix ratio is 1:0:0
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

/*static*/ AxesBitmap Tool::GetXAxes(const Tool *tool) noexcept
{
	return (tool == nullptr) ? DefaultXAxisMapping : tool->axisMapping[0];
}

/*static*/ AxesBitmap Tool::GetYAxes(const Tool *tool) noexcept
{
	return (tool == nullptr) ? DefaultYAxisMapping : tool->axisMapping[1];
}

/*static*/ float Tool::GetOffset(const Tool *tool, size_t axis) noexcept
{
	return (tool == nullptr) ? 0.0 : tool->offset[axis];
}

void Tool::Print(const StringRef& reply) const noexcept
{
	reply.printf("Tool %u - ", myNumber);
	if (name != nullptr)
	{
		reply.catf("name: %s; ", name);
	}

	if (driveCount == 0)
	{
		reply.cat("no drives");
	}
	else
	{
		reply.cat("drives:");
		char sep = ' ';
		for (size_t drive = 0; drive < driveCount; drive++)
		{
			reply.catf("%c%d", sep, drives[drive]);
			sep = ',';
		}
	}

	if (heaterCount == 0)
	{
		reply.cat("; no heaters");
	}
	else
	{
		reply.cat("; heaters (active/standby temps):");
		char sep = ' ';
		for (size_t heater = 0; heater < heaterCount; heater++)
		{
			reply.catf("%c%d (%.1f/%.1f)", sep, heaters[heater], (double)activeTemperatures[heater], (double)standbyTemperatures[heater]);
			sep = ',';
		}
	}

	reply.cat("; xmap:");
	char sep = ' ';
	for (size_t xi = 0; xi < MaxAxes; ++xi)
	{
		if (axisMapping[0].IsBitSet(xi))
		{
			reply.catf("%c%c", sep, reprap.GetGCodes().GetAxisLetters()[xi]);
			sep = ',';
		}
	}

	reply.cat("; ymap:");
	sep = ' ';
	for (size_t yi = 0; yi < MaxAxes; ++yi)
	{
		if (axisMapping[1].IsBitSet(yi))
		{
			reply.catf("%c%c", sep, reprap.GetGCodes().GetAxisLetters()[yi]);
			sep = ',';
		}
	}

	reply.cat("; fans:");
	sep = ' ';
	for (size_t fi = 0; fi < MaxFans; ++fi)
	{
		if (fanMapping.IsBitSet(fi))
		{
			reply.catf("%c%u", sep, fi);
			sep = ',';
		}
	}

	reply.catf("; status: %s", (state == ToolState::active) ? "selected" : (state == ToolState::standby) ? "standby" : "off");
}

// There is a temperature fault on a heater, so disable all tools using that heater.
// This function must be called for the first entry in the linked list.
void Tool::FlagTemperatureFault(int8_t heater) noexcept
{
	Tool* n = this;
	while (n != nullptr)
	{
		n->SetTemperatureFault(heater);
		n = n->Next();
	}
}

void Tool::ClearTemperatureFault(int8_t heater) noexcept
{
	Tool* n = this;
	while (n != nullptr)
	{
		n->ResetTemperatureFault(heater);
		n = n->Next();
	}
}

void Tool::SetTemperatureFault(int8_t dudHeater) noexcept
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

void Tool::ResetTemperatureFault(int8_t wasDudHeater) noexcept
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

bool Tool::AllHeatersAtHighTemperature(bool forExtrusion) const noexcept
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		const float temperature = reprap.GetHeat().GetHeaterTemperature(heaters[heater]);
		if (temperature < reprap.GetHeat().GetRetractionMinTemp() || (forExtrusion && temperature < reprap.GetHeat().GetExtrusionMinTemp()))
		{
			return false;
		}
	}
	return true;
}

void Tool::Activate() noexcept
{
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		reprap.GetHeat().SetActiveTemperature(heaters[heater], activeTemperatures[heater]);
		reprap.GetHeat().SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
		String<1> dummy;
		(void)reprap.GetHeat().Activate(heaters[heater], dummy.GetRef());
	}
	state = ToolState::active;
}

void Tool::Standby() noexcept
{
	const Tool * const currentTool = reprap.GetCurrentTool();
	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		// Don't switch a heater to standby if the active tool is using it and is different from this tool
		if (currentTool == this || currentTool == nullptr || !currentTool->UsesHeater(heater))
		{
			reprap.GetHeat().SetStandbyTemperature(heaters[heater], standbyTemperatures[heater]);
			reprap.GetHeat().Standby(heaters[heater], this);
		}
	}
	state = ToolState::standby;
}

// May be called from ISR
bool Tool::ToolCanDrive(bool extrude) noexcept
{
	if (!heaterFault && AllHeatersAtHighTemperature(extrude))
	{
		return true;
	}

	displayColdExtrudeWarning = true;
	return false;
}

// Update the number of active drives and extruders in use to reflect what this tool uses
void Tool::UpdateExtruderAndHeaterCount(uint16_t &numExtruders, uint16_t &numHeaters, uint16_t &numToolsToReport) const noexcept
{
	for (size_t drive = 0; drive < driveCount; drive++)
	{
		if (drives[drive] >= numExtruders)
		{
			numExtruders = drives[drive] + 1;
		}
	}

	for (size_t heater = 0; heater < heaterCount; heater++)
	{
		if (!reprap.GetHeat().IsBedOrChamberHeater(heaters[heater]) && heaters[heater] >= numHeaters)
		{
			numHeaters = heaters[heater] + 1;
		}
	}

	if (myNumber >= numToolsToReport)
	{
		numToolsToReport = myNumber + 1;
	}
}

bool Tool::DisplayColdExtrudeWarning() noexcept
{
	bool result = displayColdExtrudeWarning;
	displayColdExtrudeWarning = false;
	return result;
}

void Tool::DefineMix(const float m[]) noexcept
{
	for(size_t drive = 0; drive < driveCount; drive++)
	{
		mix[drive] = m[drive];
	}
}

#if HAS_MASS_STORAGE

// Write the tool's settings to file returning true if successful. The settings written leave the tool selected unless it is off.
bool Tool::WriteSettings(FileStore *f) const noexcept
{
	String<StringLength50> buf;
	bool ok = true;

	// Set up active and standby heater temperatures
	if (heaterCount != 0)
	{
		buf.printf("G10 P%d ", myNumber);
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
		ok = f->Write(buf.c_str());
	}

	if (ok && state != ToolState::off)
	{
		ok = buf.printf("T%d P0\n", myNumber);
	}

	return ok;
}

#endif

void Tool::SetOffset(size_t axis, float offs, bool byProbing) noexcept
{
	offset[axis] = offs;
	if (byProbing)
	{
		axisOffsetsProbed.SetBit(axis);
	}
}

float Tool::GetToolHeaterActiveTemperature(size_t heaterNumber) const noexcept
{
	return (heaterNumber < heaterCount) ? activeTemperatures[heaterNumber] : 0.0;
}

float Tool::GetToolHeaterStandbyTemperature(size_t heaterNumber) const noexcept
{
	return (heaterNumber < heaterCount) ? standbyTemperatures[heaterNumber] : 0.0;
}

void Tool::SetToolHeaterActiveTemperature(size_t heaterNumber, float temp) noexcept
{
	if (heaterNumber < heaterCount)
	{
		const Tool * const currentTool = reprap.GetCurrentTool();
		const bool setHeater = (currentTool == nullptr || currentTool == this);
		if (temp < NEARLY_ABS_ZERO)								// temperatures close to ABS_ZERO turn off the heater
		{
			activeTemperatures[heaterNumber] = 0;
			if (setHeater)
			{
				reprap.GetHeat().SwitchOff(heaters[heaterNumber]);
			}
		}
		else
		{
			const float minTemperatureLimit = reprap.GetHeat().GetLowestTemperatureLimit(heaters[heaterNumber]);
			const float maxTemperatureLimit = reprap.GetHeat().GetHighestTemperatureLimit(heaters[heaterNumber]);
			if (temp > minTemperatureLimit && temp < maxTemperatureLimit)
			{
				activeTemperatures[heaterNumber] = temp;
				if (setHeater)
				{
					reprap.GetHeat().SetActiveTemperature(heaters[heaterNumber], activeTemperatures[heaterNumber]);
				}
			}
		}
	}
}

void Tool::SetToolHeaterStandbyTemperature(size_t heaterNumber, float temp) noexcept
{
	if (heaterNumber < heaterCount)
	{
		const Tool * const currentTool = reprap.GetCurrentTool();
		const Tool * const lastStandbyTool = reprap.GetHeat().GetLastStandbyTool(heaters[heaterNumber]);
		const bool setHeater = (currentTool == nullptr || currentTool == this || lastStandbyTool == nullptr || lastStandbyTool == this);
		if (temp < NEARLY_ABS_ZERO)								// temperatures close to ABS_ZERO turn off the heater
		{
			standbyTemperatures[heaterNumber] = 0;
			if (setHeater)
			{
				reprap.GetHeat().SwitchOff(heaters[heaterNumber]);
			}
		}
		else
		{
			const float minTemperatureLimit = reprap.GetHeat().GetLowestTemperatureLimit(heaters[heaterNumber]);
			const float maxTemperatureLimit = reprap.GetHeat().GetHighestTemperatureLimit(heaters[heaterNumber]);
			if (temp > minTemperatureLimit && temp < maxTemperatureLimit)
			{
				standbyTemperatures[heaterNumber] = temp;
				if (setHeater)
				{
					reprap.GetHeat().SetStandbyTemperature(heaters[heaterNumber], standbyTemperatures[heaterNumber]);
				}
			}
		}
	}
}

void Tool::IterateExtruders(std::function<void(unsigned int)> f) const noexcept
{
	for (size_t i = 0; i < driveCount; ++i)
	{
		f(drives[i]);
	}
}

void Tool::IterateHeaters(std::function<void(int)> f) const noexcept
{
	for (size_t i = 0; i < heaterCount; ++i)
	{
		f(heaters[i]);
	}
}

// Return true if this tool uses the specified heater
bool Tool::UsesHeater(int8_t heater) const noexcept
{
	for (size_t i = 0; i < heaterCount; ++i)
	{
		if (heaters[i] == heater)
		{
			return true;
		}
	}
	return false;
}

// End
