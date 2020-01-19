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

#ifndef TOOL_H_
#define TOOL_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <General/FreelistManager.h>
#include <NamedEnum.h>

#undef array
#include <functional>
#define array _ecv_array

constexpr size_t ToolNameLength = 32;						// maximum allowed length for tool names

NamedEnum(ToolState, uint8_t, off, active, standby);

class Filament;

class Tool INHERIT_OBJECT_MODEL
{
public:
	friend class RepRap;

	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<Tool>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<Tool>(p); }

	~Tool() override { delete name; }

	static Tool *Create(unsigned int toolNumber, const char *name, int32_t d[], size_t dCount, int32_t h[], size_t hCount, AxesBitmap xMap, AxesBitmap yMap, FansBitmap fanMap, int filamentDrive, const StringRef& reply) noexcept;
	static void Delete(Tool *t) noexcept { delete t; }
	static AxesBitmap GetXAxes(const Tool *tool) noexcept;
	static AxesBitmap GetYAxes(const Tool *tool) noexcept;
	static float GetOffset(const Tool *tool, size_t axis) noexcept pre(axis < MaxAxes);

	float GetOffset(size_t axis) const noexcept pre(axis < MaxAxes);
	void SetOffset(size_t axis, float offs, bool byProbing) noexcept pre(axis < MaxAxes);
	AxesBitmap GetAxisOffsetsProbed() const noexcept { return axisOffsetsProbed; }
	size_t DriveCount() const noexcept;
	int Drive(size_t driveNumber) const noexcept;
	bool ToolCanDrive(bool extrude) noexcept;
	size_t HeaterCount() const noexcept;
	int Heater(size_t heaterNumber) const noexcept;
	const char *GetName() const noexcept;
	int Number() const noexcept;
	void DefineMix(const float m[]) noexcept;
	const float* GetMix() const noexcept;
	void Print(const StringRef& reply) const noexcept;
	AxesBitmap GetXAxisMap() const noexcept { return axisMapping[0]; }
	AxesBitmap GetYAxisMap() const noexcept { return axisMapping[1]; }
	FansBitmap GetFanMapping() const noexcept { return fanMapping; }
	Filament *GetFilament() const noexcept { return filament; }
	Tool *Next() const noexcept { return next; }
	ToolState GetState() const noexcept { return state; }

#if HAS_MASS_STORAGE
	bool WriteSettings(FileStore *f) const noexcept;							// write the tool's settings to file
#endif

	float GetToolHeaterActiveTemperature(size_t heaterNumber) const noexcept;
	float GetToolHeaterStandbyTemperature(size_t heaterNumber) const noexcept;
	void SetToolHeaterActiveTemperature(size_t heaterNumber, float temp) noexcept;
	void SetToolHeaterStandbyTemperature(size_t heaterNumber, float temp) noexcept;

	bool HasTemperatureFault() const noexcept { return heaterFault; }

	void IterateExtruders(std::function<void(unsigned int)> f) const noexcept;
	void IterateHeaters(std::function<void(int)> f) const noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(activeTemps)
	OBJECT_MODEL_ARRAY(axes)
	OBJECT_MODEL_ARRAY(extruders)
	OBJECT_MODEL_ARRAY(heaters)
	OBJECT_MODEL_ARRAY(mix)
	OBJECT_MODEL_ARRAY(offsets)
	OBJECT_MODEL_ARRAY(standbyTemps)

	void Activate() noexcept;
	void Standby() noexcept;
	void FlagTemperatureFault(int8_t dudHeater) noexcept;
	void ClearTemperatureFault(int8_t wasDudHeater) noexcept;
	void UpdateExtruderAndHeaterCount(uint16_t &extruders, uint16_t &heaters, uint16_t &numToolsToReport) const noexcept;
	bool DisplayColdExtrudeWarning() noexcept;

private:
	Tool() noexcept : next(nullptr), filament(nullptr), name(nullptr), state(ToolState::off) { }

	void SetTemperatureFault(int8_t dudHeater) noexcept;
	void ResetTemperatureFault(int8_t wasDudHeater) noexcept;
	bool AllHeatersAtHighTemperature(bool forExtrusion) const noexcept;
	bool UsesHeater(int8_t heater) const noexcept;

	Tool* next;
	Filament *filament;
	const char *name;
	float offset[MaxAxes];
	float mix[MaxExtrudersPerTool];
	float activeTemperatures[MaxHeatersPerTool];
	float standbyTemperatures[MaxHeatersPerTool];
	uint8_t driveCount;
	uint8_t heaterCount;
	uint16_t myNumber;
	AxesBitmap axisMapping[2];
	AxesBitmap axisOffsetsProbed;
	FansBitmap fanMapping;
	uint8_t drives[MaxExtrudersPerTool];
	int8_t heaters[MaxHeatersPerTool];

	ToolState state;
	bool heaterFault;
	volatile bool displayColdExtrudeWarning;
};

inline int Tool::Drive(size_t driveNumber) const noexcept
{
	return drives[driveNumber];
}

inline size_t Tool::HeaterCount() const noexcept
{
	return heaterCount;
}

inline int Tool::Heater(size_t heaterNumber) const noexcept
{
	return heaters[heaterNumber];
}

inline const char *Tool::GetName() const noexcept
{
	return (name == nullptr) ? "" : name;
}

inline int Tool::Number() const noexcept
{
	return myNumber;
}

inline const float* Tool::GetMix() const noexcept
{
	return mix;
}

inline size_t Tool::DriveCount() const noexcept
{
	return driveCount;
}

inline float Tool::GetOffset(size_t axis) const noexcept
{
	return offset[axis];
}

#endif /* TOOL_H_ */
