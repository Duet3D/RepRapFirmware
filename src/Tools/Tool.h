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
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>
#include <General/FreelistManager.h>
#include <General/NamedEnum.h>
#include <General/function_ref.h>

// Bits for T-code P-parameter to specify which macros are supposed to be run
constexpr uint8_t TFreeBit = 1u << 0;
constexpr uint8_t TPreBit = 1u << 1;
constexpr uint8_t TPostBit = 1u << 2;
constexpr uint8_t DefaultToolChangeParam = TFreeBit | TPreBit | TPostBit;

constexpr size_t ToolNameLength = 32;						// maximum allowed length for tool names

NamedEnum(ToolState, uint8_t, off, active, standby);

class Filament;

class Tool INHERIT_OBJECT_MODEL
{
public:
	DECLARE_FREELIST_NEW_DELETE(Tool)

	~Tool() override { delete name; }

	static Tool *Create(
			unsigned int toolNumber,
			const char *_ecv_array toolName,
			int32_t d[], size_t dCount,
			int32_t h[], size_t hCount,
			AxesBitmap xMap,
			AxesBitmap yMap,
			FansBitmap fanMap,
			int filamentDrive,
			size_t sCount,
			int8_t spindleNo,
			const StringRef& reply) noexcept;
	static void Delete(Tool *t) noexcept { delete t; }
	static AxesBitmap GetXAxes(const Tool *tool) noexcept;
	static AxesBitmap GetYAxes(const Tool *tool) noexcept;
	static AxesBitmap GetAxisMapping(const Tool *tool, unsigned int axis) noexcept;
	static float GetOffset(const Tool *tool, size_t axis) noexcept pre(axis < MaxAxes);
	static void FlagTemperatureFault(int8_t dudHeater) noexcept;
	static GCodeResult ClearTemperatureFault(int8_t wasDudHeater, const StringRef& reply) noexcept;
	static void AddTool(Tool* t) noexcept;
	static void DeleteTool(int toolNumber) noexcept;
	static uint16_t GetExtrudersInUse() noexcept { return activeExtruders; }
	static uint16_t GetToolHeatersInUse() noexcept { return activeToolHeaters; }
	static uint16_t GetNumToolsToReport() noexcept { return numToolsToReport; }
	static Tool *GetToolList() noexcept { return toolList; }
	static ReadLockedPointer<Tool> GetLockedTool(int toolNumber) noexcept;
	static unsigned int GetNumberOfContiguousTools() noexcept;
	static unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions, const Tool *tool) noexcept;
	static bool DisplayColdExtrusionWarnings() noexcept;
	static bool IsHeaterAssignedToTool(int8_t heater) noexcept;
	static GCodeResult SetAllToolsFirmwareRetraction(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& outBuf) THROWS(GCodeException);

	float GetOffset(size_t axis) const noexcept pre(axis < MaxAxes);
	void SetOffset(size_t axis, float offs, bool byProbing) noexcept pre(axis < MaxAxes);
	AxesBitmap GetAxisOffsetsProbed() const noexcept { return axisOffsetsProbed; }
	size_t DriveCount() const noexcept;
	int GetDrive(size_t driveNumber) const noexcept pre(driverNumber < DriveCount());
	bool CanDriveExtruder(bool extrude) const noexcept;
	size_t HeaterCount() const noexcept;
	int GetHeater(size_t heaterNumber) const noexcept pre(heaterNumber < HeaterCount());
	const char *_ecv_array GetName() const noexcept;
	int Number() const noexcept;
	void DefineMix(const float m[]) noexcept;
	const float *_ecv_array GetMix() const noexcept;
	void PrintTool(const StringRef& reply) const noexcept;
	AxesBitmap GetXAxisMap() const noexcept { return axisMapping[0]; }
	AxesBitmap GetYAxisMap() const noexcept { return axisMapping[1]; }
	FansBitmap GetFanMapping() const noexcept { return fanMapping; }
	Filament *GetFilament() const noexcept { return filament; }
	const char *GetFilamentName() const noexcept;
	Tool *null Next() const noexcept { return next; }
	ToolState GetState() const noexcept { return state; }

	bool IsRetracted() const noexcept { return isRetracted; }
	float GetRetractLength() const noexcept { return retractLength; }
	float GetRetractHop() const noexcept { return retractHop; }
	float GetRetractExtra() const noexcept { return retractExtra; }
	float GetRetractSpeed() const noexcept { return retractSpeed; }
	float GetUnRetractSpeed() const noexcept { return unRetractSpeed; }
	void SetRetracted(bool b) noexcept { isRetracted = b; }
	int8_t GetSpindleNumber() const noexcept { return spindleNumber; }
	uint32_t GetSpindleRpm() const noexcept { return spindleRpm; }
	void SetSpindleRpm(uint32_t rpm, bool isCurrentTool) THROWS(GCodeException);

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteSettings(FileStore *f) const noexcept;							// write the tool's settings to file
#endif

	float GetToolHeaterActiveTemperature(size_t heaterNumber) const noexcept;
	float GetToolHeaterStandbyTemperature(size_t heaterNumber) const noexcept;
	void SetToolHeaterActiveTemperature(size_t heaterNumber, float temp) THROWS(GCodeException) { SetToolHeaterActiveOrStandbyTemperature(heaterNumber, temp, true); }
	void SetToolHeaterStandbyTemperature(size_t heaterNumber, float temp) THROWS(GCodeException) { SetToolHeaterActiveOrStandbyTemperature(heaterNumber, temp, false); }

	GCodeResult SetFirmwareRetraction(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& outBuf) THROWS(GCodeException);
	GCodeResult GetSetFeedForward(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	bool HasTemperatureFault() const noexcept { return heaterFault; }

	void IterateExtruders(function_ref<void(unsigned int)> f) const noexcept;
	void IterateHeaters(function_ref<void(int)> f) const noexcept;
	bool UsesHeater(int8_t heater) const noexcept;

	void SetFansPwm(float f) const noexcept;

	void HeatersToOff() const noexcept;
	void HeatersToActiveOrStandby(bool active) const noexcept;

	void ApplyFeedForward(float extrusionSpeed) const noexcept;
	void StopFeedForward() const noexcept;

	void Activate() noexcept;
	void Standby() noexcept;
	void UpdateExtruderAndHeaterCount(uint16_t &numExtruders, uint16_t &numHeaters, uint16_t &numToolsToReport) const noexcept;

	static ReadWriteLock toolListLock;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(activeTemps)
	OBJECT_MODEL_ARRAY(axes)
	OBJECT_MODEL_ARRAY(extruders)
	OBJECT_MODEL_ARRAY(heaters)
	OBJECT_MODEL_ARRAY(mix)
	OBJECT_MODEL_ARRAY(offsets)
	OBJECT_MODEL_ARRAY(standbyTemps)
	OBJECT_MODEL_ARRAY(feedForward)

private:
	Tool() noexcept : next(nullptr), filament(nullptr), name(nullptr), state(ToolState::off) { }

	void SetToolHeaterActiveOrStandbyTemperature(size_t heaterNumber, float temp, bool active) THROWS(GCodeException);
	void SetTemperatureFault(int8_t dudHeater) noexcept;
	void ResetTemperatureFault(int8_t wasDudHeater) noexcept;
	bool AllHeatersAtHighTemperature(bool forExtrusion) const noexcept;

	static void ToolUpdated() noexcept { reprap.ToolsUpdated(); }	// call this whenever we change a variable that is reported in the OM as non-live

	static Tool* toolList;						// the tool list is sorted in order of increasing tool number
	static ToolNumbersBitmap prohibitedExtrusionTools;
	static uint16_t activeExtruders;
	static uint16_t activeToolHeaters;
	static uint16_t numToolsToReport;

	Tool* null next;
	Filament *filament;
	int filamentExtruder;
	const char *_ecv_array name;
	float offset[MaxAxes];
	float mix[MaxExtrudersPerTool];
	float activeTemperatures[MaxHeatersPerTool];
	float standbyTemperatures[MaxHeatersPerTool];
	float heaterFeedForward[MaxHeatersPerTool];

	// Firmware retraction settings
	float retractLength, retractExtra;			// retraction length and extra length to un-retract
	float retractSpeed;							// retract speed in mm per step clock
	float unRetractSpeed;						// un-retract speed in mm per step clock
	float retractHop;							// Z hop when retracting

	FansBitmap fanMapping;
	uint8_t driveCount;
	uint8_t heaterCount;
	uint16_t myNumber;
	AxesBitmap axisMapping[2];
	AxesBitmap axisOffsetsProbed;

	uint8_t drives[MaxExtrudersPerTool];
	int8_t heaters[MaxHeatersPerTool];

	int8_t spindleNumber;
	uint32_t spindleRpm;

	ToolState state;
	bool heaterFault;
	bool isRetracted;							// true if filament has been firmware-retracted
};

inline int Tool::GetDrive(size_t driveNumber) const noexcept
{
	return drives[driveNumber];
}

inline size_t Tool::HeaterCount() const noexcept
{
	return heaterCount;
}

inline int Tool::GetHeater(size_t heaterNumber) const noexcept
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
