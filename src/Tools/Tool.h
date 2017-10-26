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

#include "RepRapFirmware.h"

const size_t ToolNameLength = 32;						// maximum allowed length for tool names
const AxesBitmap DefaultXAxisMapping = 1u << X_AXIS;	// by default, X is mapped to X
const AxesBitmap DefaultYAxisMapping = 1u << Y_AXIS;	// by default, Y is mapped to Y

enum class ToolState : uint8_t
{
	off = 0,
	active,
	standby
};

class Filament;
class Tool
{
public:

	static Tool *Create(int toolNumber, const char *name, long d[], size_t dCount, long h[], size_t hCount, AxesBitmap xMap, AxesBitmap yMap, FansBitmap fanMap, StringRef& reply);
	static void Delete(Tool *t);

	float GetOffset(size_t axis) const pre(axis < MaxAxes);
	void SetOffset(size_t axis, float offs, bool byProbing) pre(axis < MaxAxes);
	AxesBitmap GetAxisOffsetsProbed() const { return axisOffsetsProbed; }
	size_t DriveCount() const;
	int Drive(size_t driveNumber) const;
	bool ToolCanDrive(bool extrude);
	size_t HeaterCount() const;
	int Heater(size_t heaterNumber) const;
	const char *GetName() const;
	int Number() const;
	void SetVariables(const float* standby, const float* active);
	void GetVariables(float* standby, float* active) const;
	void DefineMix(const float m[]);
	const float* GetMix() const;
	float MaxFeedrate() const;
	float InstantDv() const;
	void Print(StringRef& reply);
	AxesBitmap GetXAxisMap() const { return xMapping; }
	AxesBitmap GetYAxisMap() const { return yMapping; }
	FansBitmap GetFanMapping() const { return fanMapping; }
	Filament *GetFilament() const { return filament; }
	Tool *Next() const { return next; }
	ToolState GetState() const { return state; }
	bool WriteSettings(FileStore *f) const;			// write the tool's settings to file

	friend class RepRap;

protected:
	void Activate(Tool* currentlyActive);
	void Standby();
	void FlagTemperatureFault(int8_t dudHeater);
	void ClearTemperatureFault(int8_t wasDudHeater);
	void UpdateExtruderAndHeaterCount(uint16_t &extruders, uint16_t &heaters) const;
	bool DisplayColdExtrudeWarning();

private:
	static Tool *freelist;

	void SetTemperatureFault(int8_t dudHeater);
	void ResetTemperatureFault(int8_t wasDudHeater);
	bool AllHeatersAtHighTemperature(bool forExtrusion) const;

	int myNumber;
	char name[ToolNameLength];
	int drives[MaxExtruders];
	float mix[MaxExtruders];
	size_t driveCount;
	int heaters[Heaters];
	float activeTemperatures[Heaters];
	float standbyTemperatures[Heaters];
	size_t heaterCount;
	float offset[MaxAxes];
	AxesBitmap axisOffsetsProbed;
	AxesBitmap xMapping, yMapping;
	FansBitmap fanMapping;
	Filament *filament;
	Tool* next;

	ToolState state;
	bool heaterFault;
	volatile bool displayColdExtrudeWarning;
};

inline int Tool::Drive(size_t driveNumber) const
{
	return drives[driveNumber];
}

inline size_t Tool::HeaterCount() const
{
	return heaterCount;
}

inline int Tool::Heater(size_t heaterNumber) const
{
	return heaters[heaterNumber];
}

inline const char *Tool::GetName() const
{
	return name;
}

inline int Tool::Number() const
{
	return myNumber;
}

inline const float* Tool::GetMix() const
{
	return mix;
}

inline size_t Tool::DriveCount() const
{
	return driveCount;
}

inline float Tool::GetOffset(size_t axis) const
{
	return offset[axis];
}

#endif /* TOOL_H_ */
