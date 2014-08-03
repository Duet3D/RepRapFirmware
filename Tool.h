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

class Tool
{
public:

	Tool(int toolNumber, long d[], int dCount, long h[], int hCount);
	void GetOffset(float& xx, float& yy, float& zz) const;
	int DriveCount() const;
	int Drive(int driveNumber) const;
	bool ToolCanDrive() const;
	int HeaterCount() const;
	int Heater(int heaterNumber) const;
	int Number() const;
	void SetVariables(float* standby, float* active);
	void GetVariables(float* standby, float* active) const;
	void DefineMix(float* m);
	float* GetMix() const;
	void TurnMixingOn();
	void TurnMixingOff();
	bool Mixing() const;
	float MaxFeedrate() const;
	float InstantDv() const;
	void Print(char* reply);

	friend class RepRap;

protected:

	Tool* Next() const;
	void Activate(Tool* currentlyActive);
	void Standby();
	void AddTool(Tool* tool);
	void FlagTemperatureFault(int8_t dudHeater);
	void ClearTemperatureFault(int8_t wasDudHeater);
	void UpdateExtruderAndHeaterCount(uint16_t &extruders, uint16_t &heaters) const;

private:

	void SetTemperatureFault(int8_t dudHeater);
	void ResetTemperatureFault(int8_t wasDudHeater);
	bool AllHeatersAtHighTemperature() const;
	int myNumber;
	int* drives;
	float* mix;
	bool mixing;
	int driveCount;
	int* heaters;
	float* activeTemperatures;
	float* standbyTemperatures;
	int heaterCount;
	Tool* next;
	bool active;
	bool heaterFault;
};

inline int Tool::Drive(int driveNumber) const
{
	return drives[driveNumber];
}

inline int Tool::HeaterCount() const
{
	return heaterCount;
}

inline int Tool::Heater(int heaterNumber) const
{
	return heaters[heaterNumber];
}

inline Tool* Tool::Next() const
{
	return next;
}

inline int Tool::Number() const
{
	return myNumber;
}

inline void Tool::DefineMix(float* m)
{
	for(int8_t drive = 0; drive < driveCount; drive++)
	{
		mix[drive] = m[drive];
	}
}

inline float* Tool::GetMix() const
{
	return mix;
}

inline void Tool::TurnMixingOn()
{
	mixing = true;
}

inline void Tool::TurnMixingOff()
{
	mixing = false;
}

inline bool Tool::Mixing() const
{
	return mixing;
}

inline int Tool::DriveCount() const
{
	return driveCount;
}

#endif /* TOOL_H_ */
