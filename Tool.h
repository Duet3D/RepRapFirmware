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
	int HeaterCount() const;
	int Heater(int heaterNumber) const;
	int Number() const;
	void SetVariables(float* standby, float* active);
	void GetVariables(float* standby, float* active) const;
	float MaxFeedrate() const;
	float InstantDv() const;
	void Print(char* reply) const;

	friend class RepRap;

protected:

	Tool* Next() const;
	void Activate(Tool* currentlyActive);
	void Standby();
	void AddTool(Tool* t);
	void UpdateExtruderAndHeaterCount(uint16_t &extruders, uint16_t &heaters) const;

private:

	int myNumber;
	int* drives;
	int driveCount;
	int* heaters;
	float* activeTemperatures;
	float* standbyTemperatures;
	int heaterCount;
	Tool* next;
	bool active;
};

inline int Tool::DriveCount() const
{
	return driveCount;
}

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




#endif /* TOOL_H_ */
