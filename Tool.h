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

	Tool(int tNum, int d[], int h[]);

	friend class RepRap;

protected:

	Tool* Next();
	int Number();
	void Activate(Tool* currentlyActive);
	void Standby();
	void AddTool(Tool* t);
	void SetVariables(float xx, float yy, float zz, float* standbyTemperatures, float* activeTemperatures);
	void GetOffset(float& xx, float& yy, float& zz);

private:

	int myNumber;
	int* drives;
	int driveCount;
	int* heaters;
	int heaterCount;
	Tool* next;
	float x, y, z;
	bool active;
};

inline Tool* Tool::Next()
{
	return next;
}

inline int Tool::Number()
{
	return myNumber;
}

inline void Tool::GetOffset(float& xx, float& yy, float& zz)
{
	xx = x;
	yy = y;
	zz = z;
}



#endif /* TOOL_H_ */
