/*
 * DriverData.h
 *
 *  Created on: 25 Sept 2023
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>

// Data structure to represent driver parameters
class DriverData INHERIT_OBJECT_MODEL
{
protected:
	DECLARE_OBJECT_MODEL

public:
	StandardDriverStatus status;
	// Add additional fields here e.g. configured motor current
	// ...
	// Fields for closed loop data collection
	bool haveClosedLoopData = false;
	float16_t averageCurrentFraction = 0.0, maxCurrentFraction = 0.0, rmsPositionError = 0.0, maxAbsPositionError = 0.0;
};

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_ */
