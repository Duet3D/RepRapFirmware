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

class ClosedLoopStatus;
class OpenLoopStatus;

// Data structure to represent driver parameters
class DriverData INHERIT_OBJECT_MODEL
{
protected:
	DECLARE_OBJECT_MODEL

public:
	DriverData() noexcept { configuredDirection = 0; }

	void StoreCurrent(uint16_t current) noexcept { configuredCurrent = current; }
	void StoreDirection(bool dir) noexcept { configuredDirection = dir; }
	void StoreClosedLoopStatus(const ClosedLoopStatus& clStatus) noexcept;
	void StoreOpenLoopStatus(const OpenLoopStatus& olStatus) noexcept;

private:
	// Reported status
	StandardDriverStatus status;

	// Configured values
	uint16_t configuredCurrent = 0;
	uint16_t configuredDirection : 1;

	// Fields for closed loop data collection
	bool haveClosedLoopData = false;
	float16_t averageCurrentFraction = 0.0, maxCurrentFraction = 0.0, rmsPositionError = 0.0, maxAbsPositionError = 0.0;
};

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_ */
