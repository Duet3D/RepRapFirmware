/*
 * DriverData.h
 *
 *  Created on: 25 Sept 2023
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_

#include <RepRapFirmware.h>

#if SUPPORT_CAN_EXPANSION

#include <ObjectModel/ObjectModel.h>
#include "DriverMode.h"

class ClosedLoopStatus;
class OpenLoopStatus;

// Data structure to represent driver parameters
class DriverData INHERIT_OBJECT_MODEL
{
protected:
	DECLARE_OBJECT_MODEL

public:
	DriverData() noexcept { configuredDirection = 1; configuredMode = (uint16_t)DriverMode::spreadCycle; }

	bool GetDirection() const noexcept { return configuredDirection; }
	DriverMode GetMode() const noexcept { return (DriverMode)configuredMode; }

	void StoreDirection(bool dir) noexcept { configuredDirection = dir; }
	void StoreMode(uint32_t mode) noexcept { configuredMode = (uint16_t)mode; }
	void StoreClosedLoopStatus(const ClosedLoopStatus& clStatus) noexcept;
	void StoreOpenLoopStatus(const OpenLoopStatus& olStatus) noexcept;

private:
	// Reported status
	StandardDriverStatus status;

	// Configured values
	uint16_t configuredMode : 3,
			 configuredDirection : 1;

	// Fields for closed loop data collection
	bool haveClosedLoopData = false;
	float16_t averageCurrentFraction = 0.0, maxCurrentFraction = 0.0, rmsPositionError = 0.0, maxAbsPositionError = 0.0;
};

#endif

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_DRIVERDATA_H_ */
