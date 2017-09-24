/*
 * CoreXZKinematics.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXZKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREXZKINEMATICS_H_

#include "CoreBaseKinematics.h"

class CoreXZKinematics : public CoreBaseKinematics
{
public:
	CoreXZKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool allowModeChange) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	AxesBitmap AxesToHomeBeforeProbing() const override { return MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS); }
	bool DriveIsShared(size_t drive) const override;
	bool SupportsAutoCalibration() const override { return false; }
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXZKINEMATICS_H_ */
