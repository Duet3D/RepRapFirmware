/*
 * CoreXYKinematics.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_

#include "CoreBaseKinematics.h"

class CoreXYKinematics : public CoreBaseKinematics
{
public:
	CoreXYKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool DriveIsShared(size_t drive) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_ */
