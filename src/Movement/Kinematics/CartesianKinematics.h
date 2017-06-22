/*
 * CartesianKinematics.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_CARTESIANKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_CARTESIANKINEMATICS_H_

#include "Kinematics.h"

class CartesianKinematics : public Kinematics
{
public:
	CartesianKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
    bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[]) const override;
    void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool SupportsAutoCalibration() const override { return false; }
	bool DriveIsShared(size_t drive) const override { return false; }
	HomingMode GetHomingMode() const override { return homeCartesianAxes; }
};

#endif /* SRC_MOVEMENT_KINEMATICS_CARTESIANKINEMATICS_H_ */
