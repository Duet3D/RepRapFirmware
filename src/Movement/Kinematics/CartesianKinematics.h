/*
 * CartesianKinematics.h
 *
 *  Created on: 6 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_CARTESIANKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_CARTESIANKINEMATICS_H_

#include "ZLeadscrewKinematics.h"

class CartesianKinematics : public ZLeadscrewKinematics
{
public:
	CartesianKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
    bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
    void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	HomingMode GetHomingMode() const override { return homeCartesianAxes; }
	bool QueryTerminateHomingMove(size_t axis) const override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_CARTESIANKINEMATICS_H_ */
