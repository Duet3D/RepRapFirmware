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
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	uint16_t AxesToHomeBeforeProbing() const override { return (1 << X_AXIS) | (1 << Y_AXIS) | (1 << Z_AXIS); }
	bool DriveIsShared(size_t drive) const override;

protected:
	float MotorFactor(size_t drive, const float directionVector[]) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXZKINEMATICS_H_ */
