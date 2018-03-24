/*
 * CoreXYUVKinematics.h
 *
 *  Created on: 23 Nov 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXYUVKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREXYUVKINEMATICS_H_

#include "CoreBaseKinematics.h"

class CoreXYUVKinematics : public CoreBaseKinematics
{
public:
	CoreXYUVKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool DriveIsShared(size_t drive) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYUVKINEMATICS_H_ */
