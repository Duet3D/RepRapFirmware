/*
 * CoreXYUKinematics.h
 *
 *  Created on: 4 Jun 2017
 *      Author: Lars
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREXYUKINEMATICS_H_

#include "CoreBaseKinematics.h"

class CoreXYUKinematics : public CoreBaseKinematics
{
public:
	CoreXYUKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool DriveIsShared(size_t drive) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_ */
