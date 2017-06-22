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
	bool Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool DriveIsShared(size_t drive) const override;

protected:
	float MotorFactor(size_t drive, const float directionVector[]) const override;
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREXYKINEMATICS_H_ */
