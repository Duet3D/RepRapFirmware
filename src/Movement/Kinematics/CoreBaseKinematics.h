/*
 * CoreBaseKinematics.h
 *
 *  Created on: 7 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_

#include "Kinematics.h"

class CoreBaseKinematics : public Kinematics
{
public:
	CoreBaseKinematics(KinematicsType t);

	// Overridden base class functions. See Kinematics.h for descriptions.
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numAxes, int32_t motorPos[]) const override final;
	bool SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override final;
	bool SupportsAutoCalibration() const override final { return false; }
	bool ShowCoordinatesWhenNotHomed() const override { return true; }

protected:
	float axisFactors[CART_AXES];
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_ */
