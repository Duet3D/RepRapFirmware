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
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[]) const override final;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override final;
	bool SupportsAutoCalibration() const override final { return false; }

protected:
	// Calculate the movement fraction for a single axis motor of a Cartesian-like printer.
	// The default implementation just returns directionVector[drive] but this needs to be overridden for CoreXY and CoreXZ printers.
	virtual float MotorFactor(size_t drive, const float directionVector[]) const = 0;

	float axisFactors[XYZ_AXES];
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_ */
