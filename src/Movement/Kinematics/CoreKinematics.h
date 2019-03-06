/*
 * CoreKinematics.h
 *
 *  Created on: 4 Jan 2019
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREKINEMATICS_H_

#include "ZLeadscrewKinematics.h"
#include "Math/Matrix.h"

class CoreKinematics : public ZLeadscrewKinematics
{
public:
	CoreKinematics(KinematicsType k);

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool QueryTerminateHomingMove(size_t axis) const override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const override;
	HomingMode GetHomingMode() const override { return HomingMode::homeCartesianAxes; }
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const override;
	AxesBitmap GetConnectedAxes(size_t axis) const override;
	AxesBitmap GetLinearAxes() const override;

private:
	void Recalc();											// recalculate internal variables following a configuration change
	bool HasSharedMotor(size_t axis) const;					// return true if the axis doesn't have a single dedicated motor

	// Primary parameters
	FixedMatrix<float, MaxAxes, MaxAxes> inverseMatrix;		// maps coordinates to motor positions

	// Derived parameters
	FixedMatrix<float, MaxAxes, MaxAxes> forwardMatrix;		// maps motor positions to coordinates
	AxesBitmap connectedAxes[MaxAxes];						// which other axes are connected to each axis by shared motors etc.
	bool modified;											// true if matrix has been altered
	uint8_t firstMotor[MaxAxes], lastMotor[MaxAxes];		// first and last motor used by each axis
	uint8_t firstAxis[MaxAxes], lastAxis[MaxAxes];			// first and last axis that each motor controls
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREKINEMATICS_H_ */
