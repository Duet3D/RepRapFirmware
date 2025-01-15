/*
 * CoreKinematics.h
 *
 *  Created on: 4 Jan 2019
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREKINEMATICS_H_

#include "ZLeadscrewKinematics.h"
#include <Math/Matrix.h>

class CoreKinematics : public ZLeadscrewKinematics
{
public:
	CoreKinematics(KinematicsType k) noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *_ecv_array GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeCartesianAxes; }
	void LimitSpeedAndAcceleration(DDA& dda, const float *_ecv_array normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	LogicalDrivesBitmap GetControllingDrives(size_t axis, bool forHoming) const noexcept override;
	void ConvertAxisAmountsToLogicalDriveAmounts(float amounts[MaxAxes], size_t numVisibleAxes, size_t numTotalAxes) const noexcept override;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	void Recalc() noexcept;									// recalculate internal variables following a configuration change
	bool HasSharedMotor(size_t axis) const noexcept;		// return true if the axis doesn't have a single dedicated motor

	// Primary parameters
	FixedMatrix<float, MaxAxes, MaxAxes> inverseMatrix;		// maps coordinates to motor positions

	// Derived parameters
	FixedMatrix<float, MaxAxes, MaxAxes> forwardMatrix;		// maps motor positions to coordinates
	LogicalDrivesBitmap controllingDrivers[MaxAxes];		// which drives control each axis
	bool modified;											// true if matrix has been altered
	uint8_t firstMotor[MaxAxes], lastMotor[MaxAxes];		// first and last motor used by each axis
	uint8_t firstAxis[MaxAxes], lastAxis[MaxAxes];			// first and last axis that each motor controls
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREKINEMATICS_H_ */
