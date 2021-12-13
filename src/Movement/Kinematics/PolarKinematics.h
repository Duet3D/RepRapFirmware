/*
 * PolarKinematics.h
 *
 *  Created on: 13 Oct 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_POLARKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_POLARKINEMATICS_H_

#include "Kinematics.h"

class PolarKinematics : public Kinematics
{
public:
	PolarKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultMaxRadius = 150.0;
	static constexpr float DefaultMaxTurntableSpeed = 30.0;				// degrees per second
	static constexpr float DefaultMaxTurntableAcceleration = 30.0;		// degrees per second per second
	static constexpr const char *HomeRadiusFileName = "homeradius.g";
	static constexpr const char *HomeBedFileName = "homebed.g";

	void Recalc();

	float minRadius, maxRadius, homedRadius;
	float maxTurntableSpeed, maxTurntableAcceleration;

	float minRadiusSquared, maxRadiusSquared;
};

#endif /* SRC_MOVEMENT_KINEMATICS_POLARKINEMATICS_H_ */
