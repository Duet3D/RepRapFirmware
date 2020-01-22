/*
 * RotaryDeltaKinematics.h
 *
 *  Created on: 1 Aug 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ROTARYDELTAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ROTARYDELTAKINEMATICS_H_

#include "Kinematics.h"

class RotaryDeltaKinematics : public Kinematics
{
public:
	// Constructors
	RotaryDeltaKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS_GCODE_EXCEPTION override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool SupportsAutoCalibration() const noexcept override { return false; }		// TODO support autocalibration
	bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;
	void SetCalibrationDefaults() noexcept override { Init(); }
#if HAS_MASS_STORAGE
	bool WriteCalibrationParameters(FileStore *f) const noexcept override;
#endif
	bool IsReachable(float x, float y, bool isCoordinated) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	AxesBitmap AxesToHomeBeforeProbing() const noexcept override { return XyzAxes; }
	size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; }
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
#if HAS_MASS_STORAGE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	void Init() noexcept;
	void Recalc() noexcept;
    float Transform(const float headPos[], size_t axis) const noexcept;						// Calculate the motor position for a single tower from a Cartesian coordinate
    void ForwardTransform(float Ha, float Hb, float Hc, float headPos[]) const noexcept;	// Calculate the Cartesian position from the motor positions

	// Axis names used internally
	static constexpr size_t DELTA_AXES = 3;
	static constexpr size_t DELTA_A_AXIS = 0;
	static constexpr size_t DELTA_B_AXIS = 1;
	static constexpr size_t DELTA_C_AXIS = 2;

	// Delta mechanism parameter defaults
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;
	static constexpr float DefaultArmLength = 100.0;
	static constexpr float DefaultRodLength = 200.0;
	static constexpr float DefaultDeltaRadius = 50.0;
	static constexpr float DefaultPrintRadius = 80.0;
	static constexpr float DefaultBearingHeight = 250.0;
	static constexpr float DefaultMinArmAngle = -45.0;
	static constexpr float DefaultMaxArmAngle = 45.0;

	static const float NormalTowerAngles[DELTA_AXES];

	// Core parameters
	float radius;										// The nominal delta radius, before any fine tuning of tower positions
	float bearingHeights[DELTA_AXES];
	float armLengths[DELTA_AXES];
	float rodLengths[DELTA_AXES];
	float angleCorrections[DELTA_AXES];					// Tower position corrections
	float endstopAdjustments[DELTA_AXES];				// How much above or below the ideal position each endstop is
	float minMaxArmAngles[2];
	float& minArmAngle = minMaxArmAngles[0];
	float& maxArmAngle = minMaxArmAngles[1];
	float printRadius;

	// Derived values
	float armAngleCosines[DELTA_AXES];
	float armAngleSines[DELTA_AXES];
	float twiceU[DELTA_AXES];
	float rodSquared[DELTA_AXES];
	float rodSquaredMinusArmSquared[DELTA_AXES];
	float printRadiusSquared;
};

#endif /* SRC_MOVEMENT_KINEMATICS_ROTARYDELTAKINEMATICS_H_ */
