/*
 * RotaryDeltaKinematics.h
 *
 *  Created on: 1 Aug 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ROTARYDELTAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ROTARYDELTAKINEMATICS_H_

#include "RoundBedKinematics.h"

class RotaryDeltaKinematics : public RoundBedKinematics
{
public:
	// Constructors
	RotaryDeltaKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool SupportsAutoCalibration() const noexcept override { return true; }
	bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;
	void SetCalibrationDefaults() noexcept override { Init(); }
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteCalibrationParameters(FileStore *f) const noexcept override;
#endif
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	AxesBitmap AxesToHomeBeforeProbing() const noexcept override { return XyzAxes; }
	size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; }
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif

protected:
	DECLARE_OBJECT_MODEL

private:
	void Init() noexcept;
	void Recalc() noexcept;
    float Transform(const float headPos[], size_t axis) const noexcept;								// Calculate the motor position for a single tower from a Cartesian coordinate
    void ForwardTransform(float Ha, float Hb, float Hc, float headPos[]) const noexcept;			// Calculate the Cartesian position from the motor positions

    floatc_t ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const noexcept;	// Compute the derivative of height with respect to a parameter at a set of motor endpoints
	void Adjust(size_t numFactors, const floatc_t v[]) noexcept;									// Perform 3-, 4- or 6-factor adjustment
	void PrintParameters(const StringRef& reply) const noexcept;									// Print all the parameters for debugging

	// Axis names used internally
	static constexpr size_t DELTA_AXES = 3;
	static constexpr size_t DELTA_A_AXIS = 0;
	static constexpr size_t DELTA_B_AXIS = 1;
	static constexpr size_t DELTA_C_AXIS = 2;

	// Delta mechanism parameter defaults
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

	bool doneAutoCalibration;							// True if we have done auto calibration
};

#endif /* SRC_MOVEMENT_KINEMATICS_ROTARYDELTAKINEMATICS_H_ */
