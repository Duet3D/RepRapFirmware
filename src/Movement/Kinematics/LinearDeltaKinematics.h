/*
 * DeltaParameters.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef LINEARDELTAKINEMATICS_H_
#define LINEARDELTAKINEMATICS_H_

#include "RepRapFirmware.h"

#if SUPPORT_LINEAR_DELTA

#include "RoundBedKinematics.h"

// Class to hold the parameter for a delta machine.
class LinearDeltaKinematics : public RoundBedKinematics
{
public:
	// Constructors
	LinearDeltaKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool SupportsAutoCalibration() const noexcept override { return true; }
	bool DoAutoCalibration(MovementSystemNumber msNumber, size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;
	void SetCalibrationDefaults() noexcept override { Init(); }

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteCalibrationParameters(FileStore *f) const noexcept override;
#endif

	float GetTiltCorrection(size_t axis) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	AxesBitmap AxesToHomeBeforeProbing() const noexcept override { return XyzAxes; }
	MotionType GetMotionType(size_t axis) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif

	AxesBitmap GetLinearAxes() const noexcept override;

    // Public functions specific to this class
	float GetDiagonalSquared(size_t tower) const noexcept { return D2[tower]; }
    float GetTowerX(size_t axis) const noexcept { return towerX[axis]; }
    float GetTowerY(size_t axis) const noexcept { return towerY[axis]; }

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	void Init() noexcept;
	void Recalc() noexcept;
	void NormaliseEndstopAdjustments() noexcept;													// Make the average of the endstop adjustments zero
    float Transform(const float headPos[], size_t axis) const noexcept;								// Calculate the motor position for a single tower from a Cartesian coordinate
    void ForwardTransform(float Ha, float Hb, float Hc, float headPos[XYZ_AXES]) const noexcept;	// Calculate the Cartesian position from the motor positions

	floatc_t ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const noexcept;	// Compute the derivative of height with respect to a parameter at a set of motor endpoints
	void Adjust(size_t numFactors, const floatc_t v[]) noexcept;									// Perform 3-, 4-, 6- or 7-factor adjustment
	void PrintParameters(const StringRef& reply) const noexcept;									// Print all the parameters for debugging

	static constexpr size_t MaxTowers = 6;				// maximum number of delta towers
	static constexpr size_t UsualNumTowers = 3;			// the usual number of towers, which are the ones we use for forward kinematics and the ones we calibrate

	// Axis names used internally
	static constexpr size_t DELTA_A_AXIS = 0;
	static constexpr size_t DELTA_B_AXIS = 1;
	static constexpr size_t DELTA_C_AXIS = 2;

	// Delta parameter defaults in mm
	static constexpr float DefaultDiagonal = 215.0;
	static constexpr float DefaultDeltaRadius = 105.6;
	static constexpr float DefaultPrintRadius = 80.0;
	static constexpr float DefaultDeltaHomedHeight = 240.0;

	// Core parameters
	size_t numTowers;
	float diagonals[MaxTowers];							// The diagonal rod lengths
	float radius;										// The nominal delta radius, before any fine tuning of tower positions
	float angleCorrections[UsualNumTowers];				// Tower position corrections for the first 3 axes
	float endstopAdjustments[MaxTowers];				// How much above or below the ideal position each endstop is
	float printRadius;
	float homedHeight;
	float xTilt, yTilt;									// How much we need to raise Z for each unit of movement in the +X and +Y directions

	// Derived values
	float towerX[MaxTowers];							// The X coordinate of each tower
	float towerY[MaxTowers];							// The Y coordinate of each tower
	float homedCarriageHeights[MaxTowers];
	float Xbc, Xca, Xab, Ybc, Yca, Yab;
	float coreKa, coreKb, coreKc;
	float Q, Q2;
	float D2[MaxTowers];
	float alwaysReachableHeight;

	bool doneAutoCalibration;							// True if we have done auto calibration
};

#endif	// SUPPORT_LINEAR_DELTA

#endif /* LINEARDELTAKINEMATICS_H_ */
