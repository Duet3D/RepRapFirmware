/*
 * DeltaParameters.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef LINEARDELTAKINEMATICS_H_
#define LINEARDELTAKINEMATICS_H_

#include "RepRapFirmware.h"
#include "Kinematics.h"

const size_t DELTA_AXES = 3;
const size_t A_AXIS = 0;
const size_t B_AXIS = 1;
const size_t C_AXIS = 2;

// Class to hold the parameter for a delta machine.
class LinearDeltaKinematics : public Kinematics
{
public:
	// Constructors
	LinearDeltaKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[]) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool SupportsAutoCalibration() const override { return true; }
	void DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, StringRef& reply) override;
	void SetCalibrationDefaults() override { Init(); }
	bool WriteCalibrationParameters(FileStore *f) const override;
	float GetTiltCorrection(size_t axis) const override;
	bool IsReachable(float x, float y) const override;
	bool LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed) const override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const override;
	AxesBitmap AxesToHomeBeforeProbing() const override { return MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS); }
	MotionType GetMotionType(size_t axis) const override;
	size_t NumHomingButtons(size_t numVisibleAxes) const override { return 0; }
	bool DriveIsShared(size_t drive) const override { return false; }
	HomingMode GetHomingMode() const override { return homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const override;

#ifdef DUET_NG
	bool WriteResumeSettings(FileStore *f) const override;
#endif

    // Public functions specific to this class
	float GetDiagonalSquared() const { return D2; }
    float GetTowerX(size_t axis) const { return towerX[axis]; }
    float GetTowerY(size_t axis) const { return towerY[axis]; }
    float GetHomedCarriageHeight(size_t axis) const { return homedCarriageHeight + endstopAdjustments[axis]; }
	float GetHomedHeight() const { return homedHeight; }

private:
	void Init();
	void Recalc();
	void NormaliseEndstopAdjustments();												// Make the average of the endstop adjustments zero
    float Transform(const float headPos[], size_t axis) const;						// Calculate the motor position for a single tower from a Cartesian coordinate
    void InverseTransform(float Ha, float Hb, float Hc, float headPos[]) const;		// Calculate the Cartesian position from the motor positions

	floatc_t ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const;	// Compute the derivative of height with respect to a parameter at a set of motor endpoints
	void Adjust(size_t numFactors, const floatc_t v[]);								// Perform 3-, 4-, 6- or 7-factor adjustment
	void PrintParameters(StringRef& reply) const;									// Print all the parameters for debugging

	// Delta parameter defaults
	const float defaultDiagonal = 215.0;
	const float defaultDeltaRadius = 105.6;
	const float defaultPrintRadius = 80.0;				// mm
	const float defaultDeltaHomedHeight = 240.0;		// mm

	// Core parameters
    float diagonal;										// The diagonal rod length, all 3 are assumed to be the same length
    float radius;										// The nominal delta radius, before any fine tuning of tower positions
    float angleCorrections[DELTA_AXES];					// Tower position corrections
    float endstopAdjustments[DELTA_AXES];				// How much above or below the ideal position each endstop is
    float printRadius;
    float homedHeight;
    float xTilt, yTilt;									// How much we need to raise Z for each unit of movement in the +X and +Y directions

    // Derived values
    float towerX[DELTA_AXES];							// The X coordinate of each tower
    float towerY[DELTA_AXES];							// The Y coordinate of each tower
    float printRadiusSquared;
    float homedCarriageHeight;
	float Xbc, Xca, Xab, Ybc, Yca, Yab;
	float coreFa, coreFb, coreFc;
    float Q, Q2, D2;
    bool doneAutoCalibration;							// True if we have done auto calibration
};

#endif /* LINEARDELTAKINEMATICS_H_ */
