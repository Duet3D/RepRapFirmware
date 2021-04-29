/*
 * HangprinterKinematics.h
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_

#include "RoundBedKinematics.h"

class HangprinterKinematics : public RoundBedKinematics
{
public:
	// Constructors
	HangprinterKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool SupportsAutoCalibration() const noexcept override { return true; }
	bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;
	void SetCalibrationDefaults() noexcept override { Init(); }
#if HAS_MASS_STORAGE
	bool WriteCalibrationParameters(FileStore *f) const noexcept override;
#endif
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	size_t NumHomingButtons(size_t numVisibleAxes) const noexcept override { return 0; }
	const char* HomingButtonNames() const noexcept override { return "ABCD"; }
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
#if HAS_MASS_STORAGE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(anchors)
	OBJECT_MODEL_ARRAY(anchorCoordinates)

private:
	// Basic facts about movement system
	static constexpr size_t HANGPRINTER_AXES = 4;
	static constexpr size_t A_AXIS = 0;
	static constexpr size_t B_AXIS = 1;
	static constexpr size_t C_AXIS = 2;
	static constexpr size_t D_AXIS = 3;

	void Init() noexcept;
	void Recalc() noexcept;
	float LineLengthSquared(const float machinePos[3], const float anchor[3]) const noexcept;		// Calculate the square of the line length from a spool from a Cartesian coordinate
	void InverseTransform(float La, float Lb, float Lc, float machinePos[3]) const noexcept;
	float MotorPosToLinePos(const int32_t motorPos, size_t axis) const noexcept;

	floatc_t ComputeDerivative(unsigned int deriv, float La, float Lb, float Lc) const noexcept;	// Compute the derivative of height with respect to a parameter at a set of motor endpoints
	void Adjust(size_t numFactors, const floatc_t v[]) noexcept;									// Perform 3-, 6- or 9-factor adjustment
	void PrintParameters(const StringRef& reply) const noexcept;									// Print all the parameters for debugging

	float anchors[HANGPRINTER_AXES][3];				// XYZ coordinates of the anchors
	float printRadius;
	// Line buildup compensation
	float spoolBuildupFactor;
	float spoolRadii[HANGPRINTER_AXES];
	uint32_t mechanicalAdvantage[HANGPRINTER_AXES], linesPerSpool[HANGPRINTER_AXES];
	uint32_t motorGearTeeth[HANGPRINTER_AXES], spoolGearTeeth[HANGPRINTER_AXES], fullStepsPerMotorRev[HANGPRINTER_AXES];

	// Derived parameters
	float k0[HANGPRINTER_AXES], spoolRadiiSq[HANGPRINTER_AXES], k2[HANGPRINTER_AXES], lineLengthsOrigin[HANGPRINTER_AXES];
	float printRadiusSquared;
	float Da2, Db2, Dc2;
	float Xab, Xbc, Xca;
	float Yab, Ybc, Yca;
	float Zab, Zbc, Zca;
	float P, Q, R, P2, U, A;

	bool doneAutoCalibration;							// True if we have done auto calibration
};

#endif /* SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_ */
