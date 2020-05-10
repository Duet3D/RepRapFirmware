/*
 * HangprinterKinematics.h
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_

#include "Kinematics.h"

class HangprinterKinematics : public Kinematics
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
	bool IsReachable(float x, float y, bool isCoordinated) const noexcept override;
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;
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
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(anchorA)
	OBJECT_MODEL_ARRAY(anchorB)
	OBJECT_MODEL_ARRAY(anchorC)

private:
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

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

	floatc_t ComputeDerivative(unsigned int deriv, float La, float Lb, float Lc) const noexcept;	// Compute the derivative of height with respect to a parameter at a set of motor endpoints
	void Adjust(size_t numFactors, const floatc_t v[]) noexcept;									// Perform 3-, 6- or 9-factor adjustment
	void PrintParameters(const StringRef& reply) const noexcept;									// Print all the parameters for debugging

	float anchorA[3], anchorB[3], anchorC[3];				// XYZ coordinates of the anchors
	float anchorDz;
	float printRadius;

	// Derived parameters
	float printRadiusSquared;
	float Da2, Db2, Dc2;
	float Xab, Xbc, Xca;
	float Yab, Ybc, Yca;
	float Zab, Zbc, Zca;
	float P, Q, R, P2, U, A;

	bool doneAutoCalibration;							// True if we have done auto calibration
};

#endif /* SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_ */
