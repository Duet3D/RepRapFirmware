/*
 * ScaraKinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_

#include "ZLeadscrewKinematics.h"

// Standard setup for SCARA machines assumed by this firmware
// The X motor output drives the proximal arm joint, unless remapped using M584
// The Y motor output drives the distal arm joint, unless remapped using M584
// Forward motion of a motor rotates the arm anticlockwise as seen from above (use M569 to reverse it)
// Theta is the angle of the proximal arm joint from the reference position.
// At theta = 0 the proximal arm points in the Cartesian +X direction
// Phi is the angle of the distal arm relative to the Cartesian X axis. Therefore the angle of the distal arm joint is (phi - theta).
// The M92 steps/mm settings for X and Y are interpreted as steps per degree of theta and phi respectively.

class ScaraKinematics : public ZLeadscrewKinematics
{
public:
	// Constructors
	ScaraKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool IsReachable(float x, float y, bool isCoordinated) const override;
	bool IntermediatePositionsReachable(const float initialCoords[], const float finalCoords[], float margin) const override;
	bool LimitPosition(float finalCoords[], float * null initialCoords, size_t numAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const override;
	size_t NumHomingButtons(size_t numVisibleAxes) const override;
	const char* HomingButtonNames() const override { return "PDZUVWABC"; }
	HomingMode GetHomingMode() const override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const override;
	bool QueryTerminateHomingMove(size_t axis) const override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const override;
	bool IsContinuousRotationAxis(size_t axis) const override;
	AxesBitmap GetLinearAxes() const override;

private:
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;
	static constexpr float DefaultProximalArmLength = 100.0;
	static constexpr float DefaultDistalArmLength = 100.0;
	static constexpr float DefaultMinTheta = -90.0;					// minimum proximal joint angle
	static constexpr float DefaultMaxTheta = 90.0;					// maximum proximal joint angle
	static constexpr float DefaultMinPsi = -135.0;					// minimum distal joint angle
	static constexpr float DefaultMaxPsi = 135.0;					// maximum distal joint angle

	static constexpr const char *HomeProximalFileName = "homeproximal.g";
	static constexpr const char *HomeDistalFileName = "homedistal.g";

	void Recalc();
	bool CalculateThetaAndPsi(const float machinePos[], bool isCoordinated, float& theta, float& psi, bool& armMode) const;

	// Primary parameters
	float proximalArmLength;
	float distalArmLength;
	float thetaLimits[2];							// minimum proximal joint angle
	float psiLimits[2];								// minimum distal joint angle
	float crosstalk[3];								// proximal to distal, proximal to Z and distal to Z crosstalk
	float xOffset;									// where bed X=0 is relative to the proximal joint
	float yOffset;									// where bed Y=0 is relative to the proximal joint
	float requestedMinRadius;						// requested minimum radius
	bool supportsContinuousRotation[2];				// true if the (proximal, distal) arms support continuous rotation

	// Derived parameters
	float minRadius;
	float maxRadius;
	float proximalArmLengthSquared;
	float distalArmLengthSquared;
	float twoPd;

	// State variables
	mutable float cachedX, cachedY, cachedTheta, cachedPsi;
	mutable bool currentArmMode, cachedArmMode;
};

#endif /* SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_ */
