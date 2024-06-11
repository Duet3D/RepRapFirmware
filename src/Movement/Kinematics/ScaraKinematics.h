/*
 * ScaraKinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_

#include "ZLeadscrewKinematics.h"

#if SUPPORT_SCARA

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
	ScaraKinematics() noexcept;

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
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	AxesBitmap GetControllingDrives(size_t axis, bool forHoming) const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultProximalArmLength = 100.0;
	static constexpr float DefaultDistalArmLength = 100.0;
	static constexpr float DefaultMinTheta = -90.0;					// minimum proximal joint angle
	static constexpr float DefaultMaxTheta = 90.0;					// maximum proximal joint angle
	static constexpr float DefaultMinPsi = -135.0;					// minimum distal joint angle
	static constexpr float DefaultMaxPsi = 135.0;					// maximum distal joint angle

	static constexpr const char *HomeProximalFileName = "homeproximal.g";
	static constexpr const char *HomeDistalFileName = "homedistal.g";

	void Recalc() noexcept;
	bool CalculateThetaAndPsi(const float machinePos[], bool isCoordinated, float& theta, float& psi, bool& armMode) const noexcept;

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
	float minRadiusSquared;
	float proximalArmLengthSquared;
	float distalArmLengthSquared;
	float twoPd;

	// State variables
	mutable float cachedX, cachedY, cachedTheta, cachedPsi;
	mutable bool currentArmMode, cachedArmMode;
};

#endif // SUPPORT_SCARA

#endif /* SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_ */
