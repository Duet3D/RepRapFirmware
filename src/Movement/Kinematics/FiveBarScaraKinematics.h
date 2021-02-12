/*
 * FiveBarScaraKinematics.h
 *
 *  Created on: 11 Nov 2018
 *      Author: JoergS5, bondus
 *
 *	documentation: https://duet3d.dozuki.com/Guide/Five+Bar+Parallel+SCARA/24?lang=en
 */

#ifndef SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_

#include "ZLeadscrewKinematics.h"

// Standard setup for 5 Bar SCARA (parallel SCARA) machines assumed by this firmware
enum class Arm : uint8_t
{
	left,
	right
};

class FiveBarScaraKinematics : public ZLeadscrewKinematics
{
public:
	// Constructors
	FiveBarScaraKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes, bool isCoordinated) const noexcept override;
	LimitPositionResult LimitPosition(float coords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	const char* HomingButtonNames() const noexcept override { return "PDZUVWABC"; }
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept;
	AxesBitmap GetConnectedAxes(size_t axis) const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

	static constexpr const char *Home5BarScaraFileName = "home5barscara.g";

	void Recalc() noexcept;
	int getQuadrant(float x, float y) const noexcept;
	bool isCantilevered(int mode) const noexcept;
	float getAbsoluteAngle(float xOrig, float yOrig, float xDest, float yDest) const noexcept;
	void getIntersec(float result12[], float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const noexcept;
	void getTheta(float result[], float proximal, float distal, float proxX, float proxY, float destX, float destY, Arm arm) const noexcept;
	void getXYFromAngle(float resultcoords[], float angle, float length, float origX, float origY) const noexcept;
	void getForward(float resultcoords[], float thetaL, float thetaR) const noexcept;
	void getInverse(const float coords[]) const noexcept;
	float getAngle(float x1, float y1, float xAngle, float yAngle, float x2, float y2) const noexcept;
    float getTurn(float x1, float y1, float x2, float y2, float x3, float y3) const noexcept;
	bool isPointInsideDefinedPrintableArea(float x0, float y0) const noexcept;
	bool constraintsOk(const float coords[]) const noexcept;

	// Primary parameters
	float xOrigL;
	float yOrigL;
	float xOrigR;
	float yOrigR;
	float proximalL;
	float proximalR;
	float distalL;
	float distalR;
	float cantL;
	float cantR;
	int workmode;
	float homingAngleL;
	float homingAngleR;

	bool printAreaDefined;
	float printArea[4];	// x1, y1, x2, y2

	float headAngleMin;
	float headAngleMax;
	float proxDistLAngleMin;
	float proxDistLAngleMax;
	float proxDistRAngleMin;
	float proxDistRAngleMax;
	float actuatorAngleLMin;
	float actuatorAngleLMax;
	float actuatorAngleRMin;
	float actuatorAngleRMax;

	// Derived parameters

	// State variables
	mutable float cachedX0, cachedY0;
	mutable float cachedThetaL, cachedThetaR;
	mutable float cachedXL, cachedXR;
	mutable float cachedYL, cachedYR;
	mutable float cachedX1, cachedY1;
	mutable bool cachedInvalid;
};

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_ */
