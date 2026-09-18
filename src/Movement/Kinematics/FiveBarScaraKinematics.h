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

#if SUPPORT_FIVEBARSCARA

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
	MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;
	LimitPositionResult LimitPosition(float coords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualDrives; }
	float GetEndstopPosition(size_t drive, bool highEnd) noexcept override;
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	LogicalDrivesBitmap GetControllingDrives(size_t axis, bool forHoming) const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
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
	float xOrigL = 0.0;
	float yOrigL = 0.0;
	float xOrigR = 0.0;
	float yOrigR = 0.0;
	float proximalL = 0.0;
	float proximalR = 0.0;
	float distalL = 0.0;
	float distalR = 0.0;
	float cantL = 0.0;
	float cantR = 0.0;
	int workmode = 1;
	float homingAngleL = 0.0;
	float homingAngleR = 0.0;

	bool printAreaDefined = false;
	float printArea[4] = { 0.0, 0.0, 0.0, 0.0 };	// x1, y1, x2, y2

	float headAngleMin = 0.0;
	float headAngleMax = 0.0;
	float proxDistLAngleMin = 0.0;
	float proxDistLAngleMax = 0.0;
	float proxDistRAngleMin = 0.0;
	float proxDistRAngleMax = 0.0;
	float actuatorAngleLMin = 0.0;
	float actuatorAngleLMax = 0.0;
	float actuatorAngleRMin = 0.0;
	float actuatorAngleRMax = 0.0;

	// Derived parameters

	// State variables
	mutable float cachedX0, cachedY0;
	mutable float cachedThetaL, cachedThetaR;
	mutable float cachedXL, cachedXR;
	mutable float cachedYL, cachedYR;
	mutable float cachedX1, cachedY1;
	mutable bool cachedInvalid;
};

#endif // SUPPORT_FIVEBARSCARA

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_ */
