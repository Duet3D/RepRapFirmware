/*
 * FiveBarScaraKinematics.h
 *
 *  Created on: 11 Nov 2018
 *      Author: Joerg
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
	FiveBarScaraKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const override;
	bool IsReachable(float x, float y, bool isCoordinated) const override;
	bool LimitPosition(float position[], size_t numAxes, AxesBitmap axesHomed, bool isCoordinated) const override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const override;
	size_t NumHomingButtons(size_t numVisibleAxes) const override;
	const char* HomingButtonNames() const override { return "PDZUVWABC"; }
	HomingMode GetHomingMode() const override { return homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const override;
	bool QueryTerminateHomingMove(size_t axis) const override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const override;
	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const override;
	bool IsContinuousRotationAxis(size_t axis) const override;

private:
	static constexpr float DefaultSegmentsPerSecond = 100.0;
	static constexpr float DefaultMinSegmentSize = 0.2;

	static constexpr const char *Home5BarScaraFileName = "home5barscara.g";
	
	void Recalc();
	int getNumParameters(char c, GCodeBuffer gb) const;
	int getQuadrant(int x, int y) const;
	bool isCantilevered(int mode) const;
	float getAbsoluteAngle(float xOrig, float yOrig, float xDest, float yDest) const;
	float * getIntersec(float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const;
	float * getTheta(float proximal, float distal, float proxX, float proxY, float destX, float destY, Arm arm) const;
	float * getXYFromAngle(float angle, float length, float origX, float origY) const;
	float * getForward(float thetaL, float thetaR) const;
	float * getInverse(float x0, float y0) const;
	float getAngle(float x1, float y1, float xAngle, float yAngle, float x2, float y2) const;
	bool isPointInsideDefinedPrintableArea(float x0, float y0) const;
	bool constraintsOk(float x0, float y0) const;

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

	float constrMin;
	float constrMax;
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
};

#endif /* SRC_MOVEMENT_KINEMATICS_FIVEBARSCARAKINEMATICS_H_ */
