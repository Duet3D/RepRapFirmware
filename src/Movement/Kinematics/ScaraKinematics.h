/*
 * ScaraKinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_

#include "Kinematics.h"

// Standard setup for SCARA machines assumed by this firmware
// The X motor output drives the proximal arm joint, unless remapped using M584
// The Y motor output drives the distal arm joint, unless remapped using M584
// Forward motion of a motor rotates the arm anticlockwise as seen from above (use M569 to reverse it)
// Theta is the angle of the proximal arm joint from the reference position.
// At theta = 0 the proximal arm points in the Cartesian +X direction
// Phi is the angle of the distal arm relative to the Cartesian X axis. Therefore the angle of the distal arm joint is (phi - theta).
// The M92 steps/mm settings for X and Y are interpreted as steps per degree of theta and phi respectively.

class ScaraKinematics : public Kinematics
{
public:
	// Constructors
	ScaraKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const override;
	bool SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numAxes, int32_t motorPos[]) const override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numDrives, float machinePos[]) const override;
	bool SupportsAutoCalibration() const override { return false; }
	bool IsReachable(float x, float y) const override;
	void LimitPosition(float position[], size_t numAxes, uint16_t axesHomed) const override;
	bool ShowCoordinatesWhenNotHomed() const override { return false; }

private:
	const float DefaultSegmentsPerSecond = 200.0;
	const float DefaultMinSegmentSize = 0.2;
	const float DefaultProximalArmLength = 100.0;
	const float DefaultDistalArmLength = 100.0;
	const float DefaultMinTheta = -180.0;								// minimum proximal joint angle
	const float DefaultMaxTheta = 180.0;								// maximum proximal joint angle
	const float DefaultMinPhiMinusTheta = -270.0;						// minimum distal joint angle
	const float DefaultMaxPhiMinusTheta = 270.0;						// maximum distal joint angle

	void Recalc();

	// Primary parameters
	float proximalArmLength;
	float distalArmLength;
	float xToYcrosstalk;						// if we rotate the proximal arm, the distal arm also rotates by this fraction of it
	float crosstalk[3];							// if we rotate the distal arm motor, for each full rotation the Z height goes up by this amount
	float thetaLimits[2];						// minimum proximal joint angle
	float phiMinusThetaLimits[2];				// minimum distal joint angle

	// Derived parameters
	float minRadius;
	float maxRadius;
	float proximalArmLengthSquared;
	float distalArmLengthSquared;

	// State variables
	mutable bool isDefaultArmMode;					// this should be moved into class Move when it knows about different arm modes
};

#endif /* SRC_MOVEMENT_KINEMATICS_SCARAKINEMATICS_H_ */
