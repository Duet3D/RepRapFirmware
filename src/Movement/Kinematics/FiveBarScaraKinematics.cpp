/*
 * FiveBarScaraKinematics.cpp
 *
 *  Created on: 11 Nov 2018
 *      Author: Joerg
 *
 *	documentation: https://duet3d.dozuki.com/Guide/Five+Bar+Parallel+SCARA/24?lang=en
 */

#include "FiveBarScaraKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "Storage/MassStorage.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/DDA.h"

#include <limits>
#include <algorithm>

FiveBarScaraKinematics::FiveBarScaraKinematics()
	: ZLeadscrewKinematics(KinematicsType::scara, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true)
{
	Recalc();
}

// Return the name of the current kinematics
const char *FiveBarScaraKinematics::GetName(bool forStatusReport) const
{
	return "FiveBarScara";
}

//////////////////////// private functions /////////////////////////


float * FiveBarScaraKinematics::getInverse(float x_0, float y_0) const
{
	static float result[8];	// xL, yL, thetaL, xR, yR, thetaR, x1, y1 (joint of distals)

	float thetaL = -1.0;
	float thetaR = -1.0;
	float xL, xR, yL, yR, x1, y1;

	if(isCantilevered(1)) {
		// calculate cantilevered side first:
		float *theta = getTheta(proximalL, distalL + cantL, xOrigL, yOrigL, x_0, y_0);
		xL = theta[0];
		yL = theta[1];
		thetaL = theta[2];

		if(workmode == 1) {
			if(theta[0] < theta[3]) {
				xL = theta[3];
				yL = theta[4];
				thetaL = theta[5];
			}
		}
		else if(workmode == 2 || workmode == 4) {
			if(theta[0] > theta[3]) {
				xL = theta[3];
				yL = theta[4];
				thetaL = theta[5];
			}
		}
		else {
			// unsupported workmode => TODO error log or exception
		}

		// calculate x1,y1, i.e. where the distal arms meet
		float fraction = distalL / (distalL + cantL);
		x1 = (x_0 - xL) * fraction + xL;
		y1 = (y_0 - yL) * fraction + yL;

		// calculate right, non cantilevered side:
		theta = getTheta(proximalR, distalR, xOrigR, yOrigR, x1, y1);
		xR = theta[0];
		yR = theta[1];
		thetaR = theta[2];

		if(workmode == 1 || workmode == 2) {
			if(theta[0] < theta[3]) {
				xR = theta[3];
				yR = theta[4];
				thetaR = theta[5];
			}
		}
		else if(workmode == 4) {
			if(theta[0] > theta[3]) {
				xR = theta[3];
				yR = theta[4];
				thetaR = theta[5];
			}
		}
		else {
			// unsupported workmode => TODO error log or exception
		}
	}
	else if(isCantilevered(2)) {
		// calculate cantilevered side first:
		float *theta = getTheta(proximalR, distalR + cantR, xOrigR, yOrigR, x_0, y_0);
		xR = theta[0];
		yR = theta[1];
		thetaR = theta[2];

		if(workmode == 1 || workmode == 2) {
			if(theta[0] < theta[3]) {
				xR = theta[3];
				yR = theta[4];
				thetaR = theta[5];
			}
		}
		else if(workmode == 4) {
			if(theta[0] > theta[3]) {
				xR = theta[3];
				yR = theta[4];
				thetaR = theta[5];
			}
		}
		else {
			// unsupported workmode => TODO error log or exception
		}

		// calculate x1,y1, i.e. where the distal arms meet
		float fraction = distalR / (distalR + cantR);
		x1 = (x_0 - xR) * fraction + xR;
		y1 = (y_0 - yR) * fraction + yR;

		// calculate left, non cantilevered side:
		theta = getTheta(proximalL, distalL, xOrigL, yOrigL, x1, y1);
		xL = theta[0];
		yL = theta[1];
		thetaL = theta[2];

		if(workmode == 1) {
			if(theta[0] < theta[3]) {
				xL = theta[3];
				yL = theta[4];
				thetaL = theta[5];
			}
		}
		else if(workmode == 2 || workmode == 4) {
			if(theta[0] > theta[3]) {
				xL = theta[3];
				yL = theta[4];
				thetaL = theta[5];
			}
		}
		else {
			// unsupported workmode => TODO error log or exception
		}
	}
	else {	// not cantilevered, hotend is at top joint
		float *theta = getTheta(proximalL, distalL, xOrigL, yOrigL, x_0, y_0);
		x1 = x_0;
		y1 = y_0;
		float thetaL1 = theta[2];
		float thetaL2 = theta[5];
		if(workmode == 1) {
			if(thetaL1 <= thetaL2) {
				thetaL = thetaL1;
			}
			else {
				thetaL = thetaL2;
			}
		}

		theta = getTheta(proximalR, distalR, xOrigR, yOrigR, x_0, y_0);
		float thetaR1 = theta[2];
		float thetaR2 = theta[5];
		if(workmode == 1) {
			if(thetaR1 <= thetaR2) {
				thetaR = thetaR1;
			}
			else {
				thetaR = thetaR2;
			}
		}
	}

	// xL, yL, thetaL, xR, yR, thetaR, x1, y1 (joint of distals)
	result[0] = xL;
	result[1] = yL;
	result[2] = thetaL;
	result[3] = xR;
	result[4] = yR;
	result[5] = thetaR;
	result[6] = x1;
	result[7] = y1;

	return result;
}

int FiveBarScaraKinematics::getNumParameters(char c, GCodeBuffer gb) const {
	String<MaxFilenameLength> str;
	bool seen = false;
	gb.TryGetPossiblyQuotedString(c, str.GetRef(), seen);

	int count = 0;
	size_t size = str.strlen();
	for (size_t i = 0; i < size; i++) {
		if (str[i] == ':') count++;
	}

	return count + 1;
}

// quadrants: 1 is right upper, 2 is left upper, 3 is left down, 4 is right down
int FiveBarScaraKinematics::getQuadrant(int x, int y) const
{
	if(x >= 0 && y >= 0) {
		return 1;
	}
	else if(x < 0 && y >= 0) {
		return 2;
	}
	else if(x < 0 && y < 0) {
		return 3;
	}
	else { // x >= 0 && y < 0
		return 4;
	}
}


bool FiveBarScaraKinematics::isCantilevered(int mode) const
{
	return true; // TODO implement
}

float FiveBarScaraKinematics::getAbsoluteAngle(float xOrig, float yOrig, float xDest, float yDest) const
{
	float length = sqrt(pow(xOrig - xDest, 2) + pow(yOrig - yDest,2));

	float y = abs(yOrig - yDest);
	float angle = asin(y / length) * 180.0 / M_PI;

	int quad = getQuadrant(xDest-xOrig, yDest-yOrig);

	if(quad == 1) {  // right upper quadrant I
		// nothing to change
	}
	else if(quad == 2) {  // left upper quadrant II
		angle = 180.0 - angle;
	}
	else if(quad == 3) { // left lower quadrant III
		angle += 180.0;
	}
	else if(quad == 4) { // right lower quadrant IV
		angle = 360.0 - angle;
	}
	return angle;
}

// first circle, second circle. Return the two intersection points
float * FiveBarScaraKinematics::getIntersec(float firstRadius, float secondRadius, float firstX, float firstY,
		float secondX, float secondY) const
{
	static float result[4];	// x,y of first point, then x,y of second

	float distance = sqrt(pow((firstX - secondX),2) + pow((firstY - secondY),2));
	float delta = 0.25 * sqrt(
			(distance + firstRadius + secondRadius)
			* (distance + firstRadius - secondRadius)
			* (distance - firstRadius + secondRadius)
			* (-distance + firstRadius + secondRadius)
		);
	// calculate x
	float term1x = (firstX + secondX) / 2;
	float term2x = (secondX - firstX)*(pow(firstRadius,2) - pow(secondRadius,2)) / (2 * pow(distance,2));
	float term3x = 2 * (firstY - secondY) / pow(distance,2) * delta;
	float x1 = term1x + term2x + term3x;
	float x2 = term1x + term2x - term3x;

	// calculate y
	float term1y = (firstY + secondY) / 2;
	float term2y = (secondY - firstY)*(pow(firstRadius,2) - pow(secondRadius,2)) / (2 * pow(distance,2));
	float term3y = 2 * (firstX - secondX) / pow(distance,2) * delta;
	float y1 = term1y + term2y - term3y;
	float y2 = term1y + term2y + term3y;

	result[0] = x1;
	result[1] = y1;
	result[2] = x2;
	result[3] = y2;

	return result;
}


float * FiveBarScaraKinematics::getTheta(float prox, float distal, float proxX, float proxY, float destX, float destY) const
{
	static float result[6];	// x,y,theta of first point, then x,y,theta of second solution

	float *xyArr = getIntersec(prox, distal, proxX, proxY, destX, destY);
	float x1 = xyArr[0];
	float y1 = xyArr[1];
	float x2 = xyArr[2];
	float y2 = xyArr[3];

	float thetaA = getAbsoluteAngle(proxX, proxY, x1, y1);
	float thetaB = getAbsoluteAngle(proxX, proxY, x2, y2);

	result[0] = x1;
	result[1] = y1;
	result[2] = thetaA;
	result[3] = x2;
	result[4] = y2;
	result[5] = thetaB;

	return result;
}

// from given point with angle and length, calculate destination
float * FiveBarScaraKinematics::getXYFromAngle(float angle, float length, float origX, float origY) const
{
	static float result[2];	// x,y

	float xL = length * cos(angle * M_PI / 180.0);
	float yL = length * sin(angle * M_PI / 180.0);

	result[0] = xL + origX;
	result[1] = yL + origY;

	return result;
}

float * FiveBarScaraKinematics::getForward(float thetaL, float thetaR, int workmode) const
{
	static float result[6];	// xL, yL, xR, yR, x0, y0

	float *xy1 = getXYFromAngle(thetaL, proximalL, xOrigL, yOrigL);
	float xL=xy1[0];
	float yL=xy1[1];

	xy1 = getXYFromAngle(thetaR, proximalR, xOrigR, yOrigR);
	float xR=xy1[0];
	float yR=xy1[1];

	float *dest = getIntersec(distalL, distalR, xL, yL, xR, yR);

	float x_0 = dest[0];
	float y_0 = dest[1];
	if(workmode == 1 || workmode == 2 || workmode == 4) {
		if(y_0 < dest[3]) {
			x_0 = dest[2];
			y_0 = dest[3];
		}
	}
	else {
		// TODO error log or exception unsupported workmode
	}

	result[0] = xL;
	result[1] = yL;
	result[2] = xR;
	result[3] = yR;
	result[4] = x_0;
	result[5] = y_0;
	return result;
}

// 1 - angle - 2 are ordered clockwise. The angle is at the inner/right side, between 2 and 1 clockwise
float FiveBarScaraKinematics::getAngle(float x1, float y1, float xAngle, float yAngle, float x2, float y2) const
{
	float angle1 = getAbsoluteAngle(xAngle, yAngle, x1, y1);
	float angle2 = getAbsoluteAngle(xAngle, yAngle, x2, y2);

	float angle = 0.0;
	if(angle2 < angle1) {
		angle = 360 + angle2 - angle1;
	}
	else {
		angle = angle2 - angle1;
	}

	return angle;
}

bool FiveBarScaraKinematics::isPointInsideDefinedPrintableArea(float x0, float y0) const
{
	if(printAreaElementsDefined == 4) { // rectangle
		float x1 = printArea[0];
		float y1 = printArea[1];
		float x2 = printArea[2];
		float y2 = printArea[3];

		if(x1 < x2 && y1 > y2) {
			if(x0 >= x1 && x0 <= x2 && y2 <= y0 && y1 >= y0) {
				return true;
			}
			else {
				return false;
			}
		}
		else if(x1 > x2 && y1 < y2) {
			if(x0 >= x2 && x0 <= x1 && y2 >= y0 && y1 <= y0) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			// TODO error log
		}
	}
	else { // polygone
		// TODO implement
	}
	return false;
}

bool FiveBarScaraKinematics::constraintsOk(float x_0, float y_0) const
{
	float *inv = getInverse(x_0, y_0);	// xL, yL, thetaL, xR, yR, thetaR, x1, y1 (joint of distals)
	float xL = inv[0];
	float yL = inv[1];
	float thetaL = inv[2];
	float xR = inv[3];
	float yR = inv[4];
	float thetaR = inv[5];
	float x1 = inv[6];
	float y1 = inv[7];

	// check theta angles
	if(actuatorAngleLMin > thetaL || actuatorAngleLMax < thetaL) {
		return false;
	}
	if(actuatorAngleRMin > thetaR || actuatorAngleRMax < thetaR) {
		return false;
	}

	// check constr
	float constr = getAngle(xL, yL, x1, y1, xR, yR);
	if(constrMin > constr || constrMax < constr) {
		return false;
	}

	// check proxDistral angle L and R
	float angleProxDistL = getAngle(xOrigL, yOrigL, xL, yL, x1, y1);
	if(proxDistLAngleMin > angleProxDistL || proxDistLAngleMax < angleProxDistL) {
		return false;
	}
	float angleProxDistR = getAngle(xOrigR, yOrigR, xR, yR, x1, y1);
	if(proxDistRAngleMin > angleProxDistR || proxDistRAngleMax < angleProxDistR) {
		return false;
	}

	return true;
}

///////////////////////////// public functions /////////////////////////


// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool FiveBarScaraKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
{
	if (mCode == 669)
	{
		// must be defined: X, Y, P, D
		if(!gb.Seen('X') || !(gb.Seen('Y')) || !(gb.Seen('P')) || !(gb.Seen('D'))) {
			error = true;
			return true;
		}

		bool seen = false;
		bool seenNonGeometry = false;


		// parameter X: x values of actuators
		float paraX[2];
		gb.TryGetFloatArray('X', 2, paraX, reply, seen);
		xOrigL = paraX[0];
		xOrigR = paraX[1];


		// parameter Y: y values of actuators
		float paraY[2];
		gb.TryGetFloatArray('Y', 2, paraY, reply, seen);
		yOrigL = paraY[0];
		yOrigR = paraY[1];


		if(gb.Seen('L')) {
			long wm = 0L;
			gb.TryGetIValue('L', wm, seen);
			workmode = (int) wm;
		}
		else {
			workmode = 1;	// default
		}


		float proximalLengths[2];
		gb.TryGetFloatArray('P', 2, proximalLengths, reply, seen);
		proximalL = proximalLengths[0];
		proximalR = proximalLengths[1];


		int numParameters = getNumParameters('D', gb);
		if(numParameters == 2) {
			float distalLengths[2];
			gb.TryGetFloatArray('D', 2, distalLengths, reply, seen);
			distalL = distalLengths[0];
			distalR = distalLengths[1];
		}
		else if(numParameters == 4) {
			float distalLengths[4];
			gb.TryGetFloatArray('D', 4, distalLengths, reply, seen);
			distalL = distalLengths[0];
			distalR = distalLengths[1];
			cantL = distalLengths[2];
			cantR = distalLengths[3];
		}


		if(gb.Seen('B')) {
			float homingAngles[2];
			gb.TryGetFloatArray('B', 2, homingAngles, reply, seen);
			homingAngleL = homingAngles[0];
			homingAngleR = homingAngles[1];
		}
		else {
			homingAngleL = 90.0;	// default
			homingAngleR = 90.0;	// default
		}


		if(gb.Seen('A')) {
			float angles[4];
			gb.TryGetFloatArray('A', 4, angles, reply, seen);
			constrMin = angles[0];
			constrMax = angles[1];
			proxDistLAngleMin = angles[2];
			proxDistLAngleMax = angles[3];
			proxDistRAngleMin = angles[4];
			proxDistRAngleMax = angles[5];
		}
		else  {
			constrMin = 15.0;
			constrMax = 165.0;
			proxDistLAngleMin = 0.0;
			proxDistLAngleMax = 360.0;
			proxDistRAngleMin = 0.0;
			proxDistRAngleMax = 360.0;
		}


		if(gb.Seen('C')) {
			float angles[4];
			gb.TryGetFloatArray('C', 4, angles, reply, seen);
			actuatorAngleLMin = angles[0];
			actuatorAngleLMax = angles[1];
			actuatorAngleRMin = angles[2];
			actuatorAngleRMax = angles[3];
		}
		else {
			actuatorAngleLMin = 10.0;
			actuatorAngleLMax = 170.0;
			actuatorAngleRMin = 10.0;
			actuatorAngleRMax = 170.0;
		}


		if(gb.Seen('Z')) {
			int arraysize = sizeof(printArea);
			int numParameters = getNumParameters('Z', gb);
			if(numParameters > 0 && numParameters % 2 == 0) {
				float coordinates[numParameters];
				gb.TryGetFloatArray('Z', numParameters, coordinates, reply, seen);
				int limit = std::min(numParameters, arraysize);
				for(int i=0; i < limit; i++) {
					printArea[i] = coordinates[i];
				}
				for(int i=limit; i < arraysize; i++) {
					printArea[i] = 0.0;
				}
			}
			printAreaElementsDefined = numParameters;
			printAreaDefined = true;
		}
		else {
			printAreaElementsDefined = 0;
			printAreaDefined = false;
		}


		gb.TryGetFValue('S', segmentsPerSecond, seenNonGeometry);		// value defined in Kinematics.h
		gb.TryGetFValue('T', minSegmentLength, seenNonGeometry);		// value defined in Kinematics.h


		if (seen || seenNonGeometry)
		{
			Recalc();
		}
		else if (!gb.Seen('K'))
		{
			reply.printf("Kinematics is FiveBarScara, documented in https://duet3d.dozuki.com/Guide/Five+Bar+Parallel+SCARA/24?lang=en");
		}
		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
bool FiveBarScaraKinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated) const
{
	// First limit all axes according to M208
	const bool m208Limited = Kinematics::LimitPosition(coords, numVisibleAxes, axesHomed, isCoordinated);
	return m208Limited;
}


// Convert Cartesian coordinates to motor coordinates, returning true if successful
// In the following, theta is the proximal arm angle relative to the X axis, psi is the distal arm angle relative to the proximal arm
bool FiveBarScaraKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	float x_0 = machinePos[0];
	float y_0 = machinePos[1];
	float *inv = getInverse(x_0, y_0);	// xL, yL, thetaL, xR, yR, thetaR, x1, y1 (joint of distals)

	motorPos[X_AXIS] = lrintf(inv[2] * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf(inv[5] * stepsPerMm[Y_AXIS]);
	motorPos[Z_AXIS] = lrintf(machinePos[Z_AXIS] * stepsPerMm[Z_AXIS]);

	// Transform any additional axes linearly
	for (size_t axis = XYZ_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// For Scara, the X and Y components of stepsPerMm are actually steps per degree angle.
void FiveBarScaraKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	float thetaL = ((float)motorPos[X_AXIS]/stepsPerMm[X_AXIS]);
	float thetaR = ((float)motorPos[Y_AXIS]/stepsPerMm[Y_AXIS]);

	float x_0 = -1.0;
	float y_0 = -1.0;

	float *result = getForward(thetaL, thetaR, workmode);
	float x1 = result[4];
	float y1 = result[5];

	if(isCantilevered(1)) {	// left distal is prolongued
		float xL = result[0];
		float yL = result[1];

		// calculate cantilever from psiL
		float psiL = getAbsoluteAngle(xL, yL, x1, y1);
		float *destXY = getXYFromAngle(psiL, cantL, x1, y1);
		x_0 = destXY[0];
		y_0 = destXY[1];
	}
	else if(isCantilevered(2)) { // right distal is prolongued
		float xR = result[2];
		float yR = result[3];

		// now calculate cantilever from psiR
		float psiR = getAbsoluteAngle(xR, yR, x1, y1);
		float *destXY = getXYFromAngle(psiR, cantR, x1, y1);
		x_0 = destXY[0];
		y_0 = destXY[1];
	}
	else {
		x_0 = x1;
		y_0 = y1;
	}


    machinePos[X_AXIS] = x_0;
    machinePos[Y_AXIS] = y_0;

    machinePos[Z_AXIS] = (float) motorPos[Z_AXIS] / stepsPerMm[Z_AXIS];

	// Convert any additional axes linearly
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
bool FiveBarScaraKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	// Check the M208 limits first
	float coords[2] = {x, y};
	if (Kinematics::LimitPosition(coords, 2, LowestNBits<AxesBitmap>(2), isCoordinated))
	{
		return false;
	}

	if(printAreaDefined) {
		if(isPointInsideDefinedPrintableArea(x, y)) {
			return true;
		}
	}
	else {
		if(constraintsOk(x, y)) {
			return true;
		}
	}

	return false;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void FiveBarScaraKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap FiveBarScaraKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const
{
	// If both X and Y have been specified then we know the positions of both arm motors, otherwise we don't
	const AxesBitmap xyAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS);
	if ((g92Axes & xyAxes) != xyAxes)
	{
		g92Axes &= ~xyAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap FiveBarScaraKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const
{
	constexpr AxesBitmap xyzAxes = MakeBitmap<AxesBitmap>(X_AXIS) |  MakeBitmap<AxesBitmap>(Y_AXIS) |  MakeBitmap<AxesBitmap>(Z_AXIS);
	if ((axesMoving & xyzAxes) != 0)
	{
		axesMoving |= xyzAxes;
	}
	return axesMoving;
}

size_t FiveBarScaraKinematics::NumHomingButtons(size_t numVisibleAxes) const
{
	const MassStorage *storage = reprap.GetPlatform().GetMassStorage();
	if (!storage->FileExists(SYS_DIR, Home5BarScaraFileName))
	{
		return 0;
	}
	return numVisibleAxes;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap FiveBarScaraKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const
{
	// Ask the base class which homing file we should call first
	AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	filename.copy(Home5BarScaraFileName);

	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool FiveBarScaraKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return (axis == X_AXIS  || axis == Y_AXIS);
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void FiveBarScaraKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	switch (axis)
	{
	case X_AXIS:	// proximal joint homing switch
		dda.SetDriveCoordinate(lrintf(homingAngleL * stepsPerMm[axis]), axis);
		break;

	case Y_AXIS:	// distal joint homing switch
		dda.SetDriveCoordinate(lrintf(homingAngleR * stepsPerMm[axis]), axis);
		break;

	case Z_AXIS:	// Z axis homing switch
		{
			const float hitPoint = ((highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis));
			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
		}
		break;

	default:		// Additional axis
		{
			const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
		}
		break;
	}
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void FiveBarScaraKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const
{
	// For now we limit the speed in the XY plane to the lower of the X and Y maximum speeds, and similarly for the acceleration.
	// Limiting the angular rates of the arms would be better.
	const float xyFactor = sqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
	if (xyFactor > 0.01)
	{
		const Platform& platform = reprap.GetPlatform();
		const float maxSpeed = min<float>(platform.MaxFeedrate(X_AXIS), platform.MaxFeedrate(Y_AXIS));
		const float maxAcceleration = min<float>(platform.Acceleration(X_AXIS), platform.Acceleration(Y_AXIS));
		dda.LimitSpeedAndAcceleration(maxSpeed/xyFactor, maxAcceleration/xyFactor);
	}
}

// Return true if the specified axis is a continuous rotation axis
bool FiveBarScaraKinematics::IsContinuousRotationAxis(size_t axis) const
{
	//return axis < 2 && supportsContinuousRotation[axis];
	return axis < 2;
}

// Recalculate the derived parameters
void FiveBarScaraKinematics::Recalc()
{
}

// End
