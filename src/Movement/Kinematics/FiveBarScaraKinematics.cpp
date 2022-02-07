/*
 * FiveBarScaraKinematics.cpp
 *
 *  Created on: 11 Nov 2018
 *      Author: JoergS5, bondus
 *
 *	documentation: https://duet3d.dozuki.com/Guide/Five+Bar+Parallel+SCARA/24?lang=en
 */

#include "FiveBarScaraKinematics.h"

#if SUPPORT_FIVEBARSCARA

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/DDA.h>

#include <limits>

//#define debugPrintf if(0) debugPrintf

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(FiveBarScaraKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry FiveBarScaraKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
	//TODO lots more to be added here
};

constexpr uint8_t FiveBarScaraKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(FiveBarScaraKinematics, ZLeadscrewKinematics)

#endif

FiveBarScaraKinematics::FiveBarScaraKinematics() noexcept
	: ZLeadscrewKinematics(KinematicsType::scara, SegmentationType(true, false, false))
{
	Recalc();
}

// Return the name of the current kinematics
const char *FiveBarScaraKinematics::GetName(bool forStatusReport) const noexcept
{
	return "FiveBarScara";
}

//////////////////////// private functions /////////////////////////

// no results returned. All results are stored in the cached variables.
// The constraints are not checked
void FiveBarScaraKinematics::getInverse(const float coords[]) const noexcept
{
	if (!cachedInvalid && coords[0] == cachedX0 && coords[1] == cachedY0)
	{	// already solved
		return;
	}

	float thetaL = -1.0;
	float thetaR = -1.0;
	float xL = -1.0;
	float xR = -1.0;
	float yL = -1.0;
	float yR = -1.0;
	float x1 = -1.0;
	float y1 = -1.0;

	float x_0 = coords[0];
	float y_0 = coords[1];

	if (isCantilevered(1))
	{
		// calculate cantilevered side first:

		float lefttheta[6];
		getTheta(lefttheta, proximalL, distalL + cantL, xOrigL, yOrigL, x_0, y_0, Arm::left);
		xL = lefttheta[0];
		yL = lefttheta[1];
		thetaL = lefttheta[2];

		// calculate x1,y1, i.e. where the distal arms meet
		float fraction = distalL / (distalL + cantL);
		x1 = (x_0 - xL) * fraction + xL;
		y1 = (y_0 - yL) * fraction + yL;

		// calculate right, non cantilevered side:
		float righttheta[6];
		getTheta(righttheta, proximalR, distalR, xOrigR, yOrigR, x1, y1, Arm::right);
		xR = righttheta[0];
		yR = righttheta[1];
		thetaR = righttheta[2];
	}
	else if (isCantilevered(2))
	{
		// calculate cantilevered side first:
		float righttheta[6];
		getTheta(righttheta, proximalR, distalR + cantR, xOrigR, yOrigR, x_0, y_0, Arm::right);
		xR = righttheta[0];
		yR = righttheta[1];
		thetaR = righttheta[2];

		// calculate x1,y1, i.e. where the distal arms meet
		float fraction = distalR / (distalR + cantR);
		x1 = (x_0 - xR) * fraction + xR;
		y1 = (y_0 - yR) * fraction + yR;

		// calculate left, non cantilevered side:
		float lefttheta[6];
		getTheta(lefttheta, proximalL, distalL, xOrigL, yOrigL, x1, y1, Arm::left);
		xL = lefttheta[0];
		yL = lefttheta[1];
		thetaL = lefttheta[2];

	}
	else
	{	// not cantilevered, hotend is at top joint
		float lefttheta[6];
		getTheta(lefttheta, proximalL, distalL, xOrigL, yOrigL, x_0, y_0, Arm::left);
		x1 = x_0;
		y1 = y_0;
		xL = lefttheta[0];
		yL = lefttheta[1];
		thetaL = lefttheta[2];

		float righttheta[6];
		getTheta(righttheta, proximalR, distalR, xOrigR, yOrigR, x_0, y_0, Arm::right);
		xR = righttheta[0];
		yR = righttheta[1];
		thetaR = righttheta[2];
	}

	cachedX0 = x_0;
	cachedY0 = y_0;
	cachedXL = xL;
	cachedYL = yL;
	cachedThetaL = thetaL;

	cachedXR = xR;
	cachedYR = yR;
	cachedThetaR = thetaR;

	cachedX1 = x1;
	cachedY1 = y1;

	cachedInvalid =
		std::isnan(cachedX0) || std::isnan(cachedY0) ||
		std::isnan(cachedX1) || std::isnan(cachedY1) ||
		std::isnan(cachedXL) || std::isnan(cachedYL) ||
		std::isnan(cachedXR) || std::isnan(cachedYR) ||
		std::isnan(cachedThetaR) || std::isnan(cachedXR) || std::isnan(cachedYR) ||
		std::isnan(cachedThetaL) || std::isnan(cachedXL) || std::isnan(cachedYL);
}

// quadrants: 1 is right upper, 2 is left upper, 3 is left down, 4 is right down
int FiveBarScaraKinematics::getQuadrant(float x, float y) const noexcept
{
	return (x >= 0.0 && y >= 0.0) ? 1
			: (x < 0.0 && y >= 0.0) ? 2
				: (x < 0.0 && y < 0.0) ? 3
					: 4;						// x >= 0 && y < 0
}

// return true if the Scara is cantilevered
bool FiveBarScaraKinematics::isCantilevered(int mode) const noexcept
{
	return (cantL > 0.0f && mode == 1) || (cantR > 0.0f && mode == 2);
}

// get angle between -90 and 270 for given origin and destination coordinates
float FiveBarScaraKinematics::getAbsoluteAngle(float xOrig, float yOrig, float xDest, float yDest) const noexcept
{
	const float length = fastSqrtf(fsquare(xOrig - xDest) + fsquare(yOrig - yDest));
	const float y = fabs(yOrig - yDest);
	float angle = asinf(y / length) * 180.0f / Pi;

	const int quad = getQuadrant(xDest - xOrig, yDest - yOrig);

	switch(quad)
	{
	case 1: // right upper quadrant I
		// nothing to change
		break;
	case 2:  // left upper quadrant II
		angle = 180.0 - angle;
		break;
	case 3:  // left lower quadrant III
		angle = 180.0 + angle;
		break;
	case 4: // right lower quadrant IV
		angle = 360.0 - angle;
		break;
	}
	return angle;
}

// first circle, second circle. Return the two intersection points
void FiveBarScaraKinematics::getIntersec(float result[], float firstRadius, float secondRadius, float firstX, float firstY, float secondX, float secondY) const noexcept
{
	const float firstRadius2  = fsquare(firstRadius);
	const float secondRadius2 = fsquare(secondRadius);

	const float distance2 = fsquare(firstX - secondX) + fsquare(firstY - secondY);
	const float distance  = fastSqrtf(distance2);

	const float delta = 0.25 * fastSqrtf(
			(distance + firstRadius + secondRadius)
			* (distance + firstRadius - secondRadius)
			* (distance - firstRadius + secondRadius)
			* (-distance + firstRadius + secondRadius)
		);

	// calculate x
	const float term1x = (firstX + secondX) / 2;
	const float term2x = (secondX - firstX) * (firstRadius2 - secondRadius2) / (2 * distance2);
	const float term3x = 2 * (firstY - secondY) / (distance2) * delta;
	const float x1 = term1x + term2x + term3x;
	const float x2 = term1x + term2x - term3x;

	// calculate y
	const float term1y = (firstY + secondY) / 2;
	const float term2y = (secondY - firstY)*(firstRadius2 - secondRadius2) / (2 * distance2);
	const float term3y = 2 * (firstX - secondX) / distance2 * delta;
	const float y1 = term1y + term2y - term3y;
	const float y2 = term1y + term2y + term3y;

	result[0] = x1;
	result[1] = y1;
	result[2] = x2;
	result[3] = y2;
}

// return coordinates and theta angles or both possible solutions
// first solution in [0], [1], [2] is the angle which fits to the current workmode
// result: x,y,theta of first point, then x,y,theta of second solution
void FiveBarScaraKinematics::getTheta(float result[], float prox, float distal, float proxX, float proxY, float destX, float destY, Arm arm) const noexcept
{
	float inter12[4];
	getIntersec(inter12, prox, distal, proxX, proxY, destX, destY);
	const float x1 = inter12[0];
	const float y1 = inter12[1];
	const float x2 = inter12[2];
	const float y2 = inter12[3];

	const float thetaA = getAbsoluteAngle(proxX, proxY, x1, y1);
	const float thetaB = getAbsoluteAngle(proxX, proxY, x2, y2);

	const float proxTurnA = getTurn(proxX, proxY, x1, y1, destX, destY);
	const float proxTurnB = getTurn(proxX, proxY, x2, y2, destX, destY);

	int use = 0; // 1 for A, 2 for B

	switch(workmode)
	{
	case 1:
		if (proxTurnA > 0)
		{
			use = 1;
		}
		else if(proxTurnB > 0)
		{
			use = 2;
		}
		break;

	case 2:
		if (arm == Arm::left)
		{
			if (proxTurnA < 0)
			{
				use = 1;
			}
			else if (proxTurnB < 0)
			{
				use = 2;
			}
		}
		else
		{
			if (proxTurnA > 0)
			{
				use = 1;
			}
			else if (proxTurnB > 0)
			{
				use = 2;
			}
		}
		break;

	case 3:
		if (arm == Arm::left)
		{
			if (proxTurnA > 0)
			{
				use = 1;
			}
			else if (proxTurnB > 0)
			{
				use = 2;
			}
		}
		else
		{
			if (proxTurnA < 0)
			{
				use = 1;
			}
			else if (proxTurnB < 0)
			{
				use = 2;
			}
		}
		break;

	case 4:
		if (proxTurnA < 0)
		{
			use = 1;
		}
		else if (proxTurnB < 0)
		{
			use = 2;
		}
	}

	if (use == 1)
	{
	    result[0] = x1;
	    result[1] = y1;
	    result[2] = thetaA;
	    result[3] = x2;
	    result[4] = y2;
	    result[5] = thetaB;
	}
	else if (use == 2)
	{
	    result[0] = x2;
	    result[1] = y2;
	    result[2] = thetaB;
	    result[3] = x1;
	    result[4] = y1;
	    result[5] = thetaA;
	}
	else
	{
	    // fail!!
	    result[0] = std::numeric_limits<float>::quiet_NaN();
	    result[1] = std::numeric_limits<float>::quiet_NaN();
	    result[2] = std::numeric_limits<float>::quiet_NaN();
	    result[3] = std::numeric_limits<float>::quiet_NaN();
	    result[4] = std::numeric_limits<float>::quiet_NaN();
	    result[5] = std::numeric_limits<float>::quiet_NaN();
	}
}

// from given point with angle and length, calculate destination
// resultcoords: x, y
void FiveBarScaraKinematics::getXYFromAngle(float resultcoords[], float angle, float length, float origX, float origY) const noexcept
{
	const float xL = length * cosf(angle * DegreesToRadians);
	const float yL = length * sinf(angle * DegreesToRadians);

	resultcoords[0] = xL + origX;
	resultcoords[1] = yL + origY;
}

// get forward kinematics: from theta actuators angles, calculate destination coordinates
// optional cantilevered will be added later
// resultcoords: xL, yL, xR, yR, x0, y0
void FiveBarScaraKinematics::getForward(float resultcoords[], float thetaL, float thetaR) const noexcept
{
	float coordsL[2];
	getXYFromAngle(coordsL, thetaL, proximalL, xOrigL, yOrigL);
	const float xL = coordsL[0];
	const float yL = coordsL[1];

	float coordsR[2];
	getXYFromAngle(coordsR, thetaR, proximalR, xOrigR, yOrigR);
	const float xR = coordsR[0];
	const float yR = coordsR[1];


	float inter12[4];
	getIntersec(inter12, distalL, distalR, xL, yL, xR, yR); // two intersection points x,y

	resultcoords[0] = xL;
	resultcoords[1] = yL;
	resultcoords[2] = xR;
	resultcoords[3] = yR;

	// Figure out what solution to pick, depending on angle of hotend joints
	const float turnHot0 = getTurn(xL, yL, inter12[0], inter12[1], xR, yR);
	const float turnHot1 = getTurn(xL, yL, inter12[2], inter12[3], xR, yR);

	float xDst = std::numeric_limits<float>::quiet_NaN();;
	float yDst = std::numeric_limits<float>::quiet_NaN();;
	if (workmode >= 1 && workmode <= 4)
	{
		if (turnHot0 < 0)
		{ // right turn
			xDst = inter12[0];
			yDst = inter12[1];
		}
		else if (turnHot1 < 0)
		{
			xDst = inter12[2];
			yDst = inter12[3];
		}
		// Sanity check the elbow joins to make sure it's in the correct work mode
		const float tL = getTurn(xOrigL, yOrigL, xL, yL, xDst, yDst);
		const float tR = getTurn(xOrigR, yOrigR, xR, yR, xDst, yDst);

		if ((workmode == 1 && (tL < 0 || tR < 0)) ||
			(workmode == 2 && (tL > 0 || tR < 0)) ||
			(workmode == 3 && (tL < 0 || tR > 0)) ||
			(workmode == 4 && (tL > 0 || tR > 0)))
		{
			xDst = std::numeric_limits<float>::quiet_NaN();
			yDst = std::numeric_limits<float>::quiet_NaN();
		}

		//fprintf(stderr, "xy %f %f\n", xDst, yDst);
		resultcoords[4] = xDst;
		resultcoords[5] = yDst;
	}
	else
	{
		// work mode 5-8
		//TODO??
	}
}

// 1 - angle - 2 are ordered clockwise. The angle is at the inner/right side, between 2 and 1 clockwise
float FiveBarScaraKinematics::getAngle(float x1, float y1, float x2, float y2, float x3, float y3) const noexcept
{
	const float angle1 = getAbsoluteAngle(x2, y2, x1, y1);
	const float angle2 = getAbsoluteAngle(x2, y2, x3, y3);

	return (angle2 < angle1)
			? 360 + angle2 - angle1 // Keep the angle positive
				: angle2 - angle1;
}

// return positive if turn is left (counter-clockwise), negative if turn is right (clockwise)
float FiveBarScaraKinematics::getTurn(float x1, float y1, float x2, float y2, float x3, float y3) const noexcept
{
	return (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
}

bool FiveBarScaraKinematics::isPointInsideDefinedPrintableArea(float x0, float y0) const noexcept
{
	const float x1 = printArea[0];
	const float y1 = printArea[1];
	const float x2 = printArea[2];
	const float y2 = printArea[3];

	const float xmin = min(x1, x2);
	const float xmax = max(x1, x2);
	const float ymin = min(y1, y2);
	const float ymax = max(y1, y2);

	return x0 >= xmin && x0 <= xmax && y0 >= ymin && y0 <= ymax;
}

// Check that all consttraints are ok. If the constraints fails the cache is invalidated
bool FiveBarScaraKinematics::constraintsOk(const float coords[]) const noexcept
{
	if (!cachedInvalid && coords[0] == cachedX0 && coords[1] == cachedY0)
	{	// already solved and ok
		return true;
	}

	getInverse(coords);	// sets cache. xL, yL, thetaL, xR, yR, thetaR, x1, y1 (joint of distals)

	if (cachedInvalid)
	{
		return false;
    }

	// check theta angles
	float thetaL = cachedThetaL;
	if (actuatorAngleLMin < 0 && thetaL > actuatorAngleLMax)
	{
		thetaL -= 360; // Some trickery to allow negative min angles
	}
	if (thetaL < actuatorAngleLMin || thetaL > actuatorAngleLMax)
	{
		cachedInvalid = true;
		return false;
	}
	float thetaR = cachedThetaR;
	if (actuatorAngleRMin < 0 && thetaR > actuatorAngleRMax)
	{
		thetaR -= 360; // Some trickery to allow negative min angles
	}
	if (thetaR < actuatorAngleRMin || thetaR > actuatorAngleRMax)
	{
		cachedInvalid = true;
		return false;
	}

	// check head angles
	const float headAngle = getAngle(cachedXL, cachedYL, cachedX1, cachedY1, cachedXR, cachedYR);
	if (headAngle < headAngleMin|| headAngle > headAngleMax|| std::isnan(headAngle))
	{
		cachedInvalid = true;
		return false;
	}

	// check proxDistal angle L and R
	const float angleProxDistL = getAngle(xOrigL, yOrigL, cachedXL, cachedYL, cachedX1, cachedY1);
	if (angleProxDistL < proxDistLAngleMin|| angleProxDistL > proxDistLAngleMax || std::isnan(angleProxDistL))
	{
		cachedInvalid = true;
		return false;
	}
	const float angleProxDistR = getAngle(xOrigR, yOrigR, cachedXR, cachedYR, cachedX1, cachedY1);
	if (angleProxDistR < proxDistRAngleMin || angleProxDistR > proxDistRAngleMax || std::isnan(angleProxDistR))
	{
		cachedInvalid = true;
		return false;
	}

	cachedInvalid = false;
	return true;
}

///////////////////////////// public functions /////////////////////////


// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool FiveBarScaraKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) /*override*/
{
	if (mCode == 669)
	{
		//TODO this should print the existing values if no parameters are given, instead of insisting that all parameters are present
		// must be defined: X, Y, P, D
		gb.MustSee('X');
		gb.MustSee('Y');
		gb.MustSee('P');
		gb.MustSee('D');

		bool seen = false;

		// parameter X: x origins of actuators
		float paraX[2];
		gb.TryGetFloatArray('X', 2, paraX, reply, seen);
		xOrigL = paraX[0];
		xOrigR = paraX[1];

		// parameter Y: y origins of actuators
		float paraY[2];
		gb.TryGetFloatArray('Y', 2, paraY, reply, seen);
		yOrigL = paraY[0];
		yOrigR = paraY[1];

		// workmode, default 1 is left buckled, right bulged
		if (gb.Seen('L'))
		{
			int32_t wm = 0L;
			gb.TryGetIValue('L', wm, seen);
			workmode = (int)wm;
			if (!(workmode == 1 || workmode == 2 || workmode == 4))
			{
				error = true;
				return true;
			}
		}
		else
		{
			workmode = 1;	// default
		}

		// proximal arm lengths
		float proximalLengths[2];
		gb.TryGetFloatArray('P', 2, proximalLengths, reply, seen);
		proximalL = proximalLengths[0];
		proximalR = proximalLengths[1];

		// distal arm lengths and optional lengths of cantilevered arm
		bool dseen = false;
		float distalLengths[4];
		//TODO TryGetFloatArray will report an error if the wrong number of values is provided. But we want to allow either 2 or 4.
		//TODO So this code should call Seen() followed by GetFloatArray() instead, then check the number of returned values.
		if (!gb.TryGetFloatArray('D', 4, distalLengths, reply, dseen) && dseen)
		{
			distalL = distalLengths[0];
			distalR = distalLengths[1];
			cantL = distalLengths[2];
			cantR = distalLengths[3];
			seen = true;
		}
		else
		{
			dseen = false;
			if (!gb.TryGetFloatArray('D', 2, distalLengths, reply, dseen) && dseen)
			{
				distalL = distalLengths[0];
				distalR = distalLengths[1];
				cantL = 0.0;
				cantR = 0.0;
				seen = true;
			}
		}

		// angle of the actuator in the home position
		if (gb.Seen('B'))
		{
			float homingAngles[2];
			gb.TryGetFloatArray('B', 2, homingAngles, reply, seen);
			homingAngleL = homingAngles[0];
			homingAngleR = homingAngles[1];
		}
		else
		{
			//TODO instead of doing this, set these values up up during initialisation or when changing workmode, and leave them alone here
			if (workmode == 1)
			{
				homingAngleL = 20.0;	// default
				homingAngleR = 10.0;	// default
			}
			else if (workmode == 2)
			{
				homingAngleL = 110.0;	// default
				homingAngleR = 20.0;	// default
			}
			else if (workmode == 4)
			{
				homingAngleL = 110.0;	// default
				homingAngleR = 100.0;	// default
			}
			else
			{
				homingAngleL = 90.0;	// default
				homingAngleR = 90.0;	// default
			}
		}

		// angle constraints: between distals and between proximal and distal
		if (gb.Seen('A'))
		{
			float angles[6];
			gb.TryGetFloatArray('A', 6, angles, reply, seen);
			headAngleMin      = angles[0];
			headAngleMax      = angles[1];
			proxDistLAngleMin = angles[2];
			proxDistLAngleMax = angles[3];
			proxDistRAngleMin = angles[4];
			proxDistRAngleMax = angles[5];
		}
		else
		{
			headAngleMin = 15.0;
			headAngleMax = 165.0;
			proxDistLAngleMin = 0.0;
			proxDistLAngleMax = 360.0;
			proxDistRAngleMin = 0.0;
			proxDistRAngleMax = 360.0;
		}

		// actuator angle constraints
		if (gb.Seen('C'))
		{
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

		// optional rectangle definition of a print area. Must match the workmode reachable area
		//TODO is this needed? Why not use the M208 limits instead?
		bool seenNonGeometry = TryConfigureSegmentation(gb);
		if (gb.Seen('Z'))
		{
			float coordinates[4];
			gb.TryGetFloatArray('Z', 4, coordinates, reply, seenNonGeometry);
			for (int i=0; i < 4; i++)
			{
				printArea[i] = coordinates[i];
			}
			printAreaDefined = true;
		}
		else
		{
			printAreaDefined = false;
		}

		if (seen)
		{
			Recalc();
		}
		else if (!seenNonGeometry && !gb.Seen('K'))
		{
			//TODO print all the parameters here
			Kinematics::Configure(mCode, gb, reply, error);
			reply.catf(", documented in https://duet3d.dozuki.com/Guide/Five+Bar+Parallel+SCARA/24?lang=en");
		}

		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
LimitPositionResult FiveBarScaraKinematics::LimitPosition(float coords[], const float * null initialCoords,
															size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	// First limit all axes according to M208
	const bool m208Limited = applyM208Limits && Kinematics::LimitPositionFromAxis(coords, 0, numVisibleAxes, axesToLimit);

	if (!constraintsOk(coords))
	{
		return LimitPositionResult::intermediateUnreachable;	//TODO is this right?
	}

	return (m208Limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}


// Convert Cartesian coordinates to motor coordinates, returning true if successful
// In the following, theta is the proximal arm angle relative to the X axis, psi is the distal arm angle relative to the proximal arm
bool FiveBarScaraKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
													size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	float coords[2] = { machinePos[0], machinePos[1] };
	getInverse(coords);

	if (!constraintsOk(coords))
	{
		return false;
	}

	motorPos[X_AXIS] = lrintf(cachedThetaL * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf(cachedThetaR * stepsPerMm[Y_AXIS]);
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
void FiveBarScaraKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{

	const float thetaL = ((float)motorPos[X_AXIS]/stepsPerMm[X_AXIS]);
	const float thetaR = ((float)motorPos[Y_AXIS]/stepsPerMm[Y_AXIS]);

	float x_0 = -1.0;
	float y_0 = -1.0;

	float resultcoords[6];
	getForward(resultcoords, thetaL, thetaR);
	const float x1 = resultcoords[4];
	const float y1 = resultcoords[5];

	if (isCantilevered(1))
	{
		// left distal is prolonged
		const float xL = resultcoords[0];
		const float yL = resultcoords[1];

		// calculate cantilever from psiL
		const float psiL = getAbsoluteAngle(xL, yL, x1, y1);
		float dest[2];
		getXYFromAngle(dest, psiL, cantL, x1, y1);
		x_0 = dest[0];
		y_0 = dest[1];
	}
	else if(isCantilevered(2))
	{
		// right distal is prolonged
		const float xR = resultcoords[2];
		const float yR = resultcoords[3];

		// now calculate cantilever from psiR
		const float psiR = getAbsoluteAngle(xR, yR, x1, y1);
		float dest[2];
		getXYFromAngle(dest, psiR, cantR, x1, y1);
		x_0 = dest[0];
		y_0 = dest[1];
	}
	else
	{
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
bool FiveBarScaraKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
	if (axes.IsBitSet(X_AXIS) && axes.IsBitSet(Y_AXIS))
	{
		float coords[2] = {axesCoords[X_AXIS], axesCoords[Y_AXIS]};
		if (!constraintsOk(coords))
		{
			return false;
		}
	}
	axes.ClearBit(X_AXIS);
	axes.ClearBit(Y_AXIS);
	return Kinematics::IsReachable(axesCoords, axes);
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void FiveBarScaraKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	// x and y are unknown. The stepper angles shall be near the homing endstops, so some iterations turning left and right
	// will find the endstop positions. The angles and coordinates are clearly defined then.
	positions[X_AXIS] = 0.0;
	positions[Y_AXIS] = 0.0;
	for (size_t i = Z_AXIS; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap FiveBarScaraKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// If both X and Y have been specified then we know the positions of both arm motors, otherwise we don't
	if ((g92Axes & XyAxes) != XyAxes)
	{
		g92Axes &= ~XyAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap FiveBarScaraKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
{
	if (axesMoving.Intersects(XyzAxes))
	{
		axesMoving |= XyzAxes;
	}
	return axesMoving;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap FiveBarScaraKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	// Ask the base class which homing file we should call first
	AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	filename.copy(Home5BarScaraFileName);

	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool FiveBarScaraKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void FiveBarScaraKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
{
	switch (axis)
	{
	case X_AXIS:	// Left arm homing switch
		dda.SetDriveCoordinate(lrintf(homingAngleL * stepsPerMm[axis]), axis);
		break;

	case Y_AXIS:	// Left arm homing switch
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

// Return true if the specified axis is a continuous rotation axis
bool FiveBarScaraKinematics::IsContinuousRotationAxis(size_t axis) const noexcept
{
	return axis == X_AXIS || axis == Y_AXIS || Kinematics::IsContinuousRotationAxis(axis);
}

AxesBitmap FiveBarScaraKinematics::GetLinearAxes() const noexcept
{
	return AxesBitmap::MakeFromBits(Z_AXIS);
}

AxesBitmap FiveBarScaraKinematics::GetConnectedAxes(size_t axis) const noexcept
{
	return (axis == X_AXIS || axis == Y_AXIS)
			? XyAxes
				: AxesBitmap::MakeFromBits(axis);
}

// Recalculate the derived parameters
void FiveBarScaraKinematics::Recalc() noexcept
{
	cachedX0 = std::numeric_limits<float>::quiet_NaN(); // make sure that the cached values won't match any coordinates
	cachedY0 = std::numeric_limits<float>::quiet_NaN(); // make sure that the cached values won't match any coordinates
	cachedInvalid = true;
}

#endif // SUPPORT_FIVEBARSCARA

// End
