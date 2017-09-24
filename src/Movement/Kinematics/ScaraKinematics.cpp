/*
 * ScaraKinematics.cpp
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#include "ScaraKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "Storage/MassStorage.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/DDA.h"

ScaraKinematics::ScaraKinematics()
	: ZLeadscrewKinematics(KinematicsType::scara, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true),
	  proximalArmLength(DefaultProximalArmLength), distalArmLength(DefaultDistalArmLength), xOffset(0.0), yOffset(0.0)
{
	thetaLimits[0] = DefaultMinTheta;
	thetaLimits[1] = DefaultMaxTheta;
	phiLimits[0] = DefaultMinPhi;
	phiLimits[1] = DefaultMaxPhi;
	crosstalk[0] = crosstalk[1] = crosstalk[2] = 0.0;
	Recalc();
}

// Return the name of the current kinematics
const char *ScaraKinematics::GetName(bool forStatusReport) const
{
	return "Scara";
}

// Convert Cartesian coordinates to motor coordinates
// In the following, theta is the proximal arm angle relative to the X axis, psi is the distal arm angle relative to the proximal arm
bool ScaraKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool allowModeChange) const
{
	// No need to limit x,y to reachable positions here, we already did that in class GCodes
	const float x = machinePos[X_AXIS] + xOffset;
	const float y = machinePos[Y_AXIS] + yOffset;
	const float cosPsi = (fsquare(x) + fsquare(y) - proximalArmLengthSquared - distalArmLengthSquared) / twoPd;

	// SCARA position is undefined if abs(SCARA_C2) >= 1. In reality abs(SCARA_C2) >0.95 can be problematic.
	const float square = 1.0f - fsquare(cosPsi);
	if (square < 0.01f)
	{
		return false;		// not reachable
	}

	const float sinPsi = sqrtf(square);
	float psi = acosf(cosPsi);
	const float SCARA_K1 = proximalArmLength + distalArmLength * cosPsi;
	const float SCARA_K2 = distalArmLength * sinPsi;
	float theta;

	// Try the current arm mode, then the other one
	bool switchedMode = false;
	for (;;)
	{
		if (isDefaultArmMode != switchedMode)
		{
			// The following equations choose arm mode 0 i.e. distal arm rotated anticlockwise relative to proximal arm
			theta = atan2f(SCARA_K1 * y - SCARA_K2 * x, SCARA_K1 * x + SCARA_K2 * y);
			if (theta >= thetaLimits[0] && theta <= thetaLimits[1] && psi >= phiLimits[0] && psi <= phiLimits[1])
			{
				break;
			}
		}
		else
		{
			// The following equations choose arm mode 1 i.e. distal arm rotated clockwise relative to proximal arm
			theta = atan2f(SCARA_K1 * y + SCARA_K2 * x, SCARA_K1 * x - SCARA_K2 * y);
			if (theta >= thetaLimits[0] && theta <= thetaLimits[1] && psi >= phiLimits[0] && psi <= phiLimits[1])
			{
				psi = -psi;
				break;
			}
		}

		if (switchedMode)
		{
			return false;		// not reachable
		}
		switchedMode = true;
	}

	// Now that we know we are going to do the move, update the arm mode
	if (switchedMode)
	{
		isDefaultArmMode = !isDefaultArmMode;
	}

//debugPrintf("psi = %.2f, theta = %.2f\n", psi * RadiansToDegrees, theta * RadiansToDegrees);

	motorPos[X_AXIS] = lrintf(theta * RadiansToDegrees * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf((psi * RadiansToDegrees * stepsPerMm[Y_AXIS]) - (crosstalk[0] * motorPos[X_AXIS]));
	motorPos[Z_AXIS] = lrintf((machinePos[Z_AXIS] * stepsPerMm[Z_AXIS]) - (motorPos[X_AXIS] * crosstalk[1]) - (motorPos[Y_AXIS] * crosstalk[2]));

	// Transform any additional axes linearly
	for (size_t axis = XYZ_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// For Scara, the X and Y components of stepsPerMm are actually steps per degree angle.
void ScaraKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	const float arm1Angle = ((float)motorPos[X_AXIS]/stepsPerMm[X_AXIS]) * DegreesToRadians;
    const float arm2Angle = (((float)motorPos[Y_AXIS] + ((float)motorPos[X_AXIS] * crosstalk[0]))/stepsPerMm[Y_AXIS]) * DegreesToRadians;

    machinePos[X_AXIS] = (cosf(arm1Angle) * proximalArmLength + cosf(arm1Angle + arm2Angle) * distalArmLength) - xOffset;
    machinePos[Y_AXIS] = (sinf(arm1Angle) * proximalArmLength + sinf(arm1Angle + arm2Angle) * distalArmLength) - yOffset;

    // On some machines (e.g. Helios), the X and/or Y arm motors also affect the Z height
    machinePos[Z_AXIS] = ((float)motorPos[Z_AXIS] + ((float)motorPos[X_AXIS] * crosstalk[1]) + ((float)motorPos[Y_AXIS] * crosstalk[2]))/stepsPerMm[Z_AXIS];

	// Convert any additional axes linearly
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool ScaraKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) /*override*/
{
	if (mCode == 669)
	{
		bool seen = false;
		bool seenNonGeometry = false;
		gb.TryGetFValue('P', proximalArmLength, seen);
		gb.TryGetFValue('D', distalArmLength, seen);
		gb.TryGetFValue('S', segmentsPerSecond, seenNonGeometry);
		gb.TryGetFValue('T', minSegmentLength, seenNonGeometry);
		gb.TryGetFValue('X', xOffset, seen);
		gb.TryGetFValue('Y', yOffset, seen);
		if (gb.TryGetFloatArray('A', 2, thetaLimits, reply, seen))
		{
			return true;
		}
		if (gb.TryGetFloatArray('B', 2, phiLimits, reply, seen))
		{
			return true;
		}
		if (gb.TryGetFloatArray('C', 3, crosstalk, reply, seen))
		{
			return true;
		}

		if (seen || seenNonGeometry)
		{
			Recalc();
		}
		else
		{
			reply.printf("Printer mode is Scara with proximal arm %.2fmm range %.1f to %.1f" DEGREE_SYMBOL
							", distal arm %.2fmm range %.1f to %.1f" DEGREE_SYMBOL ", crosstalk %.1f:%.1f:%.1f, bed origin (%.1f, %.1f), segments/sec %d, min. segment length %.2f",
							(double)proximalArmLength, (double)thetaLimits[0], (double)thetaLimits[1],
							(double)distalArmLength, (double)phiLimits[0], (double)phiLimits[1],
							(double)crosstalk[0], (double)crosstalk[1], (double)crosstalk[2],
							(double)xOffset, (double)yOffset,
							(int)segmentsPerSecond, (double)minSegmentLength);
		}
		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
// TODO add an arm mode parameter?
bool ScaraKinematics::IsReachable(float x, float y) const
{
	// TODO The following isn't quite right, in particular it doesn't take account of the maximum arm travel
    const float r = sqrtf(fsquare(x + xOffset) + fsquare(y + yOffset));
    return r >= minRadius && r <= maxRadius && (x + xOffset) > 0.0;
}

// Limit the Cartesian position that the user wants to move to
// TODO take account of arm angle limits
bool ScaraKinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed) const
{
	// First limit all axes according to M208
	const bool m208Limited = Kinematics::LimitPosition(coords, numVisibleAxes, axesHomed);

	// Now check that the XY position is within radius limits, in case the M208 limits are too generous
	bool radiusLimited = false;
	float x = coords[X_AXIS] + xOffset;
	float y = coords[Y_AXIS] + yOffset;
	const float r2 = fsquare(x) + fsquare(y);
	if (r2 < minRadiusSquared)
	{
		const float r = sqrtf(r2);
		// The user may have specified x=0 y=0 so allow for this
		if (r < 1.0)
		{
			x = minRadius;
			y = 0.0;
		}
		else
		{
			x *= minRadius/r;
			y *= minRadius/r;
		}
		radiusLimited = true;
	}
	else if (r2 > maxRadiusSquared)
	{
		const float r = sqrtf(r2);
		x *= maxRadius/r;
		y *= maxRadius/r;
		radiusLimited = true;
	}

	if (radiusLimited)
	{
		coords[X_AXIS] = x - xOffset;
		coords[Y_AXIS] = y - yOffset;
	}

	return m208Limited || radiusLimited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void ScaraKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	positions[X_AXIS] = maxRadius - xOffset;
	positions[Y_AXIS] = -yOffset;
	for (size_t i = Z_AXIS; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap ScaraKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const
{
	// If both X and Y have been specified then we know the positions of both arm motors, otherwise we don't
	const AxesBitmap xyAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS);
	if ((g92Axes & xyAxes) != xyAxes)
	{
		g92Axes &= ~xyAxes;
	}
	return g92Axes;
}

size_t ScaraKinematics::NumHomingButtons(size_t numVisibleAxes) const
{
	const MassStorage *storage = reprap.GetPlatform().GetMassStorage();
	if (!storage->FileExists(SYS_DIR, HomeProximalFileName))
	{
		return 0;
	}
	if (!storage->FileExists(SYS_DIR, HomeDistalFileName))
	{
		return 1;
	}
	if (!storage->FileExists(SYS_DIR, StandardHomingFileNames[Z_AXIS]))
	{
		return 2;
	}
	return numVisibleAxes;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
const char* ScaraKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, AxesBitmap& mustHomeFirst) const
{
	// Ask the base class which homing file we should call first
	const char* ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, mustHomeFirst);
	// Change the returned name if it is X or Y
	if (ret == StandardHomingFileNames[X_AXIS])
	{
		ret = HomeProximalFileName;
	}
	else if (ret == StandardHomingFileNames[Y_AXIS])
	{
		ret = HomeDistalFileName;
	}

	// Some SCARA printers cannot have individual axes homed safely. So it the user doesn't provide the homing file for an axis, default to homeall.
	const MassStorage *storage = reprap.GetPlatform().GetMassStorage();
	if (!storage->FileExists(SYS_DIR, ret))
	{
		ret = HomeAllFileName;
	}
	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool ScaraKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void ScaraKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	const float hitPoint = (axis == 0)
							? ((highEnd) ? thetaLimits[1] : thetaLimits[0])						// proximal joint homing switch
							  : (axis == 1)
								? ((highEnd) ? phiLimits[1] : phiLimits[0])	// distal joint homing switch
								  : (highEnd)
									? reprap.GetPlatform().AxisMaximum(axis)					// other axis (e.g. Z) high end homing switch
									  : reprap.GetPlatform().AxisMinimum(axis);					// other axis (e.g. Z) low end homing switch
	dda.SetDriveCoordinate(hitPoint * stepsPerMm[axis], axis);
}

// Recalculate the derived parameters
void ScaraKinematics::Recalc()
{
	proximalArmLengthSquared = fsquare(proximalArmLength);
	distalArmLengthSquared = fsquare(distalArmLength);
	twoPd = proximalArmLength * distalArmLength * 2.0f;

	minRadius = (proximalArmLength + distalArmLength * min<float>(cosf(phiLimits[0] * DegreesToRadians), cosf(phiLimits[1] * DegreesToRadians))) * 1.005;
	if (phiLimits[0] < 0.0 && phiLimits[1] > 0.0)
	{
		// Zero distal arm angle is reachable
		maxRadius = proximalArmLength + distalArmLength;
	}
	else
	{
		const float minAngle = min<float>(fabs(phiLimits[0]), fabs(phiLimits[1])) * DegreesToRadians;
		maxRadius = sqrtf(proximalArmLengthSquared + distalArmLengthSquared + (twoPd * cosf(minAngle)));
	}
	maxRadius *= 0.995;
	minRadiusSquared = fsquare(minRadius);
	maxRadiusSquared = fsquare(maxRadius);
}

// End
