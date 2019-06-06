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

#include <limits>

ScaraKinematics::ScaraKinematics()
	: ZLeadscrewKinematics(KinematicsType::scara, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true),
	  proximalArmLength(DefaultProximalArmLength), distalArmLength(DefaultDistalArmLength), xOffset(0.0), yOffset(0.0)
{
	thetaLimits[0] = DefaultMinTheta;
	thetaLimits[1] = DefaultMaxTheta;
	psiLimits[0] = DefaultMinPsi;
	psiLimits[1] = DefaultMaxPsi;
	crosstalk[0] = crosstalk[1] = crosstalk[2] = requestedMinRadius = 0.0;
	Recalc();
}

// Return the name of the current kinematics
const char *ScaraKinematics::GetName(bool forStatusReport) const
{
	return "Scara";
}

// Calculate theta, psi and the new arm mode from a target position.
// If the position is not reachable because it is out of radius limits, set theta and psi to NaN and return false.
// Otherwise set theta and psi to the required values and return true if they are in range.
// Note: theta and psi are now returned in degrees.
bool ScaraKinematics::CalculateThetaAndPsi(const float machinePos[], bool isCoordinated, float& theta, float& psi, bool& armMode) const
{
	const float x = machinePos[X_AXIS] + xOffset;
	const float y = machinePos[Y_AXIS] + yOffset;
	const float cosPsi = (fsquare(x) + fsquare(y) - proximalArmLengthSquared - distalArmLengthSquared) / twoPd;

	// SCARA position is undefined if abs(SCARA_C2) >= 1. In reality abs(SCARA_C2) >0.95 can be problematic.
	const float square = 1.0 - fsquare(cosPsi);
	if (square < 0.01)
	{
		theta = psi = std::numeric_limits<float>::quiet_NaN();
		return false;		// not reachable
	}

	psi = acosf(cosPsi) * RadiansToDegrees;
	const float sinPsi = sqrtf(square);
	const float SCARA_K1 = proximalArmLength + distalArmLength * cosPsi;
	const float SCARA_K2 = distalArmLength * sinPsi;

	// Try the current arm mode, then the other one
	bool switchedMode = false;
	for (;;)
	{
		if (armMode != switchedMode)
		{
			// The following equations choose arm mode 0 i.e. distal arm rotated anticlockwise relative to proximal arm
			if (supportsContinuousRotation[1] || (psi >= psiLimits[0] && psi <= psiLimits[1]))
			{
				theta = atan2f(SCARA_K1 * y - SCARA_K2 * x, SCARA_K1 * x + SCARA_K2 * y) * RadiansToDegrees;
				if (supportsContinuousRotation[0] || (theta >= thetaLimits[0] && theta <= thetaLimits[1]))
				{
					break;
				}
			}
		}
		else
		{
			// The following equations choose arm mode 1 i.e. distal arm rotated clockwise relative to proximal arm
			if (supportsContinuousRotation[1] || ((-psi) >= psiLimits[0] && (-psi) <= psiLimits[1]))
			{
				theta = atan2f(SCARA_K1 * y + SCARA_K2 * x, SCARA_K1 * x - SCARA_K2 * y) * RadiansToDegrees;
				if (supportsContinuousRotation[0] || (theta >= thetaLimits[0] && theta <= thetaLimits[1]))
				{
					psi = -psi;
					break;
				}
			}
		}

		if (isCoordinated || switchedMode)
		{
			return false;		// not reachable
		}
		switchedMode = true;
	}

	// Now that we know we are going to do the move, update the arm mode
	if (switchedMode)
	{
		armMode = !armMode;
	}

	// Save the original and transformed coordinates so that we don't need to calculate them again if we are commanded to move to this position
	cachedX = machinePos[0];
	cachedY = machinePos[1];
	cachedTheta = theta;
	cachedPsi = psi;
	cachedArmMode = armMode;
	return true;
}

// Convert Cartesian coordinates to motor coordinates, returning true if successful
// In the following, theta is the proximal arm angle relative to the X axis, psi is the distal arm angle relative to the proximal arm
bool ScaraKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	float theta, psi;
	if (machinePos[0] == cachedX && machinePos[1] == cachedY)
	{
		theta = cachedTheta;
		psi = cachedPsi;
		currentArmMode = cachedArmMode;
	}
	else
	{
		bool armMode = currentArmMode;
		if (!CalculateThetaAndPsi(machinePos, isCoordinated, theta, psi, armMode))
		{
			return false;
		}
		currentArmMode = armMode;
	}

//debugPrintf("psi = %.2f, theta = %.2f\n", psi * RadiansToDegrees, theta * RadiansToDegrees);

	motorPos[X_AXIS] = lrintf(theta * stepsPerMm[X_AXIS]);
	motorPos[Y_AXIS] = lrintf((psi - (crosstalk[0] * theta)) * stepsPerMm[Y_AXIS]);
	motorPos[Z_AXIS] = lrintf((machinePos[Z_AXIS] - (crosstalk[1] * theta) - (crosstalk[2] * psi)) * stepsPerMm[Z_AXIS]);

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
	const float theta = ((float)motorPos[X_AXIS]/stepsPerMm[X_AXIS]);
    const float psi = ((float)motorPos[Y_AXIS]/stepsPerMm[Y_AXIS]) + (crosstalk[0] * theta);

    // Cache the current values so that a Z probe at this position won't fail due to rounding error when transforming the XY coordinates back
    currentArmMode = cachedArmMode = (motorPos[Y_AXIS] >= 0);
    cachedTheta = theta;
    cachedPsi = psi;
    cachedX = machinePos[X_AXIS] = (cosf(theta * DegreesToRadians) * proximalArmLength + cosf((psi + theta) * DegreesToRadians) * distalArmLength) - xOffset;
    cachedY = machinePos[Y_AXIS] = (sinf(theta * DegreesToRadians) * proximalArmLength + sinf((psi + theta) * DegreesToRadians) * distalArmLength) - yOffset;

    // On some machines (e.g. Helios), the X and/or Y arm motors also affect the Z height
    machinePos[Z_AXIS] = ((float)motorPos[Z_AXIS]/stepsPerMm[Z_AXIS]) + (crosstalk[1] * theta) + (crosstalk[2] * psi);

	// Convert any additional axes linearly
	for (size_t drive = XYZ_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool ScaraKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
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
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('B', 2, psiLimits, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('C', 3, crosstalk, reply, seen))
		{
			error = true;
			return true;
		}
		gb.TryGetFValue('R', requestedMinRadius, seen);

		if (seen || seenNonGeometry)
		{
			Recalc();
		}
		else if (!gb.Seen('K'))
		{
			reply.printf("Kinematics is Scara with proximal arm %.2fmm range %.1f to %.1f" DEGREE_SYMBOL
							"%s, distal arm %.2fmm range %.1f to %.1f" DEGREE_SYMBOL "%s, crosstalk %.1f:%.1f:%.1f, bed origin (%.1f, %.1f), segments/sec %d, min. segment length %.2f",
							(double)proximalArmLength, (double)thetaLimits[0], (double)thetaLimits[1], (supportsContinuousRotation[0]) ? " (continuous)" : "",
							(double)distalArmLength, (double)psiLimits[0], (double)psiLimits[1], (supportsContinuousRotation[0]) ? " (continuous)" : "",
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

// Return true if the specified XY position is reachable by the print head reference point, ignoring M208 limits.
bool ScaraKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	// See if we can transform the position
	float coords[2] = {x, y};
	float theta, psi;
	bool armMode = currentArmMode;
	return CalculateThetaAndPsi(coords, isCoordinated, theta, psi, armMode);
}

// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
LimitPositionResult ScaraKinematics::LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const
{
	// First limit all axes according to M208
	bool limited = applyM208Limits && Kinematics::LimitPositionFromAxis(finalCoords, 0, numVisibleAxes, axesHomed);

	// Now check whether the arms can reach the final position
	float theta, psi;
	bool armMode = currentArmMode;
	if (!CalculateThetaAndPsi(finalCoords, isCoordinated, theta, psi, armMode))
	{
		// The requested position was not reachable
		limited = true;
		if (std::isnan(theta))
		{
			// We are radius-limited
			float x = finalCoords[X_AXIS] + xOffset;
			float y = finalCoords[Y_AXIS] + yOffset;
			const float r = sqrtf(fsquare(x) + fsquare(y));
			if (r < minRadius)
			{
				// Radius is too small. The user may have specified x=0 y=0 so allow for this.
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
			}
			else
			{
				// Radius must be too large
				x *= maxRadius/r;
				y *= maxRadius/r;
			}

			finalCoords[X_AXIS] = x - xOffset;
			finalCoords[Y_AXIS] = y - yOffset;
		}

		// Recalculate theta and psi, but don't allow arm mode changes this time
		if (!CalculateThetaAndPsi(finalCoords, true, theta, psi, armMode) && !std::isnan(theta))
		{
			// Radius is in range but at least one arm angle isn't
			cachedTheta = theta = constrain<float>(theta, thetaLimits[0], thetaLimits[1]);
			cachedPsi = psi = constrain<float>(psi, psiLimits[0], psiLimits[1]);
			cachedX = finalCoords[X_AXIS] = (cosf(theta * DegreesToRadians) * proximalArmLength + cosf((psi + theta) * DegreesToRadians) * distalArmLength) - xOffset;
			cachedY = finalCoords[Y_AXIS] = (sinf(theta * DegreesToRadians) * proximalArmLength + sinf((psi + theta) * DegreesToRadians) * distalArmLength) - yOffset;
			cachedArmMode = currentArmMode;
		}
	}

	// The final position is now reachable. Check that we can get there from the initial position.
	if (isCoordinated && initialCoords != nullptr)
	{
		// Calculate how far along the line the closest point of approach to the distal axis is
		const float xdiff = finalCoords[0] - initialCoords[0];
		const float ydiff = finalCoords[1] - initialCoords[1];
		const float sumOfSquares = fsquare(xdiff) + fsquare(ydiff);
		const float p = -(xdiff * (initialCoords[0] + xOffset) + ydiff * (initialCoords[1] + yOffset));
		if (p > 0.0 && p < sumOfSquares)
		{
			// The closest point of approach to the distal axis is between the start and end points, so calculate the distance
			const float cpa2 = fsquare((finalCoords[0] + xOffset) * (initialCoords[1] + yOffset) - (finalCoords[1] + yOffset) * (initialCoords[0] + xOffset));
			if (cpa2 < minRadiusSquared * sumOfSquares)
			{
				return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
			}
		}
	}

	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
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

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap ScaraKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const
{
	constexpr AxesBitmap xyAxes = MakeBitmap<AxesBitmap>(X_AXIS) |  MakeBitmap<AxesBitmap>(Y_AXIS);
	if ((axesMoving & xyAxes) != 0)
	{
		axesMoving |= xyAxes;
	}
	return axesMoving;
}

size_t ScaraKinematics::NumHomingButtons(size_t numVisibleAxes) const
{
	const Platform& platform = reprap.GetPlatform();
	if (!platform.SysFileExists(HomeProximalFileName))
	{
		return 0;
	}
	if (!platform.SysFileExists(HomeDistalFileName))
	{
		return 1;
	}
	if (!platform.SysFileExists("homez.g"))
	{
		return 2;
	}
	return numVisibleAxes;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap ScaraKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const
{
	// Ask the base class which homing file we should call first
	AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);

	if (ret == 0)
	{
	// Change the returned name if it is X or Y
		if (StringEqualsIgnoreCase(filename.c_str(), "homex.g"))
		{
			filename.copy(HomeProximalFileName);
		}
		else if (StringEqualsIgnoreCase(filename.c_str(), "homey.g"))
		{
			filename.copy(HomeDistalFileName);
		}

		// Some SCARA printers cannot have individual axes homed safely. So it the user doesn't provide the homing file for an axis, default to homeall.
		if (!reprap.GetPlatform().SysFileExists(filename.c_str()))
		{
			filename.copy(HomeAllFileName);
		}
	}
	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool ScaraKinematics::QueryTerminateHomingMove(size_t axis) const
{
	// If crosstalk causes the axis motor concerned to affect other axes then must terminate the entire move
	return (axis == X_AXIS && (crosstalk[0] != 0.0 || crosstalk[1] != 0.0))
		|| (axis == Y_AXIS && crosstalk[2] != 0.0);
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate().
void ScaraKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	switch (axis)
	{
	case X_AXIS:	// proximal joint homing switch
		{
			const float hitPoint = (highEnd) ? thetaLimits[1] : thetaLimits[0];
			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
		}
		break;

	case Y_AXIS:	// distal joint homing switch
		{
			const float hitPoint = ((highEnd) ? psiLimits[1] : psiLimits[0])
									- ((dda.DriveCoordinates()[X_AXIS] * crosstalk[0])/stepsPerMm[X_AXIS]);
			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
		}
		break;

	case Z_AXIS:	// Z axis homing switch
		{
			const float hitPoint = ((highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis))
									- ((dda.DriveCoordinates()[X_AXIS] * crosstalk[1])/stepsPerMm[X_AXIS])
									- ((dda.DriveCoordinates()[Y_AXIS] * crosstalk[2])/stepsPerMm[Y_AXIS]);
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
void ScaraKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const
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
bool ScaraKinematics::IsContinuousRotationAxis(size_t axis) const
{
	return axis < 2 && supportsContinuousRotation[axis];
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap ScaraKinematics::GetLinearAxes() const
{
	return (crosstalk[1] == 0.0 && crosstalk[2] == 0.0) ? MakeBitmap<AxesBitmap>(Z_AXIS) : 0;
}

// Recalculate the derived parameters
void ScaraKinematics::Recalc()
{
	proximalArmLengthSquared = fsquare(proximalArmLength);
	distalArmLengthSquared = fsquare(distalArmLength);
	twoPd = proximalArmLength * distalArmLength * 2;

	minRadius = max<float>(sqrtf(proximalArmLengthSquared + distalArmLengthSquared
							- twoPd * max<float>(cosf(psiLimits[0] * DegreesToRadians), cosf(psiLimits[1] * DegreesToRadians))) * 1.005,
							requestedMinRadius);
	minRadiusSquared = fsquare(minRadius);

	// If the total angle range is greater than 360 degrees, we assume that it supports continuous rotation
	supportsContinuousRotation[0] = (thetaLimits[1] - thetaLimits[0] > 360.0);
	supportsContinuousRotation[1] = (psiLimits[1] - psiLimits[0] > 360.0);

	if (supportsContinuousRotation[1] || (psiLimits[0] <= 0.0 && psiLimits[1] >= 0.0))
	{
		// Zero distal arm angle is reachable
		maxRadius = proximalArmLength + distalArmLength;
	}
	else
	{
		const float minAngle = min<float>(fabsf(psiLimits[0]), fabsf(psiLimits[1])) * DegreesToRadians;
		maxRadius = sqrtf(proximalArmLengthSquared + distalArmLengthSquared + (twoPd * cosf(minAngle)));
	}
	maxRadius *= 0.995;

	cachedX = cachedY = std::numeric_limits<float>::quiet_NaN();		// make sure that the cached values won't match any coordinates
}

// End
