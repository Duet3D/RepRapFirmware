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

// quadrants: 1 is right upper, 2 is left upper, 3 is left down, 4 is right down
int FiveBarScaraKinematics::getQuadrant(int x, int y) const
{
  return 1;
}






///////////////////////////// public functions /////////////////////////

// Convert Cartesian coordinates to motor coordinates, returning true if successful
// In the following, theta is the proximal arm angle relative to the X axis, psi is the distal arm angle relative to the proximal arm
bool FiveBarScaraKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// For Scara, the X and Y components of stepsPerMm are actually steps per degree angle.
void FiveBarScaraKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
}

// Return true if the specified XY position is reachable by the print head reference point.
bool FiveBarScaraKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	return true;
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
	if (!storage->FileExists(SYS_DIR, Home5BarScaraFileName))
	{
		return 1;
	}
	if (!storage->FileExists(SYS_DIR, "homez.g"))
	{
		return 2;
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

	if (ret == 0)
	{
	// Change the returned name if it is X or Y
		if (StringEquals(filename.c_str(), "homex.g"))
		{
			filename.copy(Home5BarScaraFileName);
		}
		else if (StringEquals(filename.c_str(), "homey.g"))
		{
			filename.copy(Home5BarScaraFileName);
		}

		// Some SCARA printers cannot have individual axes homed safely. So it the user doesn't provide the homing file for an axis, default to homeall.
		const MassStorage *storage = reprap.GetPlatform().GetMassStorage();
		if (!storage->FileExists(SYS_DIR, filename.c_str()))
		{
			filename.copy(HomeAllFileName);
		}
	}
	return ret;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool FiveBarScaraKinematics::QueryTerminateHomingMove(size_t axis) const
{
	// If crosstalk causes the axis motor concerned to affect other axes then must terminate the entire move
//	return (axis == X_AXIS && (crosstalk[0] != 0.0 || crosstalk[1] != 0.0))
//		|| (axis == Y_AXIS && crosstalk[2] != 0.0);
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void FiveBarScaraKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
//	switch (axis)
//	{
//	case X_AXIS:	// proximal joint homing switch
//		{
//			const float hitPoint = (highEnd) ? thetaLimits[1] : thetaLimits[0];
//			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
//		}
//		break;
//
//	case Y_AXIS:	// distal joint homing switch
//		{
//			const float hitPoint = ((highEnd) ? psiLimits[1] : psiLimits[0])
//									- ((dda.DriveCoordinates()[X_AXIS] * crosstalk[0])/stepsPerMm[X_AXIS]);
//			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
//		}
//		break;
//
//	case Z_AXIS:	// Z axis homing switch
//		{
//			const float hitPoint = ((highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis))
//									- ((dda.DriveCoordinates()[X_AXIS] * crosstalk[1])/stepsPerMm[X_AXIS])
//									- ((dda.DriveCoordinates()[Y_AXIS] * crosstalk[2])/stepsPerMm[Y_AXIS]);
//			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
//		}
//		break;
//
//	default:		// Additional axis
//		{
//			const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
//			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
//		}
//		break;
//	}
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
