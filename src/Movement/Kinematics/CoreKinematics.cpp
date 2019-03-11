/*
 * CoreKinematics.cpp
 *
 *  Created on: 4 Jan 2019
 *      Author: David
 */

#include "CoreKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodes.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/DDA.h"

// Recalculate internal variables following a configuration change
void CoreKinematics::Recalc()
{
	// Calculate the forward matrix by inverting the inverse matrix
	{
		// Set up a double-width matrix with the inverse matrix in the left half and a unit diagonal matrix in the right half
		FixedMatrix<float, MaxAxes, 2 * MaxAxes> tempMatrix;
		for (size_t i = 0; i < MaxAxes; ++i)
		{
			for (size_t j = 0; j < MaxAxes; ++j)
			{
				tempMatrix(i, j) = inverseMatrix(i, j);
			}
			for (size_t j = MaxAxes; j < 2 * MaxAxes; ++j)
			{
				tempMatrix(i, j) = 0.0;
			}
			tempMatrix(i, i + MaxAxes) = 1.0;
		}

		// Apply the Gauss-Jordan operation to transform the right half into the inverse of the inverse matrix
		const bool ok = tempMatrix.GaussJordan(MaxAxes, 2 * MaxAxes);
		if (ok)
		{
			// Copy the right half to the forward matrix
			for (size_t i = 0; i < MaxAxes; ++i)
			{
				for (size_t j = 0; j < MaxAxes; ++j)
				{
					forwardMatrix(i, j) = tempMatrix(i, j + MaxAxes);
				}
			}
		}
		else
		{
			forwardMatrix.Fill(0.0);
			reprap.GetPlatform().Message(ErrorMessage, "Invalid kinematics matrix\n");
		}
	}

	// Calculate the first and last motors for each axis, and first and last axis controlled by each motor.
	// These are used to optimise calculations and homing behaviour.
	// It doesn't matter if an axis doesn't actually use all the motors from its first to its last inclusive.
	// Also determine which motors are shared by two or more axes.
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		firstMotor[i] = firstAxis[i] = MaxAxes;
		lastMotor[i] = lastAxis[i] = 0;
		connectedAxes[i] = 0;
	}

	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		for (size_t motor = 0; motor < MaxAxes; ++motor)
		{
			if (inverseMatrix(axis, motor) != 0.0)							// if this axis needs this motor driven
			{
				if (axis < firstAxis[motor])
				{
					firstAxis[motor] = axis;
				}
				if (axis > lastAxis[motor])
				{
					lastAxis[motor] = axis;
				}
				SetBit(connectedAxes[axis], motor);
			}

			if (forwardMatrix(motor, axis) != 0.0)							// if this motor affects this axes
			{
				if (motor < firstMotor[axis])
				{
					firstMotor[axis] = motor;
				}
				if (motor > lastMotor[axis])
				{
					lastMotor[axis] = motor;
				}
				SetBit(connectedAxes[axis], motor);
			}
		}
	}

	if (reprap.Debug(moduleMove))
	{
		PrintMatrix("Inverse", inverseMatrix);
		PrintMatrix("Forward", forwardMatrix);

		String<MediumStringLength> s;
		s.copy("First/last motors:");
		for (size_t axis = 0; axis < MaxAxes; ++axis)
		{
			s.catf(" %u/%u", firstMotor[axis], lastMotor[axis]);
		}
		debugPrintf("%s\n", s.c_str());

		s.copy("First/last axes:");
		for (size_t motor = 0; motor < MaxAxes; ++motor)
		{
			s.catf(" %u/%u", firstAxis[motor], lastAxis[motor]);
		}
		debugPrintf("%s\n", s.c_str());
	}
}

// Return true if the axis doesn't have a single dedicated motor
inline bool CoreKinematics::HasSharedMotor(size_t axis) const
{
	return connectedAxes[axis] != MakeBitmap<AxesBitmap>(axis);
}

CoreKinematics::CoreKinematics(KinematicsType k) : ZLeadscrewKinematics(k), modified(false)
{
	// Start by assuming 1:1 mapping of axes to motors by setting diagonal elements to 1 and other element to zero
	inverseMatrix.Fill(0.0);
	for (size_t i = 0; i < MaxAxes; ++i)
	{
		inverseMatrix(i, i) = 1.0;
	}

	switch (k)
	{
	case KinematicsType::cartesian:
	default:
		break;

	case KinematicsType::coreXY:
		inverseMatrix(0, 1) = 1.0;
		inverseMatrix(1, 0) = 1.0;
		inverseMatrix(1, 1) = -1.0;
		break;

	case KinematicsType::coreXYU:
		// Core XYU is like CoreXY with an additional U axis controlled by the U and V motors
		inverseMatrix(0, 1) = 1.0;
		inverseMatrix(1, 0) = 1.0;
		inverseMatrix(1, 1) = -1.0;
		inverseMatrix(1, 3) = 1.0;
		inverseMatrix(1, 4) = -1.0;
		inverseMatrix(3, 4) = 1.0;
		inverseMatrix(4, 4) = 0.0;		// V can't be commanded directly
		break;

	case KinematicsType::coreXYUV:
		// CoreXYUV is a dual CoreXY setup
		inverseMatrix(0, 1) = 1.0;
		inverseMatrix(1, 0) = 1.0;
		inverseMatrix(1, 1) = -1.0;
		inverseMatrix(3, 4) = 1.0;
		inverseMatrix(4, 3) = 1.0;
		inverseMatrix(4, 4) = -1.0;
		break;

	case KinematicsType::coreXZ:
		inverseMatrix(0, 2) = 1.0;
		inverseMatrix(2, 0) = 3.0;
		inverseMatrix(2, 2) = -3.0;
		break;

	case KinematicsType::markForged:
		inverseMatrix(1, 0) = -1.0;
		break;
	}

	Recalc();
}

// Return the name of the current kinematics
const char* CoreKinematics::GetName(bool forStatusReport) const
{
	// This reports the original kinematics that was requested. It doesn't allow for the matrix having been patched to change the kinematics.
	switch (GetKinematicsType())
	{
	case KinematicsType::cartesian:
		return (forStatusReport) ? "cartesian" : "Cartesian";

	case KinematicsType::coreXY:
		return (forStatusReport) ? "coreXY" : "CoreXY";

	case KinematicsType::coreXYU:
		return (forStatusReport) ? "coreXYU" : "CoreXYU";

	case KinematicsType::coreXYUV:
		return (forStatusReport) ? "coreXYUV" : "CoreXYUV";
		break;

	case KinematicsType::coreXZ:
		return (forStatusReport) ? "coreXZ" : "CoreXZ";

	case KinematicsType::markForged:
		return "markForged";

	default:
		return "unknown";
	}
}

// Set the parameters from a M665, M666, M667 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
// This function is used for CoreXY and CoreXZ kinematics, but it overridden for CoreXYU kinematics
bool CoreKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	bool seen;
	switch (mCode)
	{
	case 669:
		{
			seen = gb.Seen('K');
			const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
			for (size_t axis = 0; axis < numVisibleAxes; ++axis)
			{
				if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
				{
					seen = true;
					float motorFactors[MaxAxes];
					size_t numMotors = MaxAxes;
					gb.GetFloatArray(motorFactors, numMotors, false);
					for (size_t m = 0; m < numMotors; ++m)
					{
						if (inverseMatrix(axis, m) != motorFactors[m])
						{
							inverseMatrix(axis, m) = motorFactors[m];
							modified = true;
						}
					}
					for (size_t m = numMotors; m < MaxAxes; ++m)
					{
						if (inverseMatrix(axis, m) != 0.0)
						{
							inverseMatrix(axis, m) = 0.0;
							modified = true;
						}
					}
				}
			}
		}
		break;

	default:
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}

	if (seen)
	{
		Recalc();
	}
	else
	{
		reply.printf("Kinematics is %s%s, matrix:", ((modified) ? "modified " : ""), GetName(false));
		const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			for (size_t motor = 0; motor < numTotalAxes; ++motor)
			{
				reply.cat((motor == 0) ? '\n' :  ' ');
				const float val = inverseMatrix(axis, motor);
				if (val == 0.0)
				{
					reply.cat('0');						// don't print unnecessary decimals, we will probably reach the response buffer limit if we do
				}
				else
				{
					reply.catf("%.2f", (double)val);
				}
			}
		}
	}
	return seen;
}

// Convert Cartesian coordinates to motor coordinates returning true if successful.
// This is called frequently, so try to keep it efficient.
// If a motor has no visible axes that affect it, leave the old motor coordinate unchanged.
bool CoreKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes,
	int32_t motorPos[], bool isCoordinated) const
{
	for (size_t motor = 0; motor < numTotalAxes; ++motor)
	{
		const size_t axisLimit = min<size_t>(numVisibleAxes, lastAxis[motor] + 1);
		size_t axis = firstAxis[motor];
		if (axis < axisLimit)
		{
			float movement = inverseMatrix(axis, motor) * machinePos[axis];
			++axis;
			while (axis < axisLimit)
			{
				movement += inverseMatrix(axis, motor) * machinePos[axis];
				++axis;
			}
			motorPos[motor] = lrintf(movement * stepsPerMm[motor]);
		}
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void CoreKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	// If there are more motors than visible axes (e.g. CoreXYU which has a V motor), we assume that we can ignore the trailing ones when calculating the machine position
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		float position = 0.0;
		const size_t motorLimit = min<size_t>(numVisibleAxes, lastMotor[axis] + 1);
		for (size_t motor = firstMotor[axis]; motor < motorLimit; ++motor)
		{
			const float factor = forwardMatrix(motor, axis);
			if (factor != 0.0)
			{
				position += factor * (float)motorPos[motor] / stepsPerMm[motor];
			}
		}
		machinePos[axis] = position;
	}
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool CoreKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return HasSharedMotor(axis);
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void CoreKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
	if (HasSharedMotor(axis))
	{
		float tempCoordinates[MaxAxes];
		const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t axis = 0; axis < numTotalAxes; ++axis)
		{
			tempCoordinates[axis] = dda.GetEndCoordinate(axis, false);
		}
		tempCoordinates[axis] = hitPoint;
		dda.SetPositions(tempCoordinates, numTotalAxes);
	}
	else
	{
		dda.SetDriveCoordinate(lrintf(hitPoint * inverseMatrix(axis, axis) * stepsPerMm[axis]), axis);
	}
}

// Limit the speed and acceleration of a move to values that the mechanics can handle
// The speeds along individual Cartesian axes have already been limited before this is called, so we need only be concerned with shared motors
void CoreKinematics::LimitSpeedAndAcceleration(DDA& dda, const float* normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const
{
	// For each shared motor, calculate how much of the total move it contributes
	float motorMovements[MaxAxes];
	for (float& mm : motorMovements)
	{
		mm = 0.0;
	}

	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (HasSharedMotor(axis))
		{
			const float dv = normalisedDirectionVector[axis];
			if (dv != 0.0)
			{
				for (size_t motor = 0; motor < MaxAxes; ++motor)
				{
					const float factor = inverseMatrix(axis, motor);
					if (factor != 0.0)
					{
						motorMovements[motor] += factor * dv;
					}
				}
			}
		}
	}

	for (size_t motor = 0; motor < MaxAxes; ++motor)
	{
		const float mm = fabsf(motorMovements[motor]);
		if (mm != 0.0)
		{
			dda.LimitSpeedAndAcceleration(reprap.GetPlatform().MaxFeedrate(motor)/mm, reprap.GetPlatform().Acceleration(motor)/mm);
		}
	}
}

// Return a bitmap of the motors that are involved in homing a particular axis or tower. Used for implementing stall detection endstops.
// Usually it is just the corresponding motor (hence this default implementation), but CoreXY and similar kinematics move multiple motors to home an individual axis.
AxesBitmap CoreKinematics::GetConnectedAxes(size_t axis) const
{
	return connectedAxes[axis];
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap CoreKinematics::GetLinearAxes() const
{
	return LowestNBits<AxesBitmap>(reprap.GetGCodes().GetVisibleAxes());	// we can babystep all axes
}

// End
