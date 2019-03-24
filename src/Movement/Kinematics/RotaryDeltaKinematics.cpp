/*
 * RotaryDeltaKinematics.cpp
 *
 *  Created on: 1 Aug 2018
 *      Author: David
 */

#include "RotaryDeltaKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "Movement/DDA.h"
#include "GCodes/GCodeBuffer.h"

const float RotaryDeltaKinematics::NormalTowerAngles[DELTA_AXES] = { -150.0, -30.0, 90.0 };

// Constructor
RotaryDeltaKinematics::RotaryDeltaKinematics() : Kinematics(KinematicsType::rotaryDelta, DefaultSegmentsPerSecond, DefaultMinSegmentSize, false)
{
	Init();
}

// Initialise parameters to defaults
void RotaryDeltaKinematics::Init()
{
	radius = DefaultDeltaRadius;
	printRadius = DefaultPrintRadius;
	minArmAngle = DefaultMinArmAngle;
	maxArmAngle = DefaultMaxArmAngle;

	for (size_t axis = 0; axis < DELTA_AXES; ++axis)
	{
		armLengths[axis] = DefaultArmLength;
		rodLengths[axis] = DefaultRodLength;
		bearingHeights[axis] = DefaultBearingHeight;
		angleCorrections[axis] = 0.0;
		endstopAdjustments[axis] = 0.0;
	}

	Recalc();
}

// Compute the derived parameters from the primary parameters
void RotaryDeltaKinematics::Recalc()
{
	printRadiusSquared = fsquare(printRadius);
	for (size_t axis = 0; axis < DELTA_AXES; ++axis)
	{
		const float angle = (NormalTowerAngles[axis] + angleCorrections[axis]) * DegreesToRadians;
		armAngleSines[axis] = sinf(angle);
		armAngleCosines[axis] = cosf(angle);
		twiceU[axis] = armLengths[axis] * 2;
		rodSquared[axis] = fsquare(rodLengths[axis]);
		rodSquaredMinusArmSquared[axis] = rodSquared[axis] - fsquare(armLengths[axis]);
	}
}

// Return the name of the current kinematics.
// If 'forStatusReport' is true then the string must be the one for that kinematics expected by DuetWebControl and PanelDue.
// Otherwise it should be in a format suitable for printing.
const char *RotaryDeltaKinematics::GetName(bool forStatusReport) const
{
	return "Rotary delta";
}

// Set or report the parameters from a M665, M666 or M669 command
// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 665, 666, 667 or 669)
// then search for parameters used to configure the current kinematics. If any are found, perform appropriate actions,
// and return true if the changes affect the geometry.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
// If 'mCode' does not apply to this kinematics, call the base class version of this function, which will print a suitable error message.
bool RotaryDeltaKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	switch(mCode)
	{
	case 669:
		{
			bool seen = false;
			size_t numValues = 3;
			if (gb.TryGetFloatArray('U', numValues, armLengths, reply, seen, true))
			{
				error = true;
				return true;
			}

			numValues = 3;
			if (gb.TryGetFloatArray('L', numValues, rodLengths, reply, seen, true))
			{
				error = true;
				return true;
			}

			numValues = 3;
			if (gb.TryGetFloatArray('H', numValues, bearingHeights, reply, seen, true))
			{
				error = true;
				return true;
			}

			numValues = 2;
			if (gb.TryGetFloatArray('A', numValues, minMaxArmAngles, reply, seen, false))
			{
				error = true;
				return true;
			}

			gb.TryGetFValue('R', radius, seen);
			if (gb.Seen('B'))
			{
				printRadius = gb.GetFValue();
				// Set the axis limits so that DWC reports them correctly (they are not otherwise used for deltas, except Z min)
				Platform& p = reprap.GetPlatform();
				p.SetAxisMinimum(X_AXIS, -printRadius, false);
				p.SetAxisMinimum(Y_AXIS, -printRadius, false);
				p.SetAxisMaximum(X_AXIS, printRadius, false);
				p.SetAxisMaximum(Y_AXIS, printRadius, false);
				seen = true;
			}

			gb.TryGetFValue('X', angleCorrections[DELTA_A_AXIS], seen);
			gb.TryGetFValue('Y', angleCorrections[DELTA_B_AXIS], seen);
			gb.TryGetFValue('Z', angleCorrections[DELTA_C_AXIS], seen);

			if (seen)
			{
				Recalc();
			}
			else
			{
				reply.printf("Kinematics is rotary delta, arms (%.3f,%.2f,%.3f)mm, rods (%.3f,%.3f,%.3f)mm, bearingHeights (%.3f,%.2f,%.3f)mm"
							 ", arm movement %.1f to %.1f" DEGREE_SYMBOL
							 ", delta radius %.3f, bed radius %.1f"
							 ", angle corrections (%.3f,%.3f,%.3f)" DEGREE_SYMBOL ,
							 (double)armLengths[DELTA_A_AXIS], (double)armLengths[DELTA_B_AXIS], (double)armLengths[DELTA_C_AXIS],
							 (double)rodLengths[DELTA_A_AXIS], (double)rodLengths[DELTA_B_AXIS], (double)rodLengths[DELTA_C_AXIS],
							 (double)bearingHeights[DELTA_A_AXIS], (double)bearingHeights[DELTA_B_AXIS], (double)bearingHeights[DELTA_C_AXIS],
							 (double)minArmAngle, (double)maxArmAngle,
							 (double)radius, (double)printRadius,
							 (double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
			}
			return seen;
		}

	case 666:
		{
			bool seen = false;
			gb.TryGetFValue('X', endstopAdjustments[DELTA_A_AXIS], seen);
			gb.TryGetFValue('Y', endstopAdjustments[DELTA_B_AXIS], seen);
			gb.TryGetFValue('Z', endstopAdjustments[DELTA_C_AXIS], seen);

			if (!seen)
			{
				reply.printf("Endstop adjustments X%.2f Y%.2f Z%.2f" DEGREE_SYMBOL,
					(double)endstopAdjustments[X_AXIS], (double)endstopAdjustments[Y_AXIS], (double)endstopAdjustments[Z_AXIS]);
			}
			return seen;
		}

	default:
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Convert Cartesian coordinates to motor positions measured in steps from reference position
// 'machinePos' is a set of axis and extruder positions to convert
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numAxes' is the number of machine axes to convert, which will always be at least 3
// 'motorPos' is the output vector of motor positions
// Return true if successful, false if we were unable to convert
bool RotaryDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	bool ok = true;
	for (size_t axis = 0; axis < min<size_t>(numVisibleAxes, DELTA_AXES); ++axis)
	{
		const float pos = Transform(machinePos, axis);
		if (isnan(pos) || isinf(pos))
		{
			ok = false;
		}
		else
		{
			motorPos[axis] = lrintf(pos * stepsPerMm[axis]);
		}
	}

	// TEMP DEBUG
	if (reprap.Debug(moduleMove))
	{
		debugPrintf("Transformed %.2f,%.2f,%.2f mm to %" PRIi32 ",%" PRIi32 ",%" PRIi32 " steps %s\n",
			(double)machinePos[0], (double)machinePos[1], (double)machinePos[2],
			motorPos[0], motorPos[1], motorPos[2],
			(ok) ? "ok" : "fail");
	}

	// Transform any additional axes linearly
	for (size_t axis = DELTA_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return ok;
}

// Convert motor positions (measured in steps from reference position) to Cartesian coordinates
// 'motorPos' is the input vector of motor positions
// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
// 'numDrives' is the number of machine drives to convert, which will always be at least 3
// 'machinePos' is the output set of converted axis and extruder positions
void RotaryDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	ForwardTransform(motorPos[DELTA_A_AXIS]/stepsPerMm[DELTA_A_AXIS], motorPos[DELTA_B_AXIS]/stepsPerMm[DELTA_B_AXIS], motorPos[DELTA_C_AXIS]/stepsPerMm[DELTA_C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = DELTA_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Perform auto calibration. Caller already owns the movement lock.
// Return true if an error occurred.
bool RotaryDeltaKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply)
{
	return true;	// auto calibration not implemented yet
}

// Write the parameters that are set by auto calibration to a file, returning true if success
bool RotaryDeltaKinematics::WriteCalibrationParameters(FileStore *f) const
{
	return true;	// auto calibration not implemented yet
}

// Return true if the specified XY position is reachable by the print head reference point.
bool RotaryDeltaKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
bool RotaryDeltaKinematics::LimitPosition(float finalCoords[], float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const
{
	const AxesBitmap allAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS);
	bool limited = false;
	if ((axesHomed & allAxes) == allAxes)
	{
		// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
		// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
		if (diagonalSquared > printRadiusSquared)
		{
			const float factor = sqrtf(printRadiusSquared / diagonalSquared);
			finalCoords[X_AXIS] *= factor;
			finalCoords[Y_AXIS] *= factor;
			limited = true;
		}

		if (finalCoords[Z_AXIS] < reprap.GetPlatform().AxisMinimum(Z_AXIS))
		{
			finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMinimum(Z_AXIS);
			limited = true;
		}
		else if (finalCoords[Z_AXIS] > reprap.GetPlatform().AxisMaximum(Z_AXIS))
		{
			finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMaximum(Z_AXIS);
			limited = true;
		}
	}

	// Limit any additional axes according to the M208 limits
	if (applyM208Limits && LimitPositionFromAxis(finalCoords, Z_AXIS + 1, numVisibleAxes, axesHomed))
	{
		limited = true;
	}

	return limited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void RotaryDeltaKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	ForwardTransform(0.0, 0.0, 0.0, positions);			// assume that the arms are horizontal
	for (size_t i = DELTA_AXES; i < numAxes; ++i)
	{
		positions[i] = 0.0;								// set additional axes to zero
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap RotaryDeltaKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const
{
	// If all of X, Y and Z have been specified then we know the positions of all 3 tower motors, otherwise we don't
	constexpr AxesBitmap xyzAxes = MakeBitmap<AxesBitmap>(X_AXIS) |  MakeBitmap<AxesBitmap>(Y_AXIS) |  MakeBitmap<AxesBitmap>(Z_AXIS);
	if ((g92Axes & xyzAxes) != xyzAxes)
	{
		g92Axes &= ~xyzAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap RotaryDeltaKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const
{
	constexpr AxesBitmap xyzAxes = MakeBitmap<AxesBitmap>(X_AXIS) |  MakeBitmap<AxesBitmap>(Y_AXIS) |  MakeBitmap<AxesBitmap>(Z_AXIS);
	if ((axesMoving & xyzAxes) != 0)
	{
		axesMoving |= xyzAxes;
	}
	return axesMoving;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap RotaryDeltaKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const
{
	// If homing X, Y or Z we must home all the towers
	if ((toBeHomed & LowestNBits<AxesBitmap>(DELTA_AXES)) != 0)
	{
		filename.copy("homedelta.g");
		return 0;
	}

	return Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool RotaryDeltaKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate().
void RotaryDeltaKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	if (axis < DELTA_AXES)
	{
		if (highEnd)
		{
			const float hitPoint = maxArmAngle + endstopAdjustments[axis];
			dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
		}
	}
	else
	{
		// Assume that any additional axes are linear
		const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
	}
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool RotaryDeltaKinematics::WriteResumeSettings(FileStore *f) const
{
//	return !doneAutoCalibration || WriteCalibrationParameters(f);
	return true;	// auto calibration not implemented yet
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void RotaryDeltaKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const
{
	// Limit the speed in the XY plane to the lower of the X and Y maximum speeds, and similarly for the acceleration
	const float xyFactor = sqrtf(fsquare(normalisedDirectionVector[X_AXIS]) + fsquare(normalisedDirectionVector[Y_AXIS]));
	if (xyFactor > 0.01)
	{
		const Platform& platform = reprap.GetPlatform();
		const float maxSpeed = min<float>(platform.MaxFeedrate(X_AXIS), platform.MaxFeedrate(Y_AXIS));
		const float maxAcceleration = min<float>(platform.Acceleration(X_AXIS), platform.Acceleration(Y_AXIS));
		dda.LimitSpeedAndAcceleration(maxSpeed/xyFactor, maxAcceleration/xyFactor);
	}
}

// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
// This is called to determine whether we can babystep the specified axis independently of regular motion.
AxesBitmap RotaryDeltaKinematics::GetLinearAxes() const
{
	return 0;
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
// If we first transform the XY coordinates so that +X is along the direction of the arm, then we need to solve this equation:
//  L^2 = (U cos(theta) + (R - x))^2 + y^2 + (U sin(theta) + (H - z))^2
// Expanding this and using cos^2(theta) + sin^2(theta) = 1 we get:
//  L^2 = U^2 + (H - z)^2 + (R - x)^2 + y^2 + 2U(R - x)cos(theta) + 2U(H - z)sin(theta)
// This is of the form:
//  a cos(theta) + b sin(theta) = c
// where:
//  a = 2U(r - x), b = 2U(H - z), c = L^2 - U^2 - (H - z)^2 - (R - x)^2 - y^2
// We can rearrange, square and use sin^2 + cos^2 = 1 again to get this:
//  (a^2 + b^2)sin^2(theta) - 2bc sin(theta) + (c^2 - a^2) = 0
// which has solutions:
// sin(theta) = (bc +/- a sqrt(a^2 + b^2 - c^2))/(a^2 + b^2)
float RotaryDeltaKinematics::Transform(const float machinePos[], size_t axis) const
{
	if (axis < DELTA_AXES)
	{
		// 1. Transform the X and Y coordinates so that +X is along the arm and +Y is 90deg anticlockwise from +X
		const float x = machinePos[X_AXIS] * armAngleCosines[axis] + machinePos[Y_AXIS] * armAngleSines[axis];
		const float y = machinePos[Y_AXIS] * armAngleCosines[axis] - machinePos[X_AXIS] * armAngleSines[axis];

		// 2. Calculate a, b and c
		const float rMinusX = radius - x;
		const float hMinusZ = bearingHeights[axis] - machinePos[Z_AXIS];
		const float a = twiceU[axis] * rMinusX;
		const float b = twiceU[axis] * hMinusZ;
		const float c = rodSquaredMinusArmSquared[axis] - (fsquare(hMinusZ) + fsquare(rMinusX) + fsquare(y));

		// 3. Solve the quadratic equation, taking the lower root
		const float sinTheta = (b * c - a * sqrtf(fsquare(a) + fsquare(b) - fsquare(c)))/(fsquare(a) + fsquare(b));

		// 4. Take the arc sine and convert to degrees
		return asinf(sinTheta) * RadiansToDegrees;
	}
	else
	{
		return machinePos[axis];		// any additional axes must be linear
	}
}

// Calculate the Cartesian coordinates from the motor coordinates. We do this by trilateration.
void RotaryDeltaKinematics::ForwardTransform(float Ha, float Hb, float Hc, float machinePos[DELTA_AXES]) const
{
	// Calculate the Cartesian coordinates of the joints at the moving ends of the arms
	const float angleA = Ha * DegreesToRadians;
	const float posAX = (radius + (armLengths[DELTA_A_AXIS] * cosf(angleA))) * armAngleCosines[DELTA_A_AXIS];
	const float posAY = (radius + (armLengths[DELTA_A_AXIS] * cosf(angleA))) * armAngleSines[DELTA_A_AXIS];
	const float posAZ = bearingHeights[DELTA_A_AXIS] + (armLengths[DELTA_A_AXIS] * sinf(angleA));

	const float angleB = Hb * DegreesToRadians;
	const float posBX = (radius + (armLengths[DELTA_B_AXIS] * cosf(angleB))) * armAngleCosines[DELTA_B_AXIS];
	const float posBY = (radius + (armLengths[DELTA_B_AXIS] * cosf(angleB))) * armAngleSines[DELTA_B_AXIS];
	const float posBZ = bearingHeights[DELTA_B_AXIS] + (armLengths[DELTA_A_AXIS] * sinf(angleB));

	const float angleC = Hc * DegreesToRadians;
	const float posCX = (radius + (armLengths[DELTA_C_AXIS] * cosf(angleC))) * armAngleCosines[DELTA_C_AXIS];
	const float posCY = (radius + (armLengths[DELTA_C_AXIS] * cosf(angleC))) * armAngleSines[DELTA_C_AXIS];
	const float posCZ = bearingHeights[DELTA_C_AXIS] + (armLengths[DELTA_A_AXIS] * sinf(angleC));

	// Calculate some intermediate values that we will use more than once
	const float Da2 = fsquare(posAX) + fsquare(posAY) + fsquare(posAZ);
	const float Db2 = fsquare(posBX) + fsquare(posBY) + fsquare(posBZ);
	const float Dc2 = fsquare(posCX) + fsquare(posCY) + fsquare(posCZ);

	// Calculate PQRST such that x = (Qz + S)/P, y = (Rz + T)/P.
	const float P = (posBX * posCY - posAX * posCY - posCX * posBY + posAX * posBY + posCX * posAY - posBX * posAY) * 2;
	const float Q = ((posBY - posAY) * posCZ + (posAY - posCY) * posBZ + (posCY - posBY) * posAZ) * 2;
	const float R = ((posBX - posAX) * posCZ + (posAX - posCX) * posBZ + (posCX - posBX) * posAZ) * 2;

	const float S =   (rodSquared[DELTA_A_AXIS] - rodSquared[DELTA_B_AXIS] + Db2 - Da2) * posCY
					+ (rodSquared[DELTA_C_AXIS] - rodSquared[DELTA_A_AXIS] + Da2 - Dc2) * posBY
					+ (rodSquared[DELTA_B_AXIS] - rodSquared[DELTA_C_AXIS] + Dc2 - Db2) * posAY;
	const float T =   (rodSquared[DELTA_A_AXIS] - rodSquared[DELTA_B_AXIS] + Db2 - Da2) * posCX
					+ (rodSquared[DELTA_C_AXIS] - rodSquared[DELTA_A_AXIS] + Da2 - Dc2) * posBX
					+ (rodSquared[DELTA_B_AXIS] - rodSquared[DELTA_C_AXIS] + Dc2 - Db2) * posAX;

	// Calculate quadratic equation coefficients
	const float P2 = fsquare(P);
	const float A = P2 + fsquare(Q) + fsquare(R);
	const float halfB = (P * R * posAY) - (P2 * posAZ) - (P * Q * posAX) + (R * T) + (Q * S);
	const float C = fsquare(S) + fsquare(T) + (T * posAY - S * posAX) * P * 2 + (Da2 - rodSquared[DELTA_A_AXIS]) * P2;

	// Solve the quadratic equation for z
	const float z = (- halfB - sqrtf(fsquare(halfB) - A * C))/A;

	// Substitute back for X and Y
	machinePos[X_AXIS] = (Q * z + S)/P;
	machinePos[Y_AXIS] = (R * z + T)/P;
	machinePos[Z_AXIS] = z;

	if (reprap.Debug(moduleMove))
	{
		debugPrintf("Trilaterated (%.2f, %.2f, %.2f)" DEGREE_SYMBOL " to X=%.2f Y=%.2f Z=%.2f\n",
			(double)Ha, (double)Hb, (double)Hc,
			(double)machinePos[X_AXIS], (double)machinePos[Y_AXIS], (double)machinePos[Z_AXIS]);
	}
}

// End
