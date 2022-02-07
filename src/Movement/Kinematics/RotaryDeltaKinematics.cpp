/*
 * RotaryDeltaKinematics.cpp
 *
 *  Created on: 1 Aug 2018
 *      Author: David
 */

#include "RotaryDeltaKinematics.h"

#if SUPPORT_ROTARY_DELTA

#include <Movement/Move.h>
#include <Platform/RepRap.h>
#include <Storage/FileStore.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Math/Deviation.h>

const float RotaryDeltaKinematics::NormalTowerAngles[DELTA_AXES] = { -150.0, -30.0, 90.0 };

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RotaryDeltaKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry RotaryDeltaKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t RotaryDeltaKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(RotaryDeltaKinematics)

#endif

// Constructor
RotaryDeltaKinematics::RotaryDeltaKinematics() noexcept : RoundBedKinematics(KinematicsType::rotaryDelta, SegmentationType(true, true, true))
{
	Init();
}

// Initialise parameters to defaults
void RotaryDeltaKinematics::Init() noexcept
{
	radius = DefaultDeltaRadius;
	printRadius = DefaultPrintRadius;
	minArmAngle = DefaultMinArmAngle;
	maxArmAngle = DefaultMaxArmAngle;
    doneAutoCalibration = false;

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
void RotaryDeltaKinematics::Recalc() noexcept
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
const char *RotaryDeltaKinematics::GetName(bool forStatusReport) const noexcept
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
bool RotaryDeltaKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
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

			const bool seenNonGeometry = TryConfigureSegmentation(gb);

			if (seen)
			{
				Recalc();
			}
			else if (!seenNonGeometry && !gb.Seen('K'))
			{
				Kinematics::Configure(mCode, gb, reply, error);
				reply.catf(", arms (%.3f,%.2f,%.3f)mm, rods (%.3f,%.3f,%.3f)mm, bearingHeights (%.3f,%.2f,%.3f)mm"
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
bool RotaryDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
													size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept
{
	bool ok = true;
	for (size_t axis = 0; axis < min<size_t>(numVisibleAxes, DELTA_AXES); ++axis)
	{
		const float pos = Transform(machinePos, axis);
		if (std::isnan(pos) || std::isinf(pos))
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
void RotaryDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	ForwardTransform(motorPos[DELTA_A_AXIS]/stepsPerMm[DELTA_A_AXIS], motorPos[DELTA_B_AXIS]/stepsPerMm[DELTA_B_AXIS], motorPos[DELTA_C_AXIS]/stepsPerMm[DELTA_C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = DELTA_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Compute the derivative of height with respect to a parameter at a set of motor endpoints
// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = X, Y, Z tower homing switch adjustments
// 3 = bed height
// 4 = delta radius
// 5, 6 = X, Y tower corrections
floatc_t RotaryDeltaKinematics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const noexcept
{
	const float perturb = 0.2;			// perturbation amount in mm or degrees
	RotaryDeltaKinematics hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		// Endstop corrections
		break;

	case 3:
		for (unsigned int i = 0; i < DELTA_AXES; ++i)
		{
			hiParams.bearingHeights[i] += perturb;
			loParams.bearingHeights[i] -= perturb;
		}
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 4:
		hiParams.radius += perturb;
		loParams.radius -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 5:
		hiParams.angleCorrections[DELTA_A_AXIS] += perturb;
		loParams.angleCorrections[DELTA_A_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 6:
		hiParams.angleCorrections[DELTA_B_AXIS] += perturb;
		loParams.angleCorrections[DELTA_B_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;
	}

	float newPos[XYZ_AXES];
	hiParams.ForwardTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
	const float zHi = newPos[Z_AXIS];

	loParams.ForwardTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
	const float zLo = newPos[Z_AXIS];

	return ((floatc_t)zHi - (floatc_t)zLo)/(floatc_t)(2 * perturb);
}

// Perform auto calibration. Caller already owns the movement lock.
// Return true if an error occurred.
bool RotaryDeltaKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept
{
	constexpr size_t NumDeltaFactors = 7;		// maximum number of rotary delta machine factors we can adjust

	if (numFactors < 3 || numFactors > NumDeltaFactors || numFactors == 6)
	{
		reply.printf("Rotary delta calibration with %d factors requested but only 3, 4, 5 and 7 supported", numFactors);
		return true;
	}

	if (reprap.Debug(moduleMove))
	{
		String<StringLength256> scratchString;
		PrintParameters(scratchString.GetRef());
		debugPrintf("%s\n", scratchString.c_str());
	}

	// Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
	FixedMatrix<floatc_t, MaxCalibrationPoints, DELTA_AXES> probeMotorPositions;
	floatc_t corrections[MaxCalibrationPoints];
	Deviation initialDeviation;
	const size_t numPoints = probePoints.NumberOfProbePoints();

	{
		floatc_t initialSum = 0.0, initialSumOfSquares = 0.0;
		for (size_t i = 0; i < numPoints; ++i)
		{
			corrections[i] = 0.0;
			float machinePos[XYZ_AXES];
			const floatc_t zp = reprap.GetMove().GetProbeCoordinates(i, machinePos[X_AXIS], machinePos[Y_AXIS], probePoints.PointWasCorrected(i));
			machinePos[Z_AXIS] = 0.0;

			probeMotorPositions(i, DELTA_A_AXIS) = Transform(machinePos, DELTA_A_AXIS);
			probeMotorPositions(i, DELTA_B_AXIS) = Transform(machinePos, DELTA_B_AXIS);
			probeMotorPositions(i, DELTA_C_AXIS) = Transform(machinePos, DELTA_C_AXIS);

			initialSum += zp;
			initialSumOfSquares += fcsquare(zp);
		}
		initialDeviation.Set(initialSumOfSquares, initialSum, numPoints);
	}

	// Do 1 or more Newton-Raphson iterations
	Deviation finalDeviation;
	unsigned int iteration = 0;
	for (;;)
	{
		// Build a Nx9 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<floatc_t, MaxCalibrationPoints, NumDeltaFactors> derivativeMatrix;
		for (size_t i = 0; i < numPoints; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				const size_t adjustedJ = (numFactors == 8 && j >= 6) ? j + 1 : j;		// skip diagonal rod length if doing 8-factor calibration
				const floatc_t d =
					ComputeDerivative(adjustedJ, probeMotorPositions(i, DELTA_A_AXIS), probeMotorPositions(i, DELTA_B_AXIS), probeMotorPositions(i, DELTA_C_AXIS));
				if (std::isnan(d))			// a couple of users have reported getting Nans in the derivative, probably due to points being unreachable
				{
					reply.printf("Auto calibration failed because probe point P%u was unreachable using the current delta parameters. Try a smaller probing radius.", i);
					return true;
				}
				derivativeMatrix(i, j) = d;
			}
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Derivative matrix", derivativeMatrix, numPoints, numFactors);
		}

		// Now build the normal equations for least squares fitting
		FixedMatrix<floatc_t, NumDeltaFactors, NumDeltaFactors + 1> normalMatrix;
		for (size_t i = 0; i < numFactors; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				floatc_t temp = derivativeMatrix(0, i) * derivativeMatrix(0, j);
				for (size_t k = 1; k < numPoints; ++k)
				{
					temp += derivativeMatrix(k, i) * derivativeMatrix(k, j);
				}
				normalMatrix(i, j) = temp;
			}
			floatc_t temp = derivativeMatrix(0, i) * -((floatc_t)probePoints.GetZHeight(0) + corrections[0]);
			for (size_t k = 1; k < numPoints; ++k)
			{
				temp += derivativeMatrix(k, i) * -((floatc_t)probePoints.GetZHeight(k) + corrections[k]);
			}
			normalMatrix(i, numFactors) = temp;
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Normal matrix", normalMatrix, numFactors, numFactors + 1);
		}

		if (!normalMatrix.GaussJordan(numFactors, numFactors + 1))
		{
			reply.copy("Unable to calculate calibration parameters. Please choose different probe points.");
			return true;
		}

		floatc_t solution[NumDeltaFactors];
		for (size_t i = 0; i < numFactors; ++i)
		{
			solution[i] = normalMatrix(i, numFactors);
		}

		if (reprap.Debug(moduleMove))
		{
			PrintMatrix("Solved matrix", normalMatrix, numFactors, numFactors + 1);
			PrintVector("Solution", solution, numFactors);

			// Calculate and display the residuals
			// Save a little stack by not allocating a residuals vector, because stack for it doesn't only get reserved when debug is enabled.
			debugPrintf("Residuals:");
			for (size_t i = 0; i < numPoints; ++i)
			{
				floatc_t residual = probePoints.GetZHeight(i);
				for (size_t j = 0; j < numFactors; ++j)
				{
					residual += solution[j] * derivativeMatrix(i, j);
				}
				debugPrintf(" %7.4f", (double)residual);
			}

			debugPrintf("\n");
		}

		{
			Adjust(numFactors, solution);	// adjust the delta parameters

			// Adjust the motor endpoints to allow for the change to endstop adjustments
			float heightAdjust[DELTA_AXES];
			for (size_t drive = 0; drive < DELTA_AXES; ++drive)
			{
				heightAdjust[drive] = solution[drive];
			}
			reprap.GetMove().AdjustMotorPositions(heightAdjust, DELTA_AXES);
		}

		// Calculate the expected probe heights using the new parameters
		{
			floatc_t expectedResiduals[MaxCalibrationPoints];
			floatc_t finalSum = 0.0, finalSumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < DELTA_AXES; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[XYZ_AXES];
				ForwardTransform(probeMotorPositions(i, DELTA_A_AXIS), probeMotorPositions(i, DELTA_B_AXIS), probeMotorPositions(i, DELTA_C_AXIS), newPosition);
				corrections[i] = newPosition[Z_AXIS];
				expectedResiduals[i] = probePoints.GetZHeight(i) + newPosition[Z_AXIS];
				finalSum += expectedResiduals[i];
				finalSumOfSquares += fcsquare(expectedResiduals[i]);
			}

			finalDeviation.Set(finalSumOfSquares, finalSum, numPoints);

			if (reprap.Debug(moduleMove))
			{
				PrintVector("Expected probe error", expectedResiduals, numPoints);
			}
		}

		// Decide whether to do another iteration. Two is slightly better than one, but three doesn't improve things.
		// Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
		++iteration;
		if (iteration == 2)
		{
			break;
		}
	}

	// Print out the calculation time
	//debugPrintf("Time taken %dms\n", (reprap.GetPlatform()->GetInterruptClocks() - startTime) * 1000 / DDA::stepClockRate);
	if (reprap.Debug(moduleMove))
	{
		String<StringLength256> scratchString;
		PrintParameters(scratchString.GetRef());
		debugPrintf("%s\n", scratchString.c_str());
	}

	reprap.GetMove().SetInitialCalibrationDeviation(initialDeviation);
	reprap.GetMove().SetLatestCalibrationDeviation(finalDeviation, numFactors);

	reply.printf("Calibrated %d factors using %d points, (mean, deviation) before (%.3f, %.3f) after (%.3f, %.3f)",
			numFactors, numPoints,
			(double)initialDeviation.GetMean(), (double)initialDeviation.GetDeviationFromMean(),
			(double)finalDeviation.GetMean(), (double)finalDeviation.GetDeviationFromMean());

	// We don't want to call MessageF(LogMessage, "%s\n", reply.c_str()) here because that will allocate a buffer within MessageF, which adds to our stack usage.
	// Better to allocate the buffer here so that it uses the same stack space as the arrays that we have finished with
	{
		String<StringLength256> scratchString;
		scratchString.printf("%s\n", reply.c_str());
		reprap.GetPlatform().Message(LogWarn, scratchString.c_str());
	}

    doneAutoCalibration = true;
    return false;
}

// Perform 3, 4, 5 or 7-factor adjustment.
// The input vector contains the following parameters in this order:
//  X, Y and Z endstop adjustments
//  Bearing heights adjustment
//  Delta radius
//  X tower position adjustment
//  Y tower position adjustment
void RotaryDeltaKinematics::Adjust(size_t numFactors, const floatc_t v[]) noexcept
{
	// Update endstop adjustments and bearing heights
	for (size_t tower = 0; tower < DELTA_AXES; ++tower)
	{
		endstopAdjustments[tower] += (float)v[tower];
		if (numFactors >= 4)
		{
			bearingHeights[tower] += (float)v[3];
		}
	}

	// Update the delta radius
	if (numFactors >= 5)
	{
		radius += (float)v[4];
	}

	// Update the tower position corrections
	if (numFactors == 7)
	{
		angleCorrections[DELTA_A_AXIS] += (float)v[5];
		angleCorrections[DELTA_B_AXIS] += (float)v[6];
	}

	Recalc();
}

// Print all the parameters for debugging
void RotaryDeltaKinematics::PrintParameters(const StringRef& reply) const noexcept
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f bearing heights", (double)endstopAdjustments[DELTA_A_AXIS], (double)endstopAdjustments[DELTA_B_AXIS], (double)endstopAdjustments[DELTA_C_AXIS]);
	for (size_t tower = 0; tower < DELTA_AXES; ++tower)
	{
		reply.catf("%c%.3f", (tower == 0) ? ' ' : ':', (double)bearingHeights[tower]);
	}
	reply.catf(" radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f\n",
		(double)radius,
		(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Add a space, character, and 3-element vector to the string
static void CatVector3(const StringRef& str, char c, const float vec[3]) noexcept
{
	str.catf(" %c%.2f:%.2f:%.2f", c, (double)vec[0], (double)vec[1], (double)vec[2]);
}

// Write the parameters that are set by auto calibration to a file, returning true if success
bool RotaryDeltaKinematics::WriteCalibrationParameters(FileStore *f) const noexcept
{
	bool ok = f->Write("; Rotary delta parameters\n");
	if (ok)
	{
		String<StringLength256> scratchString;
		scratchString.printf("M669 K10 R%.2f B%.1f A%.2f:%.2f X%.2f Y%.2f Z%.2f",
								(double)radius, (double)printRadius, (double)minArmAngle, (double)maxArmAngle,
								(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
		CatVector3(scratchString.GetRef(), 'U', armLengths);
		CatVector3(scratchString.GetRef(), 'L', rodLengths);
		CatVector3(scratchString.GetRef(), 'H', bearingHeights);
		scratchString.cat('\n');
		ok = f->Write(scratchString.c_str());

		if (ok)
		{
			scratchString.printf("M666 X%.3f Y%.3f Z%.3f\n", (double)endstopAdjustments[X_AXIS], (double)endstopAdjustments[Y_AXIS], (double)endstopAdjustments[Z_AXIS]);
			ok = f->Write(scratchString.c_str());
		}
	}
	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool RotaryDeltaKinematics::WriteResumeSettings(FileStore *f) const noexcept
{
	return !doneAutoCalibration || WriteCalibrationParameters(f);
}

#endif

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
LimitPositionResult RotaryDeltaKinematics::LimitPosition(float finalCoords[], const float * null initialCoords,
															size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept
{
	bool limited = false;
	if ((axesToLimit & XyzAxes) == XyzAxes)
	{
		// If axes have been homed on a rotary delta printer and this isn't a homing move, check for movements outside limits.
		// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
		if (diagonalSquared > printRadiusSquared)
		{
			const float factor = fastSqrtf(printRadiusSquared / diagonalSquared);
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
	if (applyM208Limits && LimitPositionFromAxis(finalCoords, Z_AXIS + 1, numVisibleAxes, axesToLimit))
	{
		limited = true;
	}

	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void RotaryDeltaKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept
{
	ForwardTransform(0.0, 0.0, 0.0, positions);			// assume that the arms are horizontal
	for (size_t i = DELTA_AXES; i < numAxes; ++i)
	{
		positions[i] = 0.0;								// set additional axes to zero
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap RotaryDeltaKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept
{
	// If all of X, Y and Z have been specified then we know the positions of all 3 tower motors, otherwise we don't
	if ((g92Axes & XyzAxes) != XyzAxes)
	{
		g92Axes &= ~XyzAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap RotaryDeltaKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept
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
AxesBitmap RotaryDeltaKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept
{
	// If homing X, Y or Z we must home all the towers
	if (toBeHomed.Intersects(AxesBitmap::MakeLowestNBits(DELTA_AXES)))
	{
		filename.copy("homedelta.g");
		return AxesBitmap();
	}

	return Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool RotaryDeltaKinematics::QueryTerminateHomingMove(size_t axis) const noexcept
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate().
void RotaryDeltaKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept
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
float RotaryDeltaKinematics::Transform(const float machinePos[], size_t axis) const noexcept
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
		const float sinTheta = (b * c - a * fastSqrtf(fsquare(a) + fsquare(b) - fsquare(c)))/(fsquare(a) + fsquare(b));

		// 4. Take the arc sine and convert to degrees
		return asinf(sinTheta) * RadiansToDegrees;
	}
	else
	{
		return machinePos[axis];		// any additional axes must be linear
	}
}

// Calculate the Cartesian coordinates from the motor coordinates. We do this by trilateration.
void RotaryDeltaKinematics::ForwardTransform(float Ha, float Hb, float Hc, float machinePos[DELTA_AXES]) const noexcept
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

	// Calculate PQRST such that x = (Qz + S)/P, y = -(Rz + T)/P.
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
	const float z = (- halfB - fastSqrtf(fsquare(halfB) - A * C))/A;

	// Substitute back for X and Y
	machinePos[X_AXIS] = (Q * z + S)/P;
	machinePos[Y_AXIS] = -(R * z + T)/P;
	machinePos[Z_AXIS] = z;

	if (reprap.Debug(moduleMove))
	{
		debugPrintf("Trilaterated (%.2f, %.2f, %.2f)" DEGREE_SYMBOL " to X=%.2f Y=%.2f Z=%.2f\n",
			(double)Ha, (double)Hb, (double)Hc,
			(double)machinePos[X_AXIS], (double)machinePos[Y_AXIS], (double)machinePos[Z_AXIS]);
	}
}

#endif // SUPPORT_ROTARY_DELTA

// End
