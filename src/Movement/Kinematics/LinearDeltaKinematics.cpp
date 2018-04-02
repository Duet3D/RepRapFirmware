/*
 * DeltaParameters.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include "LinearDeltaKinematics.h"

#include "Movement/Move.h"
#include "RepRap.h"
#include "Storage/FileStore.h"
#include "GCodes/GCodeBuffer.h"


LinearDeltaKinematics::LinearDeltaKinematics() : Kinematics(KinematicsType::linearDelta, -1.0, 0.0, true)
{
	Init();
}

// Return the name of the current kinematics
const char *LinearDeltaKinematics::GetName(bool forStatusReport) const
{
	return (forStatusReport) ? "delta" : "Linear delta";
}

void LinearDeltaKinematics::Init()
{
	diagonal = defaultDiagonal;
	radius = defaultDeltaRadius;
	xTilt = yTilt = 0.0;
	printRadius = defaultPrintRadius;
	homedHeight = defaultDeltaHomedHeight;
    doneAutoCalibration = false;

	for (size_t axis = 0; axis < DELTA_AXES; ++axis)
	{
		angleCorrections[axis] = 0.0;
		endstopAdjustments[axis] = 0.0;
		towerX[axis] = towerY[axis] = 0.0;
	}

	Recalc();
}

void LinearDeltaKinematics::Recalc()
{
	towerX[DELTA_A_AXIS] = -(radius * cosf((30 + angleCorrections[DELTA_A_AXIS]) * DegreesToRadians));
	towerY[DELTA_A_AXIS] = -(radius * sinf((30 + angleCorrections[DELTA_A_AXIS]) * DegreesToRadians));
	towerX[DELTA_B_AXIS] = +(radius * cosf((30 - angleCorrections[DELTA_B_AXIS]) * DegreesToRadians));
	towerY[DELTA_B_AXIS] = -(radius * sinf((30 - angleCorrections[DELTA_B_AXIS]) * DegreesToRadians));
	towerX[DELTA_C_AXIS] = -(radius * sinf(angleCorrections[DELTA_C_AXIS] * DegreesToRadians));
	towerY[DELTA_C_AXIS] = +(radius * cosf(angleCorrections[DELTA_C_AXIS] * DegreesToRadians));

	Xbc = towerX[DELTA_C_AXIS] - towerX[DELTA_B_AXIS];
	Xca = towerX[DELTA_A_AXIS] - towerX[DELTA_C_AXIS];
	Xab = towerX[DELTA_B_AXIS] - towerX[DELTA_A_AXIS];
	Ybc = towerY[DELTA_C_AXIS] - towerY[DELTA_B_AXIS];
	Yca = towerY[DELTA_A_AXIS] - towerY[DELTA_C_AXIS];
	Yab = towerY[DELTA_B_AXIS] - towerY[DELTA_A_AXIS];
	coreFa = fsquare(towerX[DELTA_A_AXIS]) + fsquare(towerY[DELTA_A_AXIS]);
	coreFb = fsquare(towerX[DELTA_B_AXIS]) + fsquare(towerY[DELTA_B_AXIS]);
	coreFc = fsquare(towerX[DELTA_C_AXIS]) + fsquare(towerY[DELTA_C_AXIS]);
	Q = (Xca * Yab - Xab * Yca) * 2;
	Q2 = fsquare(Q);
	D2 = fsquare(diagonal);

	// Calculate the base carriage height when the printer is homed, i.e. the carriages are at the endstops less the corrections
	const float tempHeight = diagonal;		// any sensible height will do here
	float machinePos[DELTA_AXES];
	InverseTransform(tempHeight, tempHeight, tempHeight, machinePos);
	homedCarriageHeight = homedHeight + tempHeight - machinePos[Z_AXIS];
	printRadiusSquared = fsquare(printRadius);
}

// Make the average of the endstop adjustments zero, without changing the individual homed carriage heights
void LinearDeltaKinematics::NormaliseEndstopAdjustments()
{
	const float eav = (endstopAdjustments[DELTA_A_AXIS] + endstopAdjustments[DELTA_B_AXIS] + endstopAdjustments[DELTA_C_AXIS])/3.0;
	endstopAdjustments[DELTA_A_AXIS] -= eav;
	endstopAdjustments[DELTA_B_AXIS] -= eav;
	endstopAdjustments[DELTA_C_AXIS] -= eav;
	homedHeight += eav;
	homedCarriageHeight += eav;				// no need for a full recalc, this is sufficient
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
float LinearDeltaKinematics::Transform(const float machinePos[], size_t axis) const
{
	//TODO find a way of returning error if we can't transform the position
	if (axis < DELTA_AXES)
	{
		return sqrtf(D2 - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]))
			 + machinePos[Z_AXIS]
			 + (machinePos[X_AXIS] * xTilt)
			 + (machinePos[Y_AXIS] * yTilt);
	}
	else
	{
		return machinePos[axis];
	}
}

// Calculate the Cartesian coordinates from the motor coordinates
void LinearDeltaKinematics::InverseTransform(float Ha, float Hb, float Hc, float machinePos[DELTA_AXES]) const
{
	const float Fa = coreFa + fsquare(Ha);
	const float Fb = coreFb + fsquare(Hb);
	const float Fc = coreFc + fsquare(Hc);

	// Calculate PQRSU such that x = -(S - Uz)/Q, y = (P - Rz)/Q
	const float P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
	const float S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);
	const float R = ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc)) * 2;
	const float U = ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc)) * 2;

	const float R2 = fsquare(R), U2 = fsquare(U);

	const float A = U2 + R2 + Q2;
	const float minusHalfB = S * U + P * R + Ha * Q2 + towerX[DELTA_A_AXIS] * U * Q - towerY[DELTA_A_AXIS] * R * Q;
	const float C = fsquare(S + towerX[DELTA_A_AXIS] * Q) + fsquare(P - towerY[DELTA_A_AXIS] * Q) + (fsquare(Ha) - D2) * Q2;

	const float z = (minusHalfB - sqrtf(fsquare(minusHalfB) - A * C)) / A;
	machinePos[X_AXIS] = (U * z - S) / Q;
	machinePos[Y_AXIS] = (P - R * z) / Q;
	machinePos[Z_AXIS] = z - ((machinePos[X_AXIS] * xTilt) + (machinePos[Y_AXIS] * yTilt));
}

// Convert Cartesian coordinates to motor steps
bool LinearDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	//TODO return false if we can't transform the position
	for (size_t axis = 0; axis < min<size_t>(numVisibleAxes, DELTA_AXES); ++axis)
	{
		motorPos[axis] = lrintf(Transform(machinePos, axis) * stepsPerMm[axis]);
	}

	// Transform any additional axes linearly
	for (size_t axis = DELTA_AXES; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return true;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void LinearDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	InverseTransform(motorPos[DELTA_A_AXIS]/stepsPerMm[DELTA_A_AXIS], motorPos[DELTA_B_AXIS]/stepsPerMm[DELTA_B_AXIS], motorPos[DELTA_C_AXIS]/stepsPerMm[DELTA_C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = DELTA_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
bool LinearDeltaKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
bool LinearDeltaKinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated) const
{
	const AxesBitmap allAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS);
	bool limited = false;
	if ((axesHomed & allAxes) == allAxes)
	{
		// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
		// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(coords[X_AXIS]) + fsquare(coords[Y_AXIS]);
		if (diagonalSquared > printRadiusSquared)
		{
			const float factor = sqrtf(printRadiusSquared / diagonalSquared);
			coords[X_AXIS] *= factor;
			coords[Y_AXIS] *= factor;
			limited = true;
		}

		if (coords[Z_AXIS] < reprap.GetPlatform().AxisMinimum(Z_AXIS))
		{
			coords[Z_AXIS] = reprap.GetPlatform().AxisMinimum(Z_AXIS);
			limited = true;
		}
		else
		{
			// Determine the maximum reachable height at this radius, in the worst case when the head is on a radius to a tower
			const float maxHeight = homedCarriageHeight - sqrtf(D2 - fsquare(radius - sqrtf(diagonalSquared)));
			if (coords[Z_AXIS] > maxHeight)
			{
				coords[Z_AXIS] = maxHeight;
				limited = true;
			}
		}
	}

	// Limit any additional axes according to the M208 limits
	if (LimitPositionFromAxis(coords, Z_AXIS + 1, numVisibleAxes, axesHomed))
	{
		limited = true;
	}

	return limited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void LinearDeltaKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
	positions[Z_AXIS] = homedHeight;
}

// Auto calibrate from a set of probe points returning true if it failed
bool LinearDeltaKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply)
{
	const size_t NumDeltaFactors = 9;		// maximum number of delta machine factors we can adjust
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (numFactors < 3 || numFactors > NumDeltaFactors || numFactors == 5)
	{
		reply.printf("Delta calibration with %d factors requested but only 3, 4, 6, 7, 8 and 9 supported", numFactors);
		return true;
	}

	if (reprap.Debug(moduleMove))
	{
		String<ScratchStringLength> scratchString;
		PrintParameters(scratchString.GetRef());
		debugPrintf("%s\n", scratchString.c_str());
	}

	// The following is for printing out the calculation time, see later
	//uint32_t startTime = reprap.GetPlatform()->GetInterruptClocks();

	// Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
	FixedMatrix<floatc_t, MaxCalibrationPoints, DELTA_AXES> probeMotorPositions;
	floatc_t corrections[MaxCalibrationPoints];
	floatc_t initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		corrections[i] = 0.0;
		float machinePos[DELTA_AXES];
		const floatc_t zp = reprap.GetMove().GetProbeCoordinates(i, machinePos[X_AXIS], machinePos[Y_AXIS], probePoints.PointWasCorrected(i));
		machinePos[Z_AXIS] = 0.0;

		probeMotorPositions(i, DELTA_A_AXIS) = Transform(machinePos, DELTA_A_AXIS);
		probeMotorPositions(i, DELTA_B_AXIS) = Transform(machinePos, DELTA_B_AXIS);
		probeMotorPositions(i, DELTA_C_AXIS) = Transform(machinePos, DELTA_C_AXIS);

		initialSumOfSquares += fcsquare(zp);
	}

	// Do 1 or more Newton-Raphson iterations
	unsigned int iteration = 0;
	float expectedRmsError;
	for (;;)
	{
		// Build a Nx9 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<floatc_t, MaxCalibrationPoints, NumDeltaFactors> derivativeMatrix;
		for (size_t i = 0; i < numPoints; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				const size_t adjustedJ = (numFactors == 8 && j >= 6) ? j + 1 : j;		// skip diagonal rod length if doing 8-factor calibration
				derivativeMatrix(i, j) =
					ComputeDerivative(adjustedJ, probeMotorPositions(i, DELTA_A_AXIS), probeMotorPositions(i, DELTA_B_AXIS), probeMotorPositions(i, DELTA_C_AXIS));
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

		floatc_t solution[NumDeltaFactors];
		normalMatrix.GaussJordan(solution, numFactors);

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

		// Save the old homed carriage heights before we change the endstop corrections
		float homedCarriageHeights[DELTA_AXES];
		for (size_t drive = 0; drive < DELTA_AXES; ++drive)
		{
			homedCarriageHeights[drive] = GetHomedCarriageHeight(drive);
		}

		Adjust(numFactors, solution);	// adjust the delta parameters

		float heightAdjust[DELTA_AXES];
		for (size_t drive = 0; drive < DELTA_AXES; ++drive)
		{
			heightAdjust[drive] =  GetHomedCarriageHeight(drive) - homedCarriageHeights[drive];
		}

		// Adjust the motor endpoints to allow for the change to endstop adjustments
		reprap.GetMove().AdjustMotorPositions(heightAdjust, DELTA_AXES);

		// Calculate the expected probe heights using the new parameters
		{
			floatc_t expectedResiduals[MaxCalibrationPoints];
			floatc_t sumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < DELTA_AXES; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[DELTA_AXES];
				InverseTransform(probeMotorPositions(i, DELTA_A_AXIS), probeMotorPositions(i, DELTA_B_AXIS), probeMotorPositions(i, DELTA_C_AXIS), newPosition);
				corrections[i] = newPosition[Z_AXIS];
				expectedResiduals[i] = probePoints.GetZHeight(i) + newPosition[Z_AXIS];
				sumOfSquares += fcsquare(expectedResiduals[i]);
			}

			expectedRmsError = sqrtf((float)(sumOfSquares/numPoints));

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
		String<ScratchStringLength> scratchString;
		PrintParameters(scratchString.GetRef());
		debugPrintf("%s\n", scratchString.c_str());
	}

	reply.printf("Calibrated %d factors using %d points, deviation before %.3f after %.3f",
			numFactors, numPoints, (double)sqrtf(initialSumOfSquares/numPoints), (double)expectedRmsError);
	reprap.GetPlatform().MessageF(LogMessage, "%s\n", reply.c_str());

    doneAutoCalibration = true;
    return false;
}

// Return the type of motion computation needed by an axis
MotionType LinearDeltaKinematics::GetMotionType(size_t axis) const
{
	return (axis < DELTA_AXES) ? MotionType::segmentFreeDelta : MotionType::linear;
}

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = X, Y, Z tower endstop adjustments
// 3 = delta radius
// 4 = X tower correction
// 5 = Y tower correction
// 6 = diagonal rod length
// 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
floatc_t LinearDeltaKinematics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc) const
{
	const float perturb = 0.2;			// perturbation amount in mm or degrees
	LinearDeltaKinematics hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		// Endstop corrections
		break;

	case 3:
		hiParams.radius += perturb;
		loParams.radius -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 4:
		hiParams.angleCorrections[DELTA_A_AXIS] += perturb;
		loParams.angleCorrections[DELTA_A_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 5:
		hiParams.angleCorrections[DELTA_B_AXIS] += perturb;
		loParams.angleCorrections[DELTA_B_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 6:
		hiParams.diagonal += perturb;
		loParams.diagonal -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 7:
	case 8:
		// X and Y tilt
		break;
	}

	float newPos[DELTA_AXES];
	hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
	if (deriv == 7)
	{
		return -newPos[X_AXIS]/printRadius;
	}
	if (deriv == 8)
	{
		return -newPos[Y_AXIS]/printRadius;
	}

	const float zHi = newPos[Z_AXIS];
	loParams.InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
	const float zLo = newPos[Z_AXIS];

	return ((floatc_t)zHi - (floatc_t)zLo)/(floatc_t)(2 * perturb);
}

// Perform 3, 4, 6, 7, 8 or 9-factor adjustment.
// The input vector contains the following parameters in this order:
//  X, Y and Z endstop adjustments
//  Delta radius
//  X tower position adjustment
//  Y tower position adjustment
//  Diagonal rod length adjustment - omitted if doing 8-factor calibration (remainder are moved down)
//  X tilt adjustment
//  Y tilt adjustment
void LinearDeltaKinematics::Adjust(size_t numFactors, const floatc_t v[])
{
	const float oldCarriageHeightA = GetHomedCarriageHeight(DELTA_A_AXIS);	// save for later

	// Update endstop adjustments
	endstopAdjustments[DELTA_A_AXIS] += (float)v[0];
	endstopAdjustments[DELTA_B_AXIS] += (float)v[1];
	endstopAdjustments[DELTA_C_AXIS] += (float)v[2];
	NormaliseEndstopAdjustments();

	if (numFactors >= 4)
	{
		radius += (float)v[3];

		if (numFactors >= 6)
		{
			angleCorrections[DELTA_A_AXIS] += (float)v[4];
			angleCorrections[DELTA_B_AXIS] += (float)v[5];

			if (numFactors == 7 || numFactors == 9)
			{
				diagonal += (float)v[6];
			}

			if (numFactors == 8)
			{
				xTilt += (float)v[6]/printRadius;
				yTilt += (float)v[7]/printRadius;
			}
			else if (numFactors == 9)
			{
				xTilt += (float)v[7]/printRadius;
				yTilt += (float)v[8]/printRadius;
			}
		}

		Recalc();
	}

	// Adjusting the diagonal and the tower positions affects the homed carriage height.
	// We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
	const float heightError = GetHomedCarriageHeight(DELTA_A_AXIS) - oldCarriageHeightA - (float)v[0];
	homedHeight -= heightError;
	homedCarriageHeight -= heightError;

	// Note: if we adjusted the X and Y tilts, and there are any endstop adjustments, then the homed position won't be exactly in the centre
	// and changing the tilt will therefore affect the homed height. We ignore this for now. If it is ever significant, a second autocalibration
	// run will correct it.
}

// Print all the parameters for debugging
void LinearDeltaKinematics::PrintParameters(const StringRef& reply) const
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f height %.3f diagonal %.3f radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f xtilt %.3f%% ytilt %.3f%%\n",
		(double)endstopAdjustments[DELTA_A_AXIS], (double)endstopAdjustments[DELTA_B_AXIS], (double)endstopAdjustments[DELTA_C_AXIS], (double)homedHeight, (double)diagonal, (double)radius,
		(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS], (double)(xTilt * 100.0), (double)(yTilt * 100.0));
}

// Write the parameters that are set by auto calibration to a file, returning true if success
bool LinearDeltaKinematics::WriteCalibrationParameters(FileStore *f) const
{
	bool ok = f->Write("; Delta parameters\n");
	if (ok)
	{
		String<ScratchStringLength> scratchString;
		scratchString.printf("M665 L%.3f R%.3f H%.3f B%.1f X%.3f Y%.3f Z%.3f\n",
			(double)diagonal, (double)radius, (double)homedHeight, (double)printRadius, (double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
		ok = f->Write(scratchString.c_str());
		if (ok)
		{
			scratchString.printf("M666 X%.3f Y%.3f Z%.3f A%.2f B%.2f\n",
				(double)endstopAdjustments[X_AXIS], (double)endstopAdjustments[Y_AXIS], (double)endstopAdjustments[Z_AXIS], (double)(xTilt * 100.0), (double)(yTilt * 100.0));
			ok = f->Write(scratchString.c_str());
		}
	}
	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool LinearDeltaKinematics::WriteResumeSettings(FileStore *f) const
{
	return !doneAutoCalibration || WriteCalibrationParameters(f);
}

// Get the bed tilt fraction for the specified axis
float LinearDeltaKinematics::GetTiltCorrection(size_t axis) const
{
	return (axis == X_AXIS) ? xTilt : (axis == Y_AXIS) ? yTilt : 0.0;
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool LinearDeltaKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
{
	switch(mCode)
	{
	case 665:
		{
			bool seen = false;
			if (gb.Seen('L'))
			{
				diagonal = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('R'))
			{
				radius = gb.GetFValue();
				seen = true;
			}
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
			if (gb.Seen('X'))
			{
				// X tower position correction
				angleCorrections[DELTA_A_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('Y'))
			{
				// Y tower position correction
				angleCorrections[DELTA_B_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				// Z tower position correction
				angleCorrections[DELTA_C_AXIS] = gb.GetFValue();
				seen = true;
			}

			if (gb.Seen('H'))
			{
				homedHeight = gb.GetFValue();
				// Set the Z axis maximum so that DWC reports it correctly (it is not otherwise used for deltas)
				reprap.GetPlatform().SetAxisMaximum(Z_AXIS, homedHeight, false);
				seen = true;
			}

			if (seen)
			{
				Recalc();
			}
			else
			{
				reply.printf("Diagonal %.3f, delta radius %.3f, homed height %.3f, bed radius %.1f"
							 ", X %.3f" DEGREE_SYMBOL ", Y %.3f" DEGREE_SYMBOL ", Z %.3f" DEGREE_SYMBOL,
							 (double)diagonal, (double)radius,
							 (double)homedHeight, (double)printRadius,
							 (double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
			}
			return seen;
		}

	case 666:
		{
			bool seen = false;
			if (gb.Seen('X'))
			{
				endstopAdjustments[X_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('Y'))
			{
				endstopAdjustments[Y_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				endstopAdjustments[Z_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('A'))
			{
				xTilt = gb.GetFValue() * 0.01;
				seen = true;
			}
			if (gb.Seen('B'))
			{
				yTilt = gb.GetFValue() * 0.01;
				seen = true;
			}

			if (!seen)
			{
				reply.printf("Endstop adjustments X%.2f Y%.2f Z%.2f, tilt X%.2f%% Y%.2f%%",
					(double)endstopAdjustments[X_AXIS], (double)endstopAdjustments[Y_AXIS], (double)endstopAdjustments[Z_AXIS],
					(double)(xTilt * 100.0), (double)(yTilt * 100.0));
			}
			return seen;
		}

	default:
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap LinearDeltaKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const
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
AxesBitmap LinearDeltaKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const
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
const char* LinearDeltaKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, AxesBitmap& mustHomeFirst) const
{
	// If homing X, Y or Z we must home all the towers
	if ((toBeHomed & LowestNBits<AxesBitmap>(DELTA_AXES)) != 0)
	{
		return "homedelta.g";
	}

	// Return the homing file for the lowest axis that we have been asked to home
	for (size_t axis = DELTA_AXES; axis < numVisibleAxes; ++axis)
	{
		if (IsBitSet(toBeHomed, axis))
		{
			return StandardHomingFileNames[axis];
		}
	}

	mustHomeFirst = 0;
	return nullptr;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool LinearDeltaKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void LinearDeltaKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	if (highEnd)
	{
		const float hitPoint = GetHomedCarriageHeight(axis);
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
	}
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void LinearDeltaKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector) const
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

// End
