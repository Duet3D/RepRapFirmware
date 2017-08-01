/*
 * DeltaParameters.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include <Movement/Kinematics/LinearDeltaKinematics.h>
#include "Pins.h"
#include "Configuration.h"
#include "Movement/Move.h"
#include "RepRap.h"
#include "Storage/FileStore.h"

LinearDeltaKinematics::LinearDeltaKinematics() : Kinematics(KinematicsType::linearDelta)
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
	towerX[A_AXIS] = -(radius * cos((30 + angleCorrections[A_AXIS]) * DegreesToRadians));
	towerY[A_AXIS] = -(radius * sin((30 + angleCorrections[A_AXIS]) * DegreesToRadians));
	towerX[B_AXIS] = +(radius * cos((30 - angleCorrections[B_AXIS]) * DegreesToRadians));
	towerY[B_AXIS] = -(radius * sin((30 - angleCorrections[B_AXIS]) * DegreesToRadians));
	towerX[C_AXIS] = -(radius * sin(angleCorrections[C_AXIS] * DegreesToRadians));
	towerY[C_AXIS] = +(radius * cos(angleCorrections[C_AXIS] * DegreesToRadians));

	Xbc = towerX[C_AXIS] - towerX[B_AXIS];
	Xca = towerX[A_AXIS] - towerX[C_AXIS];
	Xab = towerX[B_AXIS] - towerX[A_AXIS];
	Ybc = towerY[C_AXIS] - towerY[B_AXIS];
	Yca = towerY[A_AXIS] - towerY[C_AXIS];
	Yab = towerY[B_AXIS] - towerY[A_AXIS];
	coreFa = fsquare(towerX[A_AXIS]) + fsquare(towerY[A_AXIS]);
	coreFb = fsquare(towerX[B_AXIS]) + fsquare(towerY[B_AXIS]);
	coreFc = fsquare(towerX[C_AXIS]) + fsquare(towerY[C_AXIS]);
	Q = 2 * (Xca * Yab - Xab * Yca);
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
	const float eav = (endstopAdjustments[A_AXIS] + endstopAdjustments[B_AXIS] + endstopAdjustments[C_AXIS])/3.0;
	endstopAdjustments[A_AXIS] -= eav;
	endstopAdjustments[B_AXIS] -= eav;
	endstopAdjustments[C_AXIS] -= eav;
	homedHeight += eav;
	homedCarriageHeight += eav;				// no need for a full recalc, this is sufficient
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
float LinearDeltaKinematics::Transform(const float machinePos[], size_t axis) const
{
	//TODO find a way of returning error if we can't transform the position
	if (axis < DELTA_AXES)
	{
		return sqrt(D2 - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]))
			 + machinePos[Z_AXIS]
			 + (machinePos[X_AXIS] * xTilt)
			 + (machinePos[Y_AXIS] * yTilt);
	}
	else
	{
		return machinePos[axis];
	}
}

// Calculate the Cartesian coordinates from the motor coordinates.
void LinearDeltaKinematics::InverseTransform(float Ha, float Hb, float Hc, float machinePos[DELTA_AXES]) const
{
	const float Fa = coreFa + fsquare(Ha);
	const float Fb = coreFb + fsquare(Hb);
	const float Fc = coreFc + fsquare(Hc);

//	debugPrintf("Ha=%f Hb=%f Hc=%f Fa=%f Fb=%f Fc=%f Xbc=%f Xca=%f Xab=%f Ybc=%f Yca=%f Yab=%f\n",
//				Ha, Hb, Hc, Fa, Fb, Fc, Xbc, Xca, Xab, Ybc, Yca, Yab);

	// Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
	const float P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
	const float S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);

	const float R = 2 * ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc));
	const float U = 2 * ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc));

//	debugPrintf("P= %f R=%f S=%f U=%f Q=%f\n", P, R, S, U, Q);

	const float R2 = fsquare(R), U2 = fsquare(U);

	float A = U2 + R2 + Q2;
	float minusHalfB = S * U + P * R + Ha * Q2 + towerX[A_AXIS] * U * Q - towerY[A_AXIS] * R * Q;
	float C = fsquare(S + towerX[A_AXIS] * Q) + fsquare(P - towerY[A_AXIS] * Q) + (fsquare(Ha) - D2) * Q2;

//	debugPrintf("A=%f minusHalfB=%f C=%f\n", A, minusHalfB, C);

	float z = (minusHalfB - sqrtf(fsquare(minusHalfB) - A * C)) / A;
	machinePos[X_AXIS] = (U * z - S) / Q;
	machinePos[Y_AXIS] = (P - R * z) / Q;
	machinePos[Z_AXIS] = z - ((machinePos[X_AXIS] * xTilt) + (machinePos[Y_AXIS] * yTilt));
}

// Convert Cartesian coordinates to motor steps
bool LinearDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[]) const
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
	InverseTransform(motorPos[A_AXIS]/stepsPerMm[A_AXIS], motorPos[B_AXIS]/stepsPerMm[B_AXIS], motorPos[C_AXIS]/stepsPerMm[C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = DELTA_AXES; drive < numVisibleAxes; ++drive)
	{
		machinePos[drive] = motorPos[drive]/stepsPerMm[drive];
	}
}

// Return true if the specified XY position is reachable by the print head reference point.
bool LinearDeltaKinematics::IsReachable(float x, float y) const
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}

// Limit the Cartesian position that the user wants to move to
bool LinearDeltaKinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed) const
{
	const AxesBitmap allAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS);
	bool limited = false;
	if ((axesHomed & allAxes) == allAxes)
	{
		// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
		// Skip this check if axes have not been homed, so that extruder-only moved are allowed before homing
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

// Auto calibrate from a set of probe points
void LinearDeltaKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, StringRef& reply)
{
	const size_t NumDeltaFactors = 9;		// maximum number of delta machine factors we can adjust
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (numFactors < 3 || numFactors > NumDeltaFactors || numFactors == 5)
	{
		reply.printf("Error: Delta calibration with %d factors requested but only 3, 4, 6, 7, 8 and 9 supported", numFactors);
		return;
	}

	if (reprap.Debug(moduleMove))
	{
		PrintParameters(scratchString);
		debugPrintf("%s\n", scratchString.Pointer());
	}

	// The following is for printing out the calculation time, see later
	//uint32_t startTime = reprap.GetPlatform()->GetInterruptClocks();

	// Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
	FixedMatrix<floatc_t, MaxDeltaCalibrationPoints, DELTA_AXES> probeMotorPositions;
	floatc_t corrections[MaxDeltaCalibrationPoints];
	floatc_t initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		corrections[i] = 0.0;
		float machinePos[DELTA_AXES];
		const floatc_t zp = reprap.GetMove().GetProbeCoordinates(i, machinePos[X_AXIS], machinePos[Y_AXIS], probePoints.PointWasCorrected(i));
		machinePos[Z_AXIS] = 0.0;

		probeMotorPositions(i, A_AXIS) = Transform(machinePos, A_AXIS);
		probeMotorPositions(i, B_AXIS) = Transform(machinePos, B_AXIS);
		probeMotorPositions(i, C_AXIS) = Transform(machinePos, C_AXIS);

		initialSumOfSquares += fcsquare(zp);
	}

	// Do 1 or more Newton-Raphson iterations
	unsigned int iteration = 0;
	float expectedRmsError;
	for (;;)
	{
		// Build a Nx9 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<floatc_t, MaxDeltaCalibrationPoints, NumDeltaFactors> derivativeMatrix;
		for (size_t i = 0; i < numPoints; ++i)
		{
			for (size_t j = 0; j < numFactors; ++j)
			{
				const size_t adjustedJ = (numFactors == 8 && j >= 6) ? j + 1 : j;		// skip diagonal rod length if doing 8-factor calibration
				derivativeMatrix(i, j) =
					ComputeDerivative(adjustedJ, probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS));
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
			floatc_t temp = derivativeMatrix(0, i) * -(probePoints.GetZHeight(0) + corrections[0]);
			for (size_t k = 1; k < numPoints; ++k)
			{
				temp += derivativeMatrix(k, i) * -(probePoints.GetZHeight(k) + corrections[k]);
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
				debugPrintf(" %7.4f", residual);
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
			floatc_t expectedResiduals[MaxDeltaCalibrationPoints];
			floatc_t sumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < DELTA_AXES; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[DELTA_AXES];
				InverseTransform(probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS), newPosition);
				corrections[i] = newPosition[Z_AXIS];
				expectedResiduals[i] = probePoints.GetZHeight(i) + newPosition[Z_AXIS];
				sumOfSquares += fcsquare(expectedResiduals[i]);
			}

			expectedRmsError = sqrt(sumOfSquares/numPoints);

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
		PrintParameters(scratchString);
		debugPrintf("%s\n", scratchString.Pointer());
	}

	reply.printf("Calibrated %d factors using %d points, deviation before %.3f after %.3f",
			numFactors, numPoints, sqrt(initialSumOfSquares/numPoints), expectedRmsError);

    doneAutoCalibration = true;
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
		hiParams.angleCorrections[A_AXIS] += perturb;
		loParams.angleCorrections[A_AXIS] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 5:
		hiParams.angleCorrections[B_AXIS] += perturb;
		loParams.angleCorrections[B_AXIS] -= perturb;
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

	return ((floatc_t)zHi - (floatc_t)zLo)/(2 * perturb);
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
	const float oldCarriageHeightA = GetHomedCarriageHeight(A_AXIS);	// save for later

	// Update endstop adjustments
	endstopAdjustments[A_AXIS] += v[0];
	endstopAdjustments[B_AXIS] += v[1];
	endstopAdjustments[C_AXIS] += v[2];
	NormaliseEndstopAdjustments();

	if (numFactors >= 4)
	{
		radius += v[3];

		if (numFactors >= 6)
		{
			angleCorrections[A_AXIS] += v[4];
			angleCorrections[B_AXIS] += v[5];

			if (numFactors == 7 || numFactors == 9)
			{
				diagonal += v[6];
			}

			if (numFactors == 8)
			{
				xTilt += v[6]/printRadius;
				yTilt += v[7]/printRadius;
			}
			else if (numFactors == 9)
			{
				xTilt += v[7]/printRadius;
				yTilt += v[8]/printRadius;
			}
		}

		Recalc();
	}

	// Adjusting the diagonal and the tower positions affects the homed carriage height.
	// We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
	const float heightError = GetHomedCarriageHeight(A_AXIS) - oldCarriageHeightA - v[0];
	homedHeight -= heightError;
	homedCarriageHeight -= heightError;

	// Note: if we adjusted the X and Y tilts, and there are any endstop adjustments, then the homed position won't be exactly in the centre
	// and changing the tilt will therefore affect the homed height. We ignore this for now. If it is ever significant, a second autocalibration
	// run will correct it.
}

void LinearDeltaKinematics::PrintParameters(StringRef& reply) const
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f height %.3f diagonal %.3f radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f xtilt %.3f%% ytilt %.3f%%\n",
					endstopAdjustments[A_AXIS], endstopAdjustments[B_AXIS], endstopAdjustments[C_AXIS], homedHeight, diagonal, radius,
					angleCorrections[A_AXIS], angleCorrections[B_AXIS], angleCorrections[C_AXIS], xTilt * 100.0, yTilt * 100.0);
}

// Write the parameters that are set by auto calibration to a file, returning true if success
bool LinearDeltaKinematics::WriteCalibrationParameters(FileStore *f) const
{
	bool ok = f->Write("; Delta parameters\n");
	if (ok)
	{
		scratchString.printf("M665 L%.3f R%.3f H%.3f B%.1f X%.3f Y%.3f Z%.3f\n",
					 diagonal, radius, homedHeight, printRadius, angleCorrections[A_AXIS], angleCorrections[B_AXIS], angleCorrections[C_AXIS]);
		ok = f->Write(scratchString.Pointer());
	}
	if (ok)
	{
		scratchString.printf("M666 X%.3f Y%.3f Z%.3f A%.2f B%.2f\n",
			endstopAdjustments[X_AXIS], endstopAdjustments[Y_AXIS], endstopAdjustments[Z_AXIS], xTilt * 100.0, yTilt * 100.0);
		ok = f->Write(scratchString.Pointer());
	}
	return ok;
}

#ifdef DUET_NG

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool LinearDeltaKinematics::WriteResumeSettings(FileStore *f) const
{
	return !doneAutoCalibration || WriteCalibrationParameters(f);
}

#endif

// Get the bed tilt fraction for the specified axis
float LinearDeltaKinematics::GetTiltCorrection(size_t axis) const
{
	return (axis == X_AXIS) ? xTilt : (axis == Y_AXIS) ? yTilt : 0.0;
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool LinearDeltaKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) /*override*/
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
				seen = true;
			}
			if (gb.Seen('X'))
			{
				// X tower position correction
				angleCorrections[A_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('Y'))
			{
				// Y tower position correction
				angleCorrections[B_AXIS] = gb.GetFValue();
				seen = true;
			}
			if (gb.Seen('Z'))
			{
				// Y tower position correction
				angleCorrections[C_AXIS] = gb.GetFValue();
				seen = true;
			}

			// The homed height must be done last, because it gets recalculated when some of the other factors are changed
			if (gb.Seen('H'))
			{
				homedHeight = gb.GetFValue();
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
								 diagonal, radius,
								 homedHeight, printRadius,
								 angleCorrections[A_AXIS], angleCorrections[B_AXIS], angleCorrections[C_AXIS]);
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
					endstopAdjustments[X_AXIS], endstopAdjustments[Y_AXIS], endstopAdjustments[Z_AXIS],
					xTilt * 100.0, yTilt * 100.0);
			}
			return seen;
		}

	default:
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
uint32_t LinearDeltaKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const
{
	// If all of X, Y and Z have been specified then we know the positions of all 3 tower motors, otherwise we don't
	const uint32_t xyzAxes = (1u << X_AXIS) | (1u << Y_AXIS) | (1u << Z_AXIS);
	if ((g92Axes & xyzAxes) != xyzAxes)
	{
		g92Axes &= ~xyzAxes;
	}
	return g92Axes;
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
const char* LinearDeltaKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap& alreadyHomed, size_t numVisibleAxes, AxesBitmap& mustHomeFirst) const
{
	alreadyHomed = 0;			// if we home one axis, we need to home them all
	return "homedelta.g";
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() or dda.SetPositions().
// Return true if the entire move should be stopped, false if only the motor concerned should be stopped.
bool LinearDeltaKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	if (highEnd)
	{
		const float hitPoint = GetHomedCarriageHeight(axis);
		dda.SetDriveCoordinate(hitPoint * stepsPerMm[axis], axis);
	}
	return false;
}

// End
