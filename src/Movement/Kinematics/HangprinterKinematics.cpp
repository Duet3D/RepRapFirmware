/*
 * HangprinterKinematics.cpp
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#include "HangprinterKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Movement/Move.h"
//#include "Movement/BedProbing/RandomProbePointSet.h"

// Default anchor coordinates
// These are only placeholders. Each machine must have these values calibrated in order to work correctly.
constexpr float DefaultAnchorA[3] = {    0.0, -2000.0, -100.0};
constexpr float DefaultAnchorB[3] = { 2000.0,  1000.0, -100.0};
constexpr float DefaultAnchorC[3] = {-2000.0,  1000.0, -100.0};
constexpr float DefaultAnchorDz = 3000.0;
constexpr float DefaultPrintRadius = 1500.0;

// Constructor
HangprinterKinematics::HangprinterKinematics()
	: Kinematics(KinematicsType::scara, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true)
{
	Init();
}

void HangprinterKinematics::Init()
{
	anchorDz = DefaultAnchorDz;
	printRadius = DefaultPrintRadius;
	ARRAY_INIT(anchorA, DefaultAnchorA);
	ARRAY_INIT(anchorB, DefaultAnchorB);
	ARRAY_INIT(anchorC, DefaultAnchorC);
	doneAutoCalibration = false;
	Recalc();
}

// Recalculate the derived parameters
void HangprinterKinematics::Recalc()
{
	printRadiusSquared = fsquare(printRadius);
	Da2 = fsquare(anchorA[0]) + fsquare(anchorA[1]) + fsquare(anchorA[2]);
	Db2 = fsquare(anchorB[0]) + fsquare(anchorB[1]) + fsquare(anchorB[2]);
	Dc2 = fsquare(anchorC[0]) + fsquare(anchorC[1]) + fsquare(anchorC[2]);
	Xab = anchorA[0] - anchorB[0];
	Xbc = anchorB[0] - anchorC[0];
	Xca = anchorC[0] - anchorA[0];
	Yab = anchorA[1] - anchorB[1];
	Ybc = anchorB[1] - anchorC[1];
	Yca = anchorC[1] - anchorA[1];
	Zab = anchorA[2] - anchorB[2];
	Zbc = anchorB[2] - anchorC[2];
	Zca = anchorC[2] - anchorA[2];
	P = (  anchorB[0] * Yca
		 - anchorA[0] * anchorC[1]
		 + anchorA[1] * anchorC[0]
		 - anchorB[1] * Xca
		) * 2;
	P2 = fsquare(P);
	Q = (  anchorB[1] * Zca
		 - anchorA[1] * anchorC[2]
		 + anchorA[2] * anchorC[1]
		 - anchorB[2] * Yca
		) * 2;
	R = - (  anchorB[0] * Zca
		   + anchorA[0] * anchorC[2]
		   + anchorA[2] * anchorC[0]
		   - anchorB[2] * Xca
		  ) * 2;
	U = (anchorA[2] * P2) + (anchorA[0] * Q * P) + (anchorA[1] * R * P);
	A = (P2 + fsquare(Q) + fsquare(R)) * 2;
}

// Return the name of the current kinematics
const char *HangprinterKinematics::GetName(bool forStatusReport) const
{
	return "Hangprinter";
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool HangprinterKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) /*override*/
{
	if (mCode == 669)
	{
		bool seen = false;
		bool seenNonGeometry = false;
		gb.TryGetFValue('S', segmentsPerSecond, seenNonGeometry);
		gb.TryGetFValue('T', minSegmentLength, seenNonGeometry);
		if (gb.TryGetFloatArray('A', 3, anchorA, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('B', 3, anchorB, reply, seen))
		{
			error = true;
			return true;
		}
		if (gb.TryGetFloatArray('C', 3, anchorC, reply, seen))
		{
			error = true;
			return true;
		}
		gb.TryGetFValue('D', anchorDz, seen);

		if (seen || seenNonGeometry)
		{
			Recalc();
		}
		if (gb.Seen('P'))
		{
			printRadius = gb.GetFValue();
			seen = true;
		}
		else if (!gb.Seen('K'))
		{
			reply.printf("Kinematics is Hangprinter with ABC anchor coordinates (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f),"
							"D anchor Z coordinate %.2f, print radius %.1f, segments/sec %d, min. segment length %.2f",
							(double)anchorA[X_AXIS], (double)anchorA[Y_AXIS], (double)anchorA[Z_AXIS],
							(double)anchorB[X_AXIS], (double)anchorB[Y_AXIS], (double)anchorB[Z_AXIS],
							(double)anchorC[X_AXIS], (double)anchorC[Y_AXIS], (double)anchorC[Z_AXIS],
							(double)anchorDz, (double)printRadius,
							(int)segmentsPerSecond, (double)minSegmentLength);
		}
		return seen;
	}
	else
	{
		return Kinematics::Configure(mCode, gb, reply, error);
	}
}

// Calculate the square of the line length from a spool from a Cartesian coordinate
inline float HangprinterKinematics::LineLengthSquared(const float machinePos[3], const float anchor[3]) const
{
	return fsquare(anchor[Z_AXIS] - machinePos[Z_AXIS]) + fsquare(anchor[Y_AXIS] - machinePos[Y_AXIS]) + fsquare(anchor[X_AXIS] - machinePos[X_AXIS]);
}

// Convert Cartesian coordinates to motor coordinates, returning true if successful
bool HangprinterKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	const float aSquared = LineLengthSquared(machinePos, anchorA);
	const float bSquared = LineLengthSquared(machinePos, anchorB);
	const float cSquared = LineLengthSquared(machinePos, anchorC);
	const float dSquared =    fsquare(machinePos[X_AXIS])
							+ fsquare(machinePos[Y_AXIS])
							+ fsquare(anchorDz - machinePos[Z_AXIS]);
	if (aSquared > 0.0 && bSquared > 0.0 && cSquared > 0.0 && dSquared > 0.0)
	{
		motorPos[A_AXIS] = lrintf(sqrtf(aSquared) * stepsPerMm[A_AXIS]);
		motorPos[B_AXIS] = lrintf(sqrtf(bSquared) * stepsPerMm[B_AXIS]);
		motorPos[C_AXIS] = lrintf(sqrtf(cSquared) * stepsPerMm[C_AXIS]);
		motorPos[D_AXIS] = lrintf(sqrtf(dSquared) * stepsPerMm[D_AXIS]);
		return true;
	}
	return false;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void HangprinterKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	InverseTransform(motorPos[A_AXIS]/stepsPerMm[A_AXIS], motorPos[B_AXIS]/stepsPerMm[B_AXIS], motorPos[C_AXIS]/stepsPerMm[C_AXIS], machinePos);
}

// Return true if the specified XY position is reachable by the print head reference point.
bool HangprinterKinematics::IsReachable(float x, float y, bool isCoordinated) const
{
	return fsquare(x) + fsquare(y) < printRadiusSquared;
}

// Limit the Cartesian position that the user wants to move to returning true if we adjusted the position
bool HangprinterKinematics::LimitPosition(float finalCoords[], float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const
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

		if (applyM208Limits)
		{
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
	}
	return limited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
void HangprinterKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
}

// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
// If we can proceed with homing some axes, return the name of the homing file to be called.
// If we can't proceed because other axes need to be homed first, return nullptr and pass those axes back in 'mustBeHomedFirst'.
AxesBitmap HangprinterKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const
{
	filename.copy("homeall.g");
	return 0;
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool HangprinterKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
void HangprinterKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	// Hangprinter homing is not supported
}

// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
AxesBitmap HangprinterKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const
{
	// If all of X, Y and Z have been specified then we know the positions of all 4 spool motors, otherwise we don't
	const uint32_t xyzAxes = (1u << X_AXIS) | (1u << Y_AXIS) | (1u << Z_AXIS);
	if ((g92Axes & xyzAxes) == xyzAxes)
	{
		g92Axes |= (1u << D_AXIS);
	}
	else
	{
		g92Axes &= ~xyzAxes;
	}
	return g92Axes;
}

// Return the set of axes that must be homed prior to regular movement of the specified axes
AxesBitmap HangprinterKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const
{
	constexpr AxesBitmap xyzAxes = MakeBitmap<AxesBitmap>(X_AXIS) |  MakeBitmap<AxesBitmap>(Y_AXIS) |  MakeBitmap<AxesBitmap>(Z_AXIS);
	if ((axesMoving & xyzAxes) != 0)
	{
		axesMoving |= xyzAxes;
	}
	return axesMoving;
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void HangprinterKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const
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
AxesBitmap HangprinterKinematics::GetLinearAxes() const
{
	return 0;
}

// Write the parameters that are set by auto calibration to a file, returning true if success
bool HangprinterKinematics::WriteCalibrationParameters(FileStore *f) const
{
	bool ok = f->Write("; Hangprinter parameters\n");
	if (ok)
	{
		String<100> scratchString;
		scratchString.printf("M669 K6 A%.3f:%.3f:%.3f B%.3f:%.3f:%.3f C%.3f:%.3f:%.3f D%.3f P%.1f\n",
							(double)anchorA[X_AXIS], (double)anchorA[Y_AXIS], (double)anchorA[Z_AXIS],
							(double)anchorB[X_AXIS], (double)anchorB[Y_AXIS], (double)anchorB[Z_AXIS],
							(double)anchorC[X_AXIS], (double)anchorC[Y_AXIS], (double)anchorC[Z_AXIS],
							(double)anchorDz, (double)printRadius);
		ok = f->Write(scratchString.c_str());
	}
	return ok;
}

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool HangprinterKinematics::WriteResumeSettings(FileStore *f) const
{
	return !doneAutoCalibration || WriteCalibrationParameters(f);
}

// Calculate the Cartesian coordinates from the motor coordinates
void HangprinterKinematics::InverseTransform(float La, float Lb, float Lc, float machinePos[3]) const
{
	// Calculate PQRST such that x = (Qz + S)/P, y = (Rz + T)/P
	const float S = - Yab * (fsquare(Lc) - Dc2)
					- Yca * (fsquare(Lb) - Db2)
					- Ybc * (fsquare(La) - Da2);
	const float T = - Xab * (fsquare(Lc) - Dc2)
					+ Xca * (fsquare(Lb) - Db2)
					+ Xbc * (fsquare(La) - Da2);

	// Calculate quadratic equation coefficients
	const float halfB = (S * Q) - (R * T) - U;
	const float C = fsquare(S) + fsquare(T) + (anchorA[1] * T - anchorA[0] * S) * P * 2 + (Da2 - fsquare(La)) * P2;

	// Solve the quadratic equation for z
	machinePos[2] = (- halfB - sqrtf(fsquare(halfB) - A * C))/A;

	// Substitute back for X and Y
	machinePos[0] = (Q * machinePos[2] + S)/P;
	machinePos[1] = (R * machinePos[2] + T)/P;

	debugPrintf("Motor %.2f,%.2f,%.2f to Cartesian %.2f,%.2f,%.2f\n", (double)La, (double)Lb, (double)Lc, (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
}

// Auto calibrate from a set of probe points returning true if it failed
// We can't calibrate all the XY parameters because translations and rotations of the anchors don't alter the height. Therefore we calibrate the following:
// 0, 1, 2 Spool zero length motor positions (this is like endstop calibration on a delta)
// 3       Y coordinate of the B anchor
// 4, 5    X and Y coordinates of the C anchor
// 6, 7, 8 Heights of the A, B, C anchors
// We don't touch the XY coordinates of the A anchor or the X coordinate of the B anchor.
bool HangprinterKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply)
{
	const size_t NumHangprinterFactors = 9;		// maximum number of machine factors we can adjust
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (numFactors != 3 && numFactors != 6 && numFactors != NumHangprinterFactors)
	{
		reply.printf("Hangprinter calibration with %d factors requested but only 3, 6, and 9 supported", numFactors);
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
	FixedMatrix<floatc_t, MaxCalibrationPoints, 3> probeMotorPositions;
	floatc_t corrections[MaxCalibrationPoints];
	floatc_t initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		corrections[i] = 0.0;
		float machinePos[3];
		const floatc_t zp = reprap.GetMove().GetProbeCoordinates(i, machinePos[X_AXIS], machinePos[Y_AXIS], probePoints.PointWasCorrected(i));
		machinePos[Z_AXIS] = 0.0;

		probeMotorPositions(i, A_AXIS) = sqrtf(LineLengthSquared(machinePos, anchorA));
		probeMotorPositions(i, B_AXIS) = sqrtf(LineLengthSquared(machinePos, anchorB));
		probeMotorPositions(i, C_AXIS) = sqrtf(LineLengthSquared(machinePos, anchorC));
		initialSumOfSquares += fcsquare(zp);
	}

	// Do 1 or more Newton-Raphson iterations
	unsigned int iteration = 0;
	float expectedRmsError;
	for (;;)
	{
		// Build a Nx9 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
		FixedMatrix<floatc_t, MaxCalibrationPoints, NumHangprinterFactors> derivativeMatrix;
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
		FixedMatrix<floatc_t, NumHangprinterFactors, NumHangprinterFactors + 1> normalMatrix;
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

		floatc_t solution[NumHangprinterFactors];
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

		Adjust(numFactors, solution);								// adjust the delta parameters

		float heightAdjust[3];
		for (size_t drive = 0; drive < 3; ++drive)
		{
			heightAdjust[drive] = (float)solution[drive];			// convert double to float
		}
		reprap.GetMove().AdjustMotorPositions(heightAdjust, 3);		// adjust the motor positions

		// Calculate the expected probe heights using the new parameters
		{
			floatc_t expectedResiduals[MaxCalibrationPoints];
			floatc_t sumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < 3; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[3];
				InverseTransform(probeMotorPositions(i, A_AXIS), probeMotorPositions(i, B_AXIS), probeMotorPositions(i, C_AXIS), newPosition);
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

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = A, B, C line length adjustments
// 3 = B anchor Y coordinate
// 4, 5 = C anchor X and Y coordinates
// 6, 7, 8 = A, B and C anchor Z coordinates
floatc_t HangprinterKinematics::ComputeDerivative(unsigned int deriv, float La, float Lb, float Lc) const
{
	const float perturb = 0.2;			// perturbation amount in mm
	HangprinterKinematics hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		// Line length corrections
		break;

	case 3:
		hiParams.anchorB[1] += perturb;
		loParams.anchorB[1] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 4:
		hiParams.anchorC[0] += perturb;
		loParams.anchorC[0] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 5:
		hiParams.anchorC[1] += perturb;
		loParams.anchorC[1] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 6:
		hiParams.anchorA[2] += perturb;
		loParams.anchorA[2] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 7:
		hiParams.anchorB[2] += perturb;
		loParams.anchorB[2] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 8:
		hiParams.anchorB[2] += perturb;
		loParams.anchorB[2] -= perturb;
		hiParams.Recalc();
		loParams.Recalc();
		break;
	}

	float newPos[3];
	hiParams.InverseTransform((deriv == 0) ? La + perturb : La, (deriv == 1) ? Lb + perturb : Lb, (deriv == 2) ? Lc + perturb : Lc, newPos);

	const float zHi = newPos[Z_AXIS];
	loParams.InverseTransform((deriv == 0) ? La - perturb : La, (deriv == 1) ? Lb - perturb : Lb, (deriv == 2) ? Lc - perturb : Lc, newPos);
	const float zLo = newPos[Z_AXIS];
	return ((floatc_t)zHi - (floatc_t)zLo)/(floatc_t)(2 * perturb);
}

// Perform 3, 6 or 9-factor adjustment.
// The input vector contains the following parameters in this order:
// 0, 1, 2 = A, B, C line length adjustments
// 3 = B anchor Y coordinate
// 4, 5 = C anchor X and Y coordinates
// 6, 7, 8 = A, B and C anchor Z coordinates
void HangprinterKinematics::Adjust(size_t numFactors, const floatc_t v[])
{
	if (numFactors >= 4)
	{
		anchorB[1] += (float)v[3];
	}
	if (numFactors >= 5)
	{
		anchorC[0] += (float)v[4];
	}
	if (numFactors >= 6)
	{
		anchorC[1] += (float)v[5];
	}
	if (numFactors >= 7)
	{
		anchorA[2] += (float)v[6];
	}
	if (numFactors >= 8)
	{
		anchorB[2] += (float)v[7];
	}
	if (numFactors >= 9)
	{
		anchorC[2] += (float)v[8];
	}

	Recalc();
}

// Print all the parameters for debugging
void HangprinterKinematics::PrintParameters(const StringRef& reply) const
{
	reply.printf("Anchor coordinates (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)\n",
					(double)anchorA[X_AXIS], (double)anchorA[Y_AXIS], (double)anchorA[Z_AXIS],
					(double)anchorB[X_AXIS], (double)anchorB[Y_AXIS], (double)anchorB[Z_AXIS],
					(double)anchorC[X_AXIS], (double)anchorC[Y_AXIS], (double)anchorC[Z_AXIS]);
}

// End
