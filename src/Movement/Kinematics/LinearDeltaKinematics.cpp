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


LinearDeltaKinematics::LinearDeltaKinematics() : Kinematics(KinematicsType::linearDelta, -1.0, 0.0, true), numTowers(UsualNumTowers)
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
	radius = DefaultDeltaRadius;
	xTilt = yTilt = 0.0;
	printRadius = DefaultPrintRadius;
	homedHeight = DefaultDeltaHomedHeight;
    doneAutoCalibration = false;

	for (size_t axis = 0; axis < UsualNumTowers; ++axis)
	{
		angleCorrections[axis] = 0.0;
	}

	for (size_t axis = 0; axis < MaxTowers; ++axis)
	{
		diagonals[axis] = DefaultDiagonal;
		towerX[axis] = towerY[axis] = 0.0;
		endstopAdjustments[axis] = 0.0;
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

	// Calculate the base carriage heights when the printer is homed, i.e. the carriages are at the endstops, and the always-reachable height
	alwaysReachableHeight = homedHeight;
	for (size_t axis = 0; axis < numTowers; ++axis)
	{
		D2[axis] = fsquare(diagonals[axis]);
		homedCarriageHeights[axis] = homedHeight
									+ sqrtf(D2[axis] - ((axis < UsualNumTowers) ? fsquare(radius) : fsquare(towerX[axis]) + fsquare(towerY[axis])))
									+ endstopAdjustments[axis];
		const float heightLimit = homedCarriageHeights[axis] - diagonals[axis];
		if (heightLimit < alwaysReachableHeight)
		{
			alwaysReachableHeight = heightLimit;
		}
	}

	printRadiusSquared = fsquare(printRadius);

	if (reprap.Debug(moduleMove))
	{
		debugPrintf("HCH:");
		for (size_t i = 0; i < numTowers; ++i)
		{
			debugPrintf(" %.2f", (double)homedCarriageHeights[i]);
		}
		debugPrintf("\n");
	}
}

// Make the average of the endstop adjustments zero, without changing the individual homed carriage heights
void LinearDeltaKinematics::NormaliseEndstopAdjustments()
{
	const float eav = (endstopAdjustments[DELTA_A_AXIS] + endstopAdjustments[DELTA_B_AXIS] + endstopAdjustments[DELTA_C_AXIS])/3.0;
	for (size_t i = 0; i < numTowers; ++i)
	{
		endstopAdjustments[i] -= eav;
	}
	homedHeight += eav;
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
float LinearDeltaKinematics::Transform(const float machinePos[], size_t axis) const
{
	if (axis < numTowers)
	{
		return sqrtf(D2[axis] - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]))
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
void LinearDeltaKinematics::ForwardTransform(float Ha, float Hb, float Hc, float machinePos[XYZ_AXES]) const
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
	const float C = fsquare(S + towerX[DELTA_A_AXIS] * Q) + fsquare(P - towerY[DELTA_A_AXIS] * Q) + (fsquare(Ha) - D2[DELTA_A_AXIS]) * Q2;

	const float z = (minusHalfB - sqrtf(fsquare(minusHalfB) - A * C)) / A;
	machinePos[X_AXIS] = (U * z - S) / Q;
	machinePos[Y_AXIS] = (P - R * z) / Q;
	machinePos[Z_AXIS] = z - ((machinePos[X_AXIS] * xTilt) + (machinePos[Y_AXIS] * yTilt));
}

// Convert Cartesian coordinates to motor steps
bool LinearDeltaKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const
{
	bool ok = true;
	for (size_t axis = 0; axis < numTowers; ++axis)
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

	// Transform any additional axes linearly
	for (size_t axis = numTowers; axis < numVisibleAxes; ++axis)
	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}
	return ok;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
void LinearDeltaKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	ForwardTransform(motorPos[DELTA_A_AXIS]/stepsPerMm[DELTA_A_AXIS], motorPos[DELTA_B_AXIS]/stepsPerMm[DELTA_B_AXIS], motorPos[DELTA_C_AXIS]/stepsPerMm[DELTA_C_AXIS], machinePos);

	// Convert any additional axes linearly
	for (size_t drive = numTowers; drive < numVisibleAxes; ++drive)
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
LimitPositionResult LinearDeltaKinematics::LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const
{
	constexpr AxesBitmap allAxes = MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS) | MakeBitmap<AxesBitmap>(Z_AXIS);
	bool limited = false;

	// If axes have been homed on a delta printer and this isn't a homing move, check for movements outside limits.
	// Skip this check if axes have not been homed, so that extruder-only moves are allowed before homing
	if ((axesHomed & allAxes) == allAxes)
	{
		// Constrain the move to be within the build radius
		const float diagonalSquared = fsquare(finalCoords[X_AXIS]) + fsquare(finalCoords[Y_AXIS]);
		if (diagonalSquared > printRadiusSquared)
		{
			const float factor = sqrtf(printRadiusSquared / diagonalSquared);
			finalCoords[X_AXIS] *= factor;
			finalCoords[Y_AXIS] *= factor;
			limited = true;
		}

		// Constrain the position to be within the reachable height
		if (initialCoords == nullptr)
		{
			// Asking to limit a single position
			if (finalCoords[Z_AXIS] > alwaysReachableHeight)
			{
				for (size_t tower = 0; tower < UsualNumTowers; ++tower)
				{
					const float carriageHeight = Transform(finalCoords, tower);
					if (carriageHeight > homedCarriageHeights[tower])
					{
						finalCoords[Z_AXIS] -= (carriageHeight - homedCarriageHeights[tower]);
						limited = true;
					}
				}
			}

		}
		else if (finalCoords[Z_AXIS] > alwaysReachableHeight || initialCoords[Z_AXIS] > alwaysReachableHeight)
		{
			// Asking to limit all positions along a straight line
			// Determine the maximum reachable height at the final position and all intermediate positions
			const float dx = finalCoords[X_AXIS] - initialCoords[X_AXIS],
						dy = finalCoords[Y_AXIS] - initialCoords[Y_AXIS];
			const float P2 = fsquare(dx) + fsquare(dy);							// square of the distance moved in the XY plane
			float dz = finalCoords[Z_AXIS] - initialCoords[Z_AXIS];
			float Q2 = P2 + fsquare(dz);										// square of the total distance moved
			if (Q2 != 0.0)														// if there is any XYZ movement
			{
				// If t is the proportion of movement completed from initial to final coordinates, the t-value corresponding to the maximum tower height is:
				// t = (+/- dz*sqrt(d^2*P2 - (dx*(y0-yt)-dy*(x0-xt))^2)*Q
			    //      -(x0-xt)*dx*Q2
			    //      -(y0-yt)*dy*Q2
				//     )
				//	   /(P2*Q2)
				// We want the root that increases with increasing dz, i.e. positive Z movement delays the maximum
				for (size_t tower = 0; tower < numTowers; ++tower)
				{
					const float tx = initialCoords[X_AXIS] - towerX[tower],
								ty = initialCoords[Y_AXIS] - towerY[tower];
					const float discriminant = (D2[tower] * P2) - fsquare((dx * ty) - (dy * tx));
					bool limitFinalHeight;
					bool again;													// we may need to iterate
					do
					{
						again = false;
						if (discriminant < 0.0)
						{
							// There is no maximum carriage height on this tower, so the maximum must occur at the initial or final point.
							// We assume that the initial point is within range, so check the final point.
							limitFinalHeight = true;
						}
						else
						{
							const float tP2Q2 = dz * sqrtf(discriminant * Q2) - ((tx * dx) + (ty * dy)) * Q2;
							const float P2Q2 = P2 * Q2;
							if (tP2Q2 >= P2Q2)
							{
								limitFinalHeight = true;						// the maximum is beyond the final position
							}
							else
							{
								limitFinalHeight = false;
								if (tP2Q2 > 0.0)
								{
									const float t = tP2Q2/P2Q2;
									float tempCoords[XYZ_AXES];
									tempCoords[X_AXIS] = initialCoords[X_AXIS] + t * dx;
									tempCoords[Y_AXIS] = initialCoords[Y_AXIS] + t * dy;
									tempCoords[Z_AXIS] = initialCoords[Z_AXIS] + t * dz;
									const float carriageHeight = Transform(tempCoords, tower);

									if (carriageHeight > homedCarriageHeights[tower])
									{
										// We can't do this move as requested
										const float proposedAdjustment = carriageHeight - homedCarriageHeights[tower] + 0.5;
										if (dz >= proposedAdjustment)
										{
											// There is some chance that if we reduce the requested final Z coordinate, we can do the move
											finalCoords[Z_AXIS] -= proposedAdjustment;
											dz -= proposedAdjustment;;
											limited = true;

											// Update the intermediate variables that have changed
											again = true;
											Q2 = P2 + fsquare(dz);
											if (reprap.Debug(moduleMove))
											{
												debugPrintf("Limit tower %u, t=%.2f\n", tower, (double)t);
											}
										}
										else
										{
											return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
										}
									}
								}
							}
						}
					} while (again);

					if (limitFinalHeight)
					{
						const float carriageHeight = Transform(finalCoords, tower);
						if (carriageHeight > homedCarriageHeights[tower])
						{
							const float proposedAdjustment = carriageHeight - homedCarriageHeights[tower];
							if (dz >= proposedAdjustment)
							{
								finalCoords[Z_AXIS] -= proposedAdjustment;
								limited = true;
								if (reprap.Debug(moduleMove))
								{
									debugPrintf("Limit tower %u\n", tower);
								}
								if (tower + 1 < numTowers)
								{
									dz -= proposedAdjustment;
									Q2 = P2 + fsquare(dz);
								}
							}
							else
							{
								return (limited) ? LimitPositionResult::adjustedAndIntermediateUnreachable : LimitPositionResult::intermediateUnreachable;
							}
						}
					}
				}
			}
		}

		if (applyM208Limits && finalCoords[Z_AXIS] < reprap.GetPlatform().AxisMinimum(Z_AXIS))
		{
			finalCoords[Z_AXIS] = reprap.GetPlatform().AxisMinimum(Z_AXIS);
			limited = true;
		}
	}

	// Limit any additional axes according to the M208 limits
	if (applyM208Limits && LimitPositionFromAxis(finalCoords, numTowers, numVisibleAxes, axesHomed))
	{
		limited = true;
	}

	return (limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;
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
	FixedMatrix<floatc_t, MaxCalibrationPoints, UsualNumTowers> probeMotorPositions;
	floatc_t corrections[MaxCalibrationPoints];
	floatc_t initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		corrections[i] = 0.0;
		float machinePos[XYZ_AXES];
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
				const floatc_t d =
					ComputeDerivative(adjustedJ, probeMotorPositions(i, DELTA_A_AXIS), probeMotorPositions(i, DELTA_B_AXIS), probeMotorPositions(i, DELTA_C_AXIS));
				if (isnan(d))			// a couple of users have reported getting Nans in the derivative, probably due to points being unreachable
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
			// Save the old homed carriage heights before we change the endstop corrections
			float heightAdjust[UsualNumTowers];
			for (size_t drive = 0; drive < UsualNumTowers; ++drive)
			{
				heightAdjust[drive] = homedCarriageHeights[drive];
			}

			Adjust(numFactors, solution);	// adjust the delta parameters

			for (size_t drive = 0; drive < UsualNumTowers; ++drive)
			{
				heightAdjust[drive] = homedCarriageHeights[drive] - heightAdjust[drive];
			}

			// Adjust the motor endpoints to allow for the change to endstop adjustments
			reprap.GetMove().AdjustMotorPositions(heightAdjust, UsualNumTowers);
		}

		// Calculate the expected probe heights using the new parameters
		{
			floatc_t expectedResiduals[MaxCalibrationPoints];
			floatc_t sumOfSquares = 0.0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				for (size_t axis = 0; axis < UsualNumTowers; ++axis)
				{
					probeMotorPositions(i, axis) += solution[axis];
				}
				float newPosition[XYZ_AXES];
				ForwardTransform(probeMotorPositions(i, DELTA_A_AXIS), probeMotorPositions(i, DELTA_B_AXIS), probeMotorPositions(i, DELTA_C_AXIS), newPosition);
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
	return (axis < numTowers) ? MotionType::segmentFreeDelta : MotionType::linear;
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
		for (size_t tower = 0; tower < UsualNumTowers; ++tower)
		{
			hiParams.diagonals[tower] += perturb;
			loParams.diagonals[tower] -= perturb;
		}
		hiParams.Recalc();
		loParams.Recalc();
		break;

	case 7:
	case 8:
		// X and Y tilt
		break;
	}

	float newPos[XYZ_AXES];
	hiParams.ForwardTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
	if (deriv == 7)
	{
		return -newPos[X_AXIS]/printRadius;
	}
	if (deriv == 8)
	{
		return -newPos[Y_AXIS]/printRadius;
	}

	const float zHi = newPos[Z_AXIS];
	loParams.ForwardTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
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
				for (size_t tower = 0; tower < UsualNumTowers; ++tower)
				{
					diagonals[tower] += (float)v[6];
				}
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
	}

	Recalc();

	// Note: if we adjusted the X and Y tilts, and there are any endstop adjustments, then the homed position won't be exactly in the centre
	// and changing the tilt will therefore affect the homed height. We ignore this for now. If it is ever significant, a second autocalibration
	// run will correct it.
}

// Print all the parameters for debugging
void LinearDeltaKinematics::PrintParameters(const StringRef& reply) const
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f height %.3f diagonals",
		(double)endstopAdjustments[DELTA_A_AXIS], (double)endstopAdjustments[DELTA_B_AXIS], (double)endstopAdjustments[DELTA_C_AXIS], (double)homedHeight);
	for (size_t tower = 0; tower < numTowers; ++tower)
	{
		reply.catf("%c%.3f", (tower == 0) ? ' ' : ':', (double)diagonals[tower]);
	}
	reply.catf(" radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f xtilt %.3f%% ytilt %.3f%%\n",
		(double)radius,
		(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS],
		(double)(xTilt * 100.0), (double)(yTilt * 100.0));
}

// Write the parameters that are set by auto calibration to a file, returning true if success
bool LinearDeltaKinematics::WriteCalibrationParameters(FileStore *f) const
{
	bool ok = f->Write("; Delta parameters\n");
	if (ok)
	{
		String<ScratchStringLength> scratchString;
		scratchString.copy("M665 ");
		for (size_t tower = 0; tower < numTowers; ++tower)
		{
			scratchString.catf("%c%.3f", (tower == 0) ? 'L' : ':', (double)diagonals[tower]);
		}

		scratchString.catf(" R%.3f H%.3f B%.1f X%.3f Y%.3f Z%.3f\n",
			(double)radius, (double)homedHeight, (double)printRadius,
			(double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
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
				seen = true;
				size_t numValues = MaxAxes;
				gb.GetFloatArray(diagonals, numValues, false);
				while (numValues < 3)
				{
					diagonals[numValues++] = diagonals[0];
				}
				numTowers = numValues;
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
				reply.copy("Diagonals");
				for (size_t tower = 0; tower < numTowers; ++tower)
				{
					reply.catf("%c%.3f", (tower == 0) ? ' ' : ':', (double)diagonals[tower]);
				}
				reply.catf(", delta radius %.3f, homed height %.3f, bed radius %.1f"
							 ", X %.3f" DEGREE_SYMBOL ", Y %.3f" DEGREE_SYMBOL ", Z %.3f" DEGREE_SYMBOL,
							 (double)radius,
							 (double)homedHeight, (double)printRadius,
							 (double)angleCorrections[DELTA_A_AXIS], (double)angleCorrections[DELTA_B_AXIS], (double)angleCorrections[DELTA_C_AXIS]);
			}
			return seen;
		}

	case 666:
		{
			bool seen = false;
			for (size_t tower = 0; tower < numTowers; ++tower)
			{
				gb.TryGetFValue("XYZUVW"[tower], endstopAdjustments[tower], seen);
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

			if (seen)
			{
				Recalc();
			}
			else
			{
				reply.printf("Endstop adjustments X%.2f Y%.2f Z%.2f, tilt X%.2f%% Y%.2f%%",
					(double)endstopAdjustments[X_AXIS], (double)endstopAdjustments[Y_AXIS], (double)endstopAdjustments[Z_AXIS],
					(double)(xTilt * 100.0), (double)(yTilt * 100.0));
			}
			return seen;
		}

	case 669:
		{
			// X and Y give the X and Y coordinates of the additional towers beyond the first three
			// The correct number of L parameters must have been given in the M665 command first
			size_t numX = 0, numY = 0;
			if (gb.Seen('X'))
			{
				numX = MaxTowers - UsualNumTowers;
				gb.GetFloatArray(towerX + UsualNumTowers, numX, false);
				if (numX != numTowers - UsualNumTowers)
				{
					reply.copy("Wrong number of X values");
					error = true;
					return true;
				}
			}
			if (gb.Seen('Y'))
			{
				numY = MaxTowers - UsualNumTowers;
				gb.GetFloatArray(towerY + UsualNumTowers, numY, false);
				if (numY != numTowers - UsualNumTowers)
				{
					reply.copy("Wrong number of Y values");
					error = true;
					return true;
				}
			}
			if (numX != 0 || numY != 0)
			{
				Recalc();				// recalculate the homed carriage heights
				return true;
			}
			return Kinematics::Configure(mCode, gb, reply, error);
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
AxesBitmap LinearDeltaKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const
{
	// If homing X, Y or Z we must home all the towers
	if ((toBeHomed & LowestNBits<AxesBitmap>(XYZ_AXES)) != 0)
	{
		filename.copy("homedelta.g");
		return 0;
	}

	return Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
}

// This function is called from the step ISR when an endstop switch is triggered during homing.
// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
bool LinearDeltaKinematics::QueryTerminateHomingMove(size_t axis) const
{
	return false;
}

// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate().
void LinearDeltaKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const
{
	if (axis < numTowers)
	{
		if (highEnd)
		{
			dda.SetDriveCoordinate(lrintf(homedCarriageHeights[axis] * stepsPerMm[axis]), axis);
		}
	}
	else
	{
		// Assume that any additional axes are linear
		const float hitPoint = (highEnd) ? reprap.GetPlatform().AxisMaximum(axis) : reprap.GetPlatform().AxisMinimum(axis);
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
	}
}

// Limit the speed and acceleration of a move to values that the mechanics can handle.
// The speeds in Cartesian space have already been limited.
void LinearDeltaKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const
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
// The DDA class has special support for delta printers, so we can baystep the Z axis.
AxesBitmap LinearDeltaKinematics::GetLinearAxes() const
{
	return MakeBitmap<AxesBitmap>(Z_AXIS);
}

// End
