/*
 * ZLeadscrewKinematics.cpp
 *
 *  Created on: 8 Jul 2017
 *      Author: David
 */

#include "ZLeadscrewKinematics.h"
#include "RepRap.h"
#include "Platform.h"
#include "Movement/Move.h"

ZLeadscrewKinematics::ZLeadscrewKinematics(KinematicsType k) : Kinematics(k), numLeadscrews(0), maxCorrection(1.0)
{
}

// Configure this kinematics. We only deal with the leadscrew coordinates here
bool ZLeadscrewKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error)
{
	if (mCode == 671 && GetKinematicsType() != KinematicsType::coreXZ)
	{
		// Configuring leadscrew positions
		const size_t numZDrivers = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS).numDrivers;
		if (numZDrivers < 2 || numZDrivers > MaxLeadscrews)
		{
			reply.copy("Configure 2 to 4 Z drivers before sending M671");
			return true;
		}

		bool seenX = false, seenY = false;
		if (gb.TryGetFloatArray('X', numZDrivers, leadscrewX, reply, seenX))
		{
			return true;
		}
		if (gb.TryGetFloatArray('Y', numZDrivers, leadscrewY, reply, seenY))
		{
			return true;
		}

		bool seenS;
		gb.TryGetFValue('S', maxCorrection, seenS);

		if (seenX && seenY)
		{
			numLeadscrews = numZDrivers;
			return false;							// successful configuration
		}

		if (seenX || seenY)
		{
			reply.copy("Specify both X and Y coordinates in M671");
			return true;
		}

		// If no parameters provided so just report the existing setup
		if (seenS)
		{
			return true;							// just changed the maximum correction
		}
		else if (numLeadscrews < 2)
		{
			reply.copy("Z leadscrew coordinates are not configured");
		}
		else
		{
			reply.copy("Z leadscrew coordinates");
			for (unsigned int i = 0; i < numLeadscrews; ++i)
			{
				reply.catf(" (%.1f,%.1f)", leadscrewX[i], leadscrewY[i]);
			}
		}
		return false;
	}
	return Kinematics::Configure(mCode, gb, reply, error);
}

// Return true if the kinematics supports auto calibration based on bed probing.
bool ZLeadscrewKinematics::SupportsAutoCalibration() const
{
	return numLeadscrews >= 2;
}

// Perform auto calibration. Override this implementation in kinematics that support it.
void ZLeadscrewKinematics::DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, StringRef& reply)
{
	if (!SupportsAutoCalibration())			// should be checked by caller, but check it here too
	{
		return;
	}

	if (numFactors != numLeadscrews)
	{
		reply.printf("Error: Number of calibration factors (%u) not equal to number of leadscrews (%u)", numFactors, numLeadscrews);
	}

	const size_t numPoints = probePoints.NumberOfProbePoints();

	// Build a Nx4 matrix of derivatives with respect to the leadscrew adjustments
	// See the wxMaxima documents for the maths involved
	FixedMatrix<floatc_t, MaxDeltaCalibrationPoints, MaxLeadscrews> derivativeMatrix;
	floatc_t initialSumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		float x, y;
		const floatc_t zp = reprap.GetMove().GetProbeCoordinates(i, x, y, false);
		initialSumOfSquares += fcsquare(zp);

		switch (numFactors)
		{
		case 2:
			{
				const floatc_t d2 = fcsquare(leadscrewX[1] - leadscrewX[0]) + fcsquare(leadscrewY[1] - leadscrewY[0]);
				// There are lot of common subexpressions in the following, but the optimiser should find them
				derivativeMatrix(i, 0) = (fcsquare(leadscrewY[1]) - leadscrewY[0] * leadscrewY[1] - y * (leadscrewY[1] - leadscrewY[0]) + fcsquare(leadscrewX[1]) - leadscrewX[0] * leadscrewX[1] - x * (leadscrewX[1] - leadscrewX[0]))/d2;
				derivativeMatrix(i, 1) = (fcsquare(leadscrewY[0]) - leadscrewY[0] * leadscrewY[1] + y * (leadscrewY[1] - leadscrewY[0]) + fcsquare(leadscrewX[0]) - leadscrewX[0] * leadscrewX[1] + x * (leadscrewX[1] - leadscrewX[0]))/d2;
			}
			break;

		case 3:
			{
				const floatc_t d2 = leadscrewX[1] * leadscrewY[2] - leadscrewX[0] * leadscrewY[2] - leadscrewX[2] * leadscrewY[1] + leadscrewX[0] * leadscrewY[1] + leadscrewX[2] * leadscrewY[0] - leadscrewX[1] * leadscrewY[0];
				derivativeMatrix(i, 0) = (leadscrewX[1] * leadscrewY[2] - x * leadscrewY[2] - leadscrewX[2] * leadscrewY[1] + x * leadscrewY[1] + leadscrewX[2] * y - leadscrewX[1] * y)/d2;
				derivativeMatrix(i, 1) = (leadscrewX[0] * leadscrewY[2] - x * leadscrewY[2] - leadscrewX[2] * leadscrewY[0] + x * leadscrewY[0] + leadscrewX[2] * y - leadscrewX[0] * y)/d2;
				derivativeMatrix(i, 2) = (leadscrewX[0] * leadscrewY[1] - x * leadscrewY[1] - leadscrewX[1] * leadscrewY[0] + x * leadscrewY[0] + leadscrewX[1] * y - leadscrewX[0] * y)/d2;
			}
			break;

		case 4:
			{
				// This one is horribly complicated. It may not work on the older Duets that use single-precision maths.
				const float &x0 = leadscrewX[0], &x1 = leadscrewX[1], &x2 = leadscrewX[2], &x3 = leadscrewX[3];
				const float &y0 = leadscrewY[0], &y1 = leadscrewY[1], &y2 = leadscrewY[2], &y3 = leadscrewY[3];
				const floatc_t d2 = x1 * x3 * y2 * y3
									- x0 * x3 * y2 * y3
									- x1 * x2 * y2 * y3
									+ x0 * x2 * y2 * y3
									- x2 * x3 * y1 * y3
									+ x0 * x3 * y1 * y3
									+ x1 * x2 * y1 * y3
									- x0 * x1 * y1 * y3
									+ x2 * x3 * y0 * y3
									- x1 * x3 * y0 * y3
									- x0 * x2 * y0 * y3
									+ x0 * x1 * y0 * y3
									+ x2 * x3 * y1 * y2
									- x1 * x3 * y1 * y2
									- x0 * x2 * y1 * y2
									+ x0 * x1 * y1 * y2
									- x2 * x3 * y0 * y2
									+ x0 * x3 * y0 * y2
									+ x1 * x2 * y0 * y2
									- x0 * x1 * y0 * y2
									+ x1 * x3 * y0 * y1
									- x0 * x3 * y0 * y1
									- x1 * x2 * y0 * y1
									+ x0 * x2 * y0 * y1;
				derivativeMatrix(i, 0) = (x1*x3*y2*y3-x*x3*y2*y3-x1*x2*y2*y3+x*x2*y2*y3-x2*x3*y1*y3+x*x3*y1*y3+x1*x2*y1*y3
					-x*x1*y1*y3+x2*x3*y*y3-x1*x3*y*y3-x*x2*y*y3+x*x1*y*y3+x2*x3*y1*y2-x1*x3*y1*y2
					-x*x2*y1*y2+x*x1*y1*y2-x2*x3*y*y2+x*x3*y*y2+x1*x2*y*y2-x*x1*y*y2+x1*x3*y*y1-x*x3*y*y1-x1*x2*y*y1+x*x2*y*y1)/d2;
				derivativeMatrix(i, 1) = -(x0*x3*y2*y3-x*x3*y2*y3-x0*x2*y2*y3+x*x2*y2*y3-x2*x3*y0*y3+x*x3*y0*y3+x0*x2*y0*y3
					-x*x0*y0*y3+x2*x3*y*y3-x0*x3*y*y3-x*x2*y*y3+x*x0*y*y3+x2*x3*y0*y2-x0*x3*y0*
					y2-x*x2*y0*y2+x*x0*y0*y2-x2*x3*y*y2+x*x3*y*y2+x0*x2*y*y2-x*x0*y*y2+x0*x3*y*y0-x*x3*y*y0-x0*x2*y*y0+x*x2*y*y0)/d2;
				derivativeMatrix(i, 2) = (x0*x3*y1*y3-x*x3*y1*y3-x0*x1*y1*y3+x*x1*y1*y3-x1*x3*y0*y3+x*x3*y0*y3+x0*x1*y0*y3-x*x0*y0*y3+x1*x3*y*y3-x0*x3*y*y3-x*x1*y*y3+x*x0*y*y3+x1*x3*y0*y1-x0*x3*y0*y1
					-x*x1*y0*y1+x*x0*y0*y1-x1*x3*y*y1+x*x3*y*y1+x0*x1*y*y1-x*x0*y*y1+x0*x3*y*y0-x*x3*y*y0-x0*x1*y*y0+x*x1*y*y0)/d2;
				derivativeMatrix(i, 3) = -(x0*x2*y1*y2-x*x2*y1*y2-x0*x1*y1*y2+x*x1*y1*y2-x1*x2*y0*y2+x*x2*y0*y2+x0*x1*y0*y2
					-x*x0*y0*y2+x1*x2*y*y2-x0*x2*y*y2-x*x1*y*y2+x*x0*y*y2+x1*x2*y0*y1-x0*x2*y0*
					y1-x*x1*y0*y1+x*x0*y0*y1-x1*x2*y*y1+x*x2*y*y1+x0*x1*y*y1-x*x0*y*y1+x0*x2*y*y0-x*x2*y*y0-x0*x1*y*y0+x*x1*y*y0)/d2;
			}
			break;
		}
	}

	if (reprap.Debug(moduleMove))
	{
		PrintMatrix("Derivative matrix", derivativeMatrix, numPoints, numFactors);
	}

	// Now build the normal equations for least squares fitting
	FixedMatrix<floatc_t, MaxLeadscrews, MaxLeadscrews + 1> normalMatrix;
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
		floatc_t temp = derivativeMatrix(0, i) * -(probePoints.GetZHeight(0));
		for (size_t k = 1; k < numPoints; ++k)
		{
			temp += derivativeMatrix(k, i) * -(probePoints.GetZHeight(k));
		}
		normalMatrix(i, numFactors) = temp;
	}

	if (reprap.Debug(moduleMove))
	{
		PrintMatrix("Normal matrix", normalMatrix, numFactors, numFactors + 1);
	}

	floatc_t solution[MaxLeadscrews];
	normalMatrix.GaussJordan(solution, numFactors);

	if (reprap.Debug(moduleMove))
	{
		PrintMatrix("Solved matrix", normalMatrix, numFactors, numFactors + 1);
		PrintVector("Solution", solution, numFactors);
	}

	// Calculate and display the residuals, also check for errors
	floatc_t residuals[MaxDeltaCalibrationPoints];
	floatc_t sumOfSquares = 0.0;
	for (size_t i = 0; i < numPoints; ++i)
	{
		residuals[i] = probePoints.GetZHeight(i);
		for (size_t j = 0; j < numFactors; ++j)
		{
			residuals[i] += solution[j] * derivativeMatrix(i, j);
		}
		sumOfSquares += fcsquare(residuals[i]);
	}

	if (reprap.Debug(moduleMove))
	{
		PrintVector("Residuals", residuals, numPoints);
	}

	// Check that the corrections are sensible
	bool haveNaN = false, haveLargeCorrection = false;
	for (size_t i = 0; i < numFactors; ++i)
	{
		if (std::isnan(solution[i]))
		{
			haveNaN = true;
		}
		else if (fabs(solution[i]) > maxCorrection)
		{
			haveLargeCorrection = true;
		}
	}

	if (haveNaN)
	{
		reply.printf("Error: calibration failed, computed corrections:");
	}
	else if (haveLargeCorrection)
	{
		reply.printf("Error: computed corrections exceed 1mm:");
	}
	else
	{
		//TODO adjust the motors here
		reply.printf("Simulated calibrating %d leadscrews using %d points, deviation before %.3f after %.3f, corrections:",
						numFactors, numPoints, sqrt(initialSumOfSquares/numPoints), sqrtf(sumOfSquares/numPoints));
	}

	// Append the corrections to the reply in all cases
	for (size_t i = 0; i < numFactors; ++i)
	{
		reply.catf(" %.3f", solution[i]);
	}
}

#ifdef DUET_NG

// Write any calibration data that we need to resume a print after power fail, returning true if successful
bool ZLeadscrewKinematics::WriteResumeSettings(FileStore *f) const
{
	//TODO write leadscrew corrections, there is a chance that they will be the same as before
	return true;
}

#endif
// End
