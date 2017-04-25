/*
 * DeltaParameters.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include "DeltaKinematics.h"
#include "Pins.h"
#include "Configuration.h"
#include "Storage/FileStore.h"

DeltaKinematics::DeltaKinematics() : Kinematics(CART_AXES, DELTA_AXES)
{
	Init();
}

// Return the name of the current kinematics
const char *DeltaKinematics::GetName() const
{
	return "Delta";
}

void DeltaKinematics::Init()
{
    deltaMode = false;
	diagonal = 0.0;
	radius = 0.0;
	xTilt = yTilt = 0.0;
	printRadius = defaultPrintRadius;
	homedHeight = defaultDeltaHomedHeight;

    for (size_t axis = 0; axis < DELTA_AXES; ++axis)
    {
    	angleCorrections[axis] = 0.0;
    	endstopAdjustments[axis] = 0.0;
    	towerX[axis] = towerY[axis] = 0.0;
    }
}

void DeltaKinematics::Recalc()
{
	deltaMode = (radius > 0.0 && diagonal > radius);
	if (deltaMode)
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
}

// Make the average of the endstop adjustments zero, without changing the individual homed carriage heights
void DeltaKinematics::NormaliseEndstopAdjustments()
{
	const float eav = (endstopAdjustments[A_AXIS] + endstopAdjustments[B_AXIS] + endstopAdjustments[C_AXIS])/3.0;
	endstopAdjustments[A_AXIS] -= eav;
	endstopAdjustments[B_AXIS] -= eav;
	endstopAdjustments[C_AXIS] -= eav;
	homedHeight += eav;
	homedCarriageHeight += eav;				// no need for a full recalc, this is sufficient
}

// Calculate the motor position for a single tower from a Cartesian coordinate.
float DeltaKinematics::Transform(const float machinePos[DELTA_AXES], size_t axis) const
{
	return sqrt(D2 - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]))
		 + machinePos[Z_AXIS]
		 + (machinePos[X_AXIS] * xTilt)
		 + (machinePos[Y_AXIS] * yTilt);
}

// Calculate the Cartesian coordinates from the motor coordinates.
void DeltaKinematics::InverseTransform(float Ha, float Hb, float Hc, float machinePos[DELTA_AXES]) const
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

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = X, Y, Z tower endstop adjustments
// 3 = delta radius
// 4 = X tower correction
// 5 = Y tower correction
// 6 = diagonal rod length
// 7, 8 = X tilt, Y tilt. We scale these by the printable radius to get sensible values in the range -1..1
floatc_t DeltaKinematics::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc)
{
	const float perturb = 0.2;			// perturbation amount in mm or degrees
	DeltaKinematics hiParams(*this), loParams(*this);
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
void DeltaKinematics::Adjust(size_t numFactors, const floatc_t v[])
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

void DeltaKinematics::PrintParameters(StringRef& reply) const
{
	reply.printf("Stops X%.3f Y%.3f Z%.3f height %.3f diagonal %.3f radius %.3f xcorr %.2f ycorr %.2f zcorr %.2f xtilt %.3f%% ytilt %.3f%%\n",
					endstopAdjustments[A_AXIS], endstopAdjustments[B_AXIS], endstopAdjustments[C_AXIS], homedHeight, diagonal, radius,
					angleCorrections[A_AXIS], angleCorrections[B_AXIS], angleCorrections[C_AXIS], xTilt * 100.0, yTilt * 100.0);
}

// Write parameters to file if in delta mode, returning true if no error
// Values are written in mm
bool DeltaKinematics::WriteParameters(FileStore *f) const
{
	if (!IsDeltaMode())
	{
		return true;
	}

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
			endstopAdjustments[X_AXIS], endstopAdjustments[Y_AXIS], endstopAdjustments[Z_AXIS],
				GetXTilt() * 100.0, GetYTilt() * 100.0);
		ok = f->Write(scratchString.Pointer());
	}
	return ok;
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool DeltaKinematics::SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) /*override*/
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
		return Kinematics::SetOrReportParameters(mCode, gb, reply, error);
	}
}

// End



