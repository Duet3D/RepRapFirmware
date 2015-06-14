/*
 * DeltaParameters.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include "RepRapFirmware.h"

void DeltaParameters::Init()
{
    deltaMode = false;
	diagonal = 0.0;
	radius = 0.0;
	printRadius = defaultPrintRadius;
	homedHeight = defaultDeltaHomedHeight;
	isEquilateral = true;

    for (size_t axis = 0; axis < AXES; ++axis)
    {
    	endstopAdjustments[axis] = 0.0;
    	towerX[axis] = towerY[axis] = 0.0;
    }
}

float DeltaParameters::GetRadius() const
{
	if (isEquilateral)
	{
		return radius;
	}
	else
	{
		// Towers have been moved so we need to compute the effective radius
		const float x1 = towerX[1] - towerX[0], x2 = towerX[2] - towerX[0], y1 = towerY[1] - towerY[0], y2 = towerY[2] - towerY[0];
		return sqrt((fsquare(x1) + fsquare(y1)) * (fsquare(x2) + fsquare(y2)) * (fsquare(y1 - y2) + fsquare(x1 - x2)))/(2 * fabs(x1 * y2 - x2 * y1));
	}
}

float DeltaParameters::GetXCorrection() const
{
	return (isEquilateral) ? 0.0 : (acos((towerX[Z_AXIS] - towerX[X_AXIS])/GetRadius()) * (180.0/PI)) - 30;
}

float DeltaParameters::GetYCorrection() const
{
	return (isEquilateral) ? 0.0 : 30 - (acos((towerX[Y_AXIS] - towerX[Z_AXIS])/GetRadius()) * (180.0/PI));
}

void DeltaParameters::SetRadius(float r)
{
	radius = r;
	isEquilateral = true;

	const float cos30 = sqrtf(3.0)/2.0;
	const float sin30 = 0.5;

	towerX[A_AXIS] = -(r * cos30);
	towerX[B_AXIS] = r * cos30;
	towerX[C_AXIS] = 0.0;

	towerY[A_AXIS] = towerY[B_AXIS] = -(r * sin30);
	towerY[C_AXIS] = r;

	Recalc();
}

void DeltaParameters::Recalc()
{
	deltaMode = (radius > 0.0 && diagonal > radius);
	if (deltaMode)
	{
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

		// Calculate the base carriage height when the printer is homed.
		const float tempHeight = diagonal;		// any sensible height will do here, probably even zero
		float machinePos[AXES];
		InverseTransform(tempHeight + endstopAdjustments[X_AXIS], tempHeight + endstopAdjustments[Y_AXIS], tempHeight + endstopAdjustments[X_AXIS],
							machinePos);
		homedCarriageHeight = homedHeight + tempHeight - machinePos[Z_AXIS];
	}
}

// Make the average of the endstop adjustments zero, without changing the individual homed carriage heights
void DeltaParameters::NormaliseEndstopAdjustments()
{
	const float eav = (endstopAdjustments[A_AXIS] + endstopAdjustments[B_AXIS] + endstopAdjustments[C_AXIS])/3.0;
	endstopAdjustments[A_AXIS] -= eav;
	endstopAdjustments[B_AXIS] -= eav;
	endstopAdjustments[C_AXIS] -= eav;
	homedHeight += eav;
	homedCarriageHeight += eav;				// no need for a full recalc, this is sufficient
}

// Calculate the motor position for a single tower from a Cartesian coordinate
float DeltaParameters::Transform(const float machinePos[AXES], size_t axis) const
{
	return machinePos[Z_AXIS]
	          + sqrt(D2 - fsquare(machinePos[X_AXIS] - towerX[axis]) - fsquare(machinePos[Y_AXIS] - towerY[axis]));
}

void DeltaParameters::InverseTransform(float Ha, float Hb, float Hc, float machinePos[AXES]) const
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
	machinePos[Z_AXIS] = z;
}

// Compute the derivative of height with respect to a parameter at the specified motor endpoints.
// 'deriv' indicates the parameter as follows:
// 0, 1, 2 = X, Y, Z tower endstop adjustments
// 3, 4 = X, Y tower X position
// 5 = Z tower Y position
// 6 = diagonal rod length
// 7 = delta radius (only if isEquilateral is true)
float DeltaParameters::ComputeDerivative(unsigned int deriv, float ha, float hb, float hc)
{
	const float perturb = 0.2;			// perturbation amount in mm
	DeltaParameters hiParams(*this), loParams(*this);
	switch(deriv)
	{
	case 0:
	case 1:
	case 2:
		break;

	case 3:
	case 4:
		hiParams.towerX[deriv - 3] += perturb;
		loParams.towerX[deriv - 3] -= perturb;
		break;

	case 5:
		{
			const float yAdj = perturb * (1.0/3.0);
			hiParams.towerY[A_AXIS] -= yAdj;
			hiParams.towerY[B_AXIS] -= yAdj;
			hiParams.towerY[C_AXIS] += (perturb - yAdj);
			loParams.towerY[A_AXIS] += yAdj;
			loParams.towerY[B_AXIS] += yAdj;
			loParams.towerY[C_AXIS] -= (perturb - yAdj);
		}
		break;

	case 6:
		hiParams.diagonal += perturb;
		loParams.diagonal -= perturb;
		break;

	case 7:
		hiParams.SetRadius(radius + perturb);
		loParams.SetRadius(radius - perturb);
		break;
	}

	hiParams.Recalc();
	loParams.Recalc();

	float newPos[AXES];
	hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc, newPos);
	float zHi = newPos[Z_AXIS];
	loParams.InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc, newPos);
	float zLo = newPos[Z_AXIS];

	return (zHi - zLo)/(2 * perturb);
}

// Perform 3, 4, 6 or 7-factor adjustment.
// The input vector contains the following parameters in this order:
//  X, Y and Z endstop adjustments
//  If we are doing 4-factor adjustment, the next argument is the delta radius. Otherwise:
//  X tower X position adjustment
//  Y tower X position adjustment
//  Z tower Y position adjustment
//  Diagonal rod length adjustment
void DeltaParameters::Adjust(size_t numFactors, const float v[])
{
	const float oldCarriageHeightA = GetHomedCarriageHeight(A_AXIS);	// save for later

	// Update endstop adjustments
	endstopAdjustments[A_AXIS] += v[0];
	endstopAdjustments[B_AXIS] += v[1];
	endstopAdjustments[C_AXIS] += v[2];
	NormaliseEndstopAdjustments();

	if (numFactors == 4)
	{
		// 4-factor adjustment, so update delta radius
		SetRadius(radius + v[3]);		// this sets isEquilateral true, recalculates tower positions, then calls Recalc()
	}
	else if (numFactors > 3)
	{
		// 6- or 7-factor adjustment
		towerX[A_AXIS] += v[3];
		towerX[B_AXIS] += v[4];

		const float yAdj = v[5] * (1.0/3.0);
		towerY[A_AXIS] -= yAdj;
		towerY[B_AXIS] -= yAdj;
		towerY[C_AXIS] += (v[5] - yAdj);
		isEquilateral = false;

		if (numFactors == 7)
		{
			diagonal += v[6];
		}

		Recalc();
	}

	// Adjusting the diagonal and the tower positions affects the homed carriage height.
	// We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
	const float heightError = GetHomedCarriageHeight(A_AXIS) - oldCarriageHeightA - v[0];
	homedHeight -= heightError;
	homedCarriageHeight -= heightError;
}

void DeltaParameters::PrintParameters(StringRef& reply) const
{
	reply.printf("Endstops X%.2f Y%.2f Z%.2f, height %.2f, diagonal %.2f, radius %.2f",
					endstopAdjustments[A_AXIS], endstopAdjustments[B_AXIS], endstopAdjustments[C_AXIS], homedHeight, diagonal, GetRadius());
	if (isEquilateral)
	{
		reply.cat("\n");
	}
	else
	{
		reply.catf(", towers (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)\n",
						towerX[A_AXIS], towerY[A_AXIS], towerX[B_AXIS], towerY[B_AXIS], towerX[C_AXIS], towerY[C_AXIS]);
	}
}

// End



