/*
 * DeltaParameters.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef DELTAPARAMETERS_H_
#define DELTAPARAMETERS_H_

// Class to hold the parameter for a delta machine.
class DeltaParameters
{
public:
	DeltaParameters() { Init(); }

	bool IsDeltaMode() const { return deltaMode; }
	float GetDiagonal() const { return diagonal; }
	float GetRadius() const { return radius; }
    float GetPrintRadius() const { return printRadius; }
    float GetXCorrection() const { return xCorrection; }
    float GetYCorrection() const { return yCorrection; }
    float GetZCorrection() const { return zCorrection; }
    float GetTowerX(size_t axis) const { return towerX[axis]; }
    float GetTowerY(size_t axis) const { return towerY[axis]; }
    float GetEndstopAdjustment(size_t axis) const { return endstopAdjustments[axis]; }
    float GetHomedCarriageHeight(size_t axis) const { return homedCarriageHeight + endstopAdjustments[axis]; }
    float GetPrintRadiusSquared() const { return printRadiusSquared; }

    void Init();
    void SetDiagonal(float d) { diagonal = d; Recalc(); }
    void SetRadius(float r) { radius = r; Recalc(); }
    void SetEndstopAdjustment(size_t axis, float x) { endstopAdjustments[axis] = x; Recalc(); }
    void SetPrintRadius(float r) { printRadius = r; printRadiusSquared = r * r; }
    float GetHomedHeight() const { return homedHeight; }
    void SetHomedHeight(float h) { homedHeight = h; Recalc(); }
    void SetXCorrection(float angle) { xCorrection = angle; Recalc(); }
    void SetYCorrection(float angle) { yCorrection = angle; Recalc(); }
    void SetZCorrection(float angle) { zCorrection = angle; Recalc(); }

    float Transform(const float machinePos[AXES], size_t axis) const;				// Calculate the motor position for a single tower from a Cartesian coordinate
    void InverseTransform(float Ha, float Hb, float Hc, float machinePos[AXES]) const;	// Calculate the Cartesian position from the motor positions

    float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);		// Compute the derivative of height with respect to a parameter at a set of motor endpoints
    void Adjust(size_t numFactors, const float v[]);								// Perform 3-, 4-, 6- or 7-factor adjustment
    void PrintParameters(StringRef& reply) const;									// Print all the parameters for debugging

private:
	void Recalc();
	void NormaliseEndstopAdjustments();												// Make the average of the endstop adjustments zero

	const float degreesToRadians = PI/180.0;
	const float radiansToDegrees = 180.0/PI;

	// Core parameters
    float diagonal;										// The diagonal rod length, all 3 are assumed to be the same length
    float radius;										// The nominal delta radius, before any fine tuning of tower positions
    float xCorrection, yCorrection, zCorrection;		// Tower position corrections
    float endstopAdjustments[AXES];						// How much above or below the ideal position each endstop is
    float printRadius;
    float homedHeight;

    // Derived values
    bool deltaMode;										// True if this is a delta printer
    float towerX[AXES];									// The X coordinate of each tower
    float towerY[AXES];									// The Y coordinate of each tower
    float printRadiusSquared;
    float homedCarriageHeight;
	float Xbc, Xca, Xab, Ybc, Yca, Yab;
	float coreFa, coreFb, coreFc;
    float Q, Q2, D2;
};

#endif /* DELTAPARAMETERS_H_ */
