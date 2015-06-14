/*
 * DeltaParameters.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef DELTAPARAMETERS_H_
#define DELTAPARAMETERS_H_

// Class to hold the parameter for a delta machine.
// Some of the values that are currently calculated on demand could be pre-calculated in Recalc() and stored instead.
class DeltaParameters
{
public:
	DeltaParameters() { Init(); }

	bool IsDeltaMode() const { return deltaMode; }
	bool IsEquilateral() const { return isEquilateral; }
	float GetDiagonal() const { return diagonal; }
	float GetRadius() const;
    float GetPrintRadius() const { return printRadius; }
    float GetTowerX(size_t axis) const { return towerX[axis]; }
    float GetTowerY(size_t axis) const { return towerY[axis]; }
    float GetEndstopAdjustment(size_t axis) const { return endstopAdjustments[axis]; }
    float GetHomedCarriageHeight(size_t axis) const { return homedCarriageHeight + endstopAdjustments[axis]; }
    float GetPrintRadiusSquared() const { return printRadiusSquared; }

    void Init();
    void SetDiagonal(float d) { diagonal = d; Recalc(); }
    void SetRadius(float r);
    void SetEndstopAdjustment(size_t axis, float x) { endstopAdjustments[axis] = x; }
    void SetPrintRadius(float r) { printRadius = r; printRadiusSquared = r * r; }
    float GetHomedHeight() const { return homedHeight; }
    void SetHomedHeight(float h) { homedHeight = h; Recalc(); }

    float Transform(const float machinePos[AXES], size_t axis) const;				// Calculate the motor position for a single tower from a Cartesian coordinate
    void InverseTransform(float Ha, float Hb, float Hc, float machinePos[AXES]) const;	// Calculate the Cartesian position from the motor positions

    float ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);		// Compute the derivative of height with respect to a parameter at a set of motor endpoints
    void Adjust(size_t numFactors, const float v[]);								// Perform 4-, 6- or 7-factor adjustment
    void PrintParameters(StringRef& reply) const;									// Print all the parameters for debugging
    float GetXCorrection() const;
    float GetYCorrection() const;

private:
	void Recalc();
	void NormaliseEndstopAdjustments();												// Make the average of the endstop adjustments zero

	// Core parameters
    float diagonal;										// The diagonal rod length, all 3 are assumed to be the same length
    float radius;										// The nominal delta radius, before any fine tuning of tower positions
    float towerX[AXES];									// The X coordinate of each tower
    float towerY[AXES];									// The Y coordinate of each tower
    float endstopAdjustments[AXES];						// How much above or below the ideal position each endstop is
    float printRadius;
    float homedHeight;

    // Derived values
    bool deltaMode;										// True if this is a delta printer
    bool isEquilateral;									// True if the towers are at the corners of an equilateral triangle
    float printRadiusSquared;
    float homedCarriageHeight;
	float Xbc, Xca, Xab, Ybc, Yca, Yab;
	float coreFa, coreFb, coreFc;
    float Q, Q2, D2;
};

#endif /* DELTAPARAMETERS_H_ */
