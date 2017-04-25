/*
 * DeltaParameters.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef DELTAKINEMATICS_H_
#define DELTAKINEMATICS_H_

#include "RepRapFirmware.h"
#include "Kinematics.h"

#ifdef DUET_NG
typedef double floatc_t;					// type of matrix element used for delta calibration
#else
// We are more memory-constrained on the SAM3X
typedef float floatc_t;						// type of matrix element used for delta calibration
#endif

const size_t DELTA_AXES = 3;
const size_t A_AXIS = 0;
const size_t B_AXIS = 1;
const size_t C_AXIS = 2;

// Class to hold the parameter for a delta machine.
class DeltaKinematics : public Kinematics
{
public:
	// Constructors
	DeltaKinematics();

    // Overridden base class functions
	const char *GetName() const;													// Return the name of the current kinematics
	bool SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;	// Set or report the parameters from a M665, M666 or M669 command
    float Transform(const float headPos[], size_t axis) const override;				// Calculate the motor position for a single tower from a Cartesian coordinate
    void InverseTransform(float Ha, float Hb, float Hc, float headPos[]) const override; // Calculate the Cartesian position from the motor positions

    // Public functions specific to this class
    bool IsDeltaMode() const { return deltaMode; }
	float GetDiagonal() const { return diagonal; }
    float GetTowerX(size_t axis) const { return towerX[axis]; }
    float GetTowerY(size_t axis) const { return towerY[axis]; }
    float GetHomedCarriageHeight(size_t axis) const { return homedCarriageHeight + endstopAdjustments[axis]; }
    float GetPrintRadiusSquared() const { return printRadiusSquared; }
    float GetXTilt() const { return xTilt; }
    float GetYTilt() const { return yTilt; }

	void Init();
	float GetHomedHeight() const { return homedHeight; }

	floatc_t ComputeDerivative(unsigned int deriv, float ha, float hb, float hc);	// Compute the derivative of height with respect to a parameter at a set of motor endpoints
	void Adjust(size_t numFactors, const floatc_t v[]);								// Perform 3-, 4-, 6- or 7-factor adjustment
	void PrintParameters(StringRef& reply) const;									// Print all the parameters for debugging
	bool WriteParameters(FileStore *f) const;										// Write parameters to file if in delta mode, returning true if no error

private:
	void Recalc();
	void NormaliseEndstopAdjustments();												// Make the average of the endstop adjustments zero

	// Delta parameter defaults
	const float defaultPrintRadius = 50;							// mm
	const float defaultDeltaHomedHeight = 200;						// mm

	// Core parameters
    float diagonal;										// The diagonal rod length, all 3 are assumed to be the same length
    float radius;										// The nominal delta radius, before any fine tuning of tower positions
    float angleCorrections[DELTA_AXES];					// Tower position corrections
    float endstopAdjustments[DELTA_AXES];				// How much above or below the ideal position each endstop is
    float printRadius;
    float homedHeight;
    float xTilt, yTilt;									// How much we need to raise Z for each unit of movement in the +X and +Y directions

    // Derived values
    bool deltaMode;										// True if this is a delta printer
    float towerX[DELTA_AXES];							// The X coordinate of each tower
    float towerY[DELTA_AXES];							// The Y coordinate of each tower
    float printRadiusSquared;
    float homedCarriageHeight;
	float Xbc, Xca, Xab, Ybc, Yca, Yab;
	float coreFa, coreFb, coreFc;
    float Q, Q2, D2;
};

#endif /* DELTAKINEMATICS_H_ */
