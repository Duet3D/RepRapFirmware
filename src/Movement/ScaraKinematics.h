/*
 * ScaraKinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_SCARAKINEMATICS_H_
#define SRC_MOVEMENT_SCARAKINEMATICS_H_

#include "Kinematics.h"

class ScaraKinematics : public Kinematics
{
public:
	// Constructors
	ScaraKinematics();

    // Overridden base class functions
	const char *GetName() const;													// Return the name of the current kinematics
	bool SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;	// Set or report the parameters from a M665, M666 or M669 command
    float Transform(const float headPos[], size_t axis) const override;	// Calculate the motor position for a single tower from a Cartesian coordinate
    void InverseTransform(float Ha, float Hb, float Hc, float headPos[]) const override; // Calculate the Cartesian position from the motor positions

private:
    const float DefaultSegmentsPerSecond = 200.0;
    const float DefaultMinSegmentSize = 0.2;
};

#endif /* SRC_MOVEMENT_SCARAKINEMATICS_H_ */
