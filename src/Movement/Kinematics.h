/*
 * Kinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_H_

#include "GCodes/GCodeBuffer.h"

class Kinematics
{
public:
	Kinematics(unsigned int nHeadAxes, unsigned int nMachineAxes);
	Kinematics(unsigned int nHeadAxes, unsigned int nMachineAxes, float segsPerSecond, float minSegLength, bool doUseRawG0);

	virtual const char *GetName() const = 0;	// Return the name of the current kinematics
	virtual bool SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error);	// Set or report the parameters from a M665, M666 or M669 command
    virtual float Transform(const float headPos[], size_t axis) const = 0;						// Calculate the motor position for a single tower from a Cartesian coordinate
    virtual void InverseTransform(float Ha, float Hb, float Hc, float headPos[]) const = 0;		// Calculate the Cartesian position from the motor positions

    bool UseSegmentation() const { return useSegmentation; }
    bool UseRawG0() const { return useRawG0; }
    float GetSegmentsPerSecond() const { return segmentsPerSecond; }
    float GetMinSegmentLength() const { return minSegmentLength; }

private:
    unsigned int numHeadAxes;				// the number of coordinates we have in Cartesian space, usually 3
    unsigned int numMachineAxes;			// the number of motors that have to be set independently, usually the same as numHeadAxes
    bool useSegmentation;					// true if we have to approximate linear movement using segmentation
    bool useRawG0;							// true if we normally use segmentation but we do not need to segment G0 moves
    float segmentsPerSecond;				// if we are using segmentation, the target number of segments/second
    float minSegmentLength;					// if we are using segmentation, the minimum segment size
};

#endif /* SRC_MOVEMENT_KINEMATICS_H_ */
