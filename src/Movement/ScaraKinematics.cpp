/*
 * ScaraKinematics.cpp
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#include "ScaraKinematics.h"

ScaraKinematics::ScaraKinematics() : Kinematics(3, 3, DefaultSegmentsPerSecond, DefaultMinSegmentSize, true)
{
	// TODO Auto-generated constructor stub
}

// Return the name of the current kinematics
const char *ScaraKinematics::GetName() const
{
	return "Scara";
}

// Calculate the position for a single motor from Cartesian coordinates
float ScaraKinematics::Transform(const float headPos[], size_t axis) const
{
	//TODO
	return 0.0;
}

// Calculate the Cartesian position from the motor positions
void ScaraKinematics::InverseTransform(float Ha, float Hb, float Hc, float headPos[]) const
{
	//TODO
}

// Set the parameters from a M665, M666 or M669 command
// Return true if we changed any parameters. Set 'error' true if there was an error, otherwise leave it alone.
bool ScaraKinematics::SetOrReportParameters(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) /*override*/
{
	if (mCode == 669)
	{
		//TODO
		return false;
	}
	else
	{
		return Kinematics::SetOrReportParameters(mCode, gb, reply, error);
	}
}

// End
