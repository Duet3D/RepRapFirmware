/*
 * CoreBaseKinematics.h
 *
 *  Created on: 7 May 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_

#include "ZLeadscrewKinematics.h"

class CoreBaseKinematics : public ZLeadscrewKinematics
{
public:
	CoreBaseKinematics(KinematicsType t);

	// Overridden base class functions. See Kinematics.h for descriptions.
	bool Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;
	HomingMode GetHomingMode() const override { return homeCartesianAxes; }

protected:
	float axisFactors[MaxAxes];			// allow more than just XYZ so that we can support e.g. CoreXYU kinematics
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_ */
