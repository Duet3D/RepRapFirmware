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
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	HomingMode GetHomingMode() const override { return homeCartesianAxes; }
	bool QueryTerminateHomingMove(size_t axis) const override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const override;

protected:
	// Return true if the specified endstop axis uses shared motors.
	// Used to determine whether to abort the whole move or just one motor when an endstop switch is triggered.
	virtual bool DriveIsShared(size_t drive) const = 0;

	float axisFactors[MaxAxes];			// allow more than just XYZ so that we can support e.g. CoreXYU kinematics
};

#endif /* SRC_MOVEMENT_KINEMATICS_COREBASEKINEMATICS_H_ */
