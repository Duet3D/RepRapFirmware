/*
 * RoundBedKinematics.h
 *
 *  Created on: 16 Feb 2021
 *      Author: manuel
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ROUNDBEDKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ROUNDBEDKINEMATICS_H_

#include "RepRapFirmware.h"
#include "Kinematics.h"
#include <Movement/DDA.h>

class RoundBedKinematics : public Kinematics
{
public:
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;
protected:
	RoundBedKinematics(KinematicsType t, SegmentationType segType) noexcept;

	float printRadiusSquared;
};

#endif /* SRC_MOVEMENT_KINEMATICS_ROUNDBEDKINEMATICS_H_ */
