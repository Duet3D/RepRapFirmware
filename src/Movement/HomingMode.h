/*
 * HomingMode.h
 *
 *  Created on: 1 Jan 2025
 *      Author: David
 */

#ifndef SRC_MOVEMENT_HOMINGMODE_H_
#define SRC_MOVEMENT_HOMINGMODE_H_

#include <cstdint>

// Class used to define homing mode
enum class HomingMode : uint8_t
{
	homeCartesianAxes,
	homeIndividualDrives,
};

#endif /* SRC_MOVEMENT_HOMINGMODE_H_ */
