/*
 * SX1509B.h
 *
 *  Created on: 19 Oct 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_SX1509B_H_
#define SRC_DUETNG_SX1509B_H_

#include "Wire.h"

enum class ExpansionBoardType : uint8_t
{
	none,
	DueX0,
	DueX2,
	DueX5
};

class SX1509B
{
public:
	ExpansionBoardType Init();			// Initialise the device and identify which expansion board (if any) is attached
};

#endif /* SRC_DUETNG_SX1509B_H_ */
