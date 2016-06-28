/*
 * ExternalDrivers.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef TMC2660_H_
#define TMC2660_H_

#include "Pins.h"

namespace TMC2660
{
	void Init(const Pin[DRIVES]);
	void SetCurrent(size_t drive, float current);
	void EnableDrive(size_t drive, bool en);
	uint32_t GetStatus(size_t drive);
	bool SetMicrostepping(size_t drive, int microsteps, int mode);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation);
	void SetDriversPowered(bool powered);
};

#endif /* TMC2660_H_ */
