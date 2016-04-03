/*
 * ExternalDrivers.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef EXTERNALDRIVERS_H_
#define EXTERNALDRIVERS_H_

namespace ExternalDrivers
{
	void Init();
	void SetCurrent(size_t drive, float current);
	void EnableDrive(size_t drive, bool en);
	uint32_t GetStatus(size_t drive);
	bool SetMicrostepping(size_t drive, int microsteps, int mode);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation);
};

#endif /* EXTERNALDRIVERS_H_ */
