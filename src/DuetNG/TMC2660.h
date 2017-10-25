/*
 * ExternalDrivers.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef TMC2660_H_
#define TMC2660_H_

#include "Pins.h"
#include "Libraries/General/StringRef.h"

// TMC2660 Read response. The microstep counter can also be read, but we don't include that here.
const uint32_t TMC_RR_SG = 1 << 0;		// stall detected
const uint32_t TMC_RR_OT = 1 << 1;		// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1 << 2;	// over temperature warning
const uint32_t TMC_RR_S2G = 3 << 3;		// short to ground counter (2 bits)
const uint32_t TMC_RR_OLA = 1 << 5;		// open load A
const uint32_t TMC_RR_OLB = 1 << 6;		// open load B
const uint32_t TMC_RR_STST = 1 << 7;	// standstill detected

namespace SmartDrivers
{
	void Init(const Pin[DRIVES], size_t numTmcDrivers)
		pre(numTmcDrivers <= DRIVES);
	void SetCurrent(size_t drive, float current);
	void EnableDrive(size_t drive, bool en);
	uint32_t GetStatus(size_t drive);
	bool SetMicrostepping(size_t drive, int microsteps, int mode);
	unsigned int GetMicrostepping(size_t drive, int mode, bool& interpolation);
	void SetDriversPowered(bool powered);
	void SetStallThreshold(size_t drive, int sgThreshold);
	void SetStallFilter(size_t drive, bool sgFilter);
	void SetCoolStep(size_t drive, uint16_t coolStepConfig);
	void AppendStallConfig(size_t drive, StringRef& reply);
};

#endif /* TMC2660_H_ */
