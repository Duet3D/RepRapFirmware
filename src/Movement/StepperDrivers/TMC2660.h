/*
 * TMC2660.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef TMC2660_H_
#define TMC2660_H_

#if SUPPORT_TMC2660

#include "RepRapFirmware.h"
#include "DriverMode.h"
#include "Pins.h"
#include "MessageType.h"

// TMC2660 read response bits that are returned by the status calls
const uint32_t TMC_RR_SG = 1 << 0;			// stall detected
const uint32_t TMC_RR_OT = 1 << 1;			// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1 << 2;		// over temperature warning
const uint32_t TMC_RR_S2G = 3 << 3;			// short to ground counter (2 bits)
const uint32_t TMC_RR_OLA = 1 << 5;			// open load A
const uint32_t TMC_RR_OLB = 1 << 6;			// open load B
const uint32_t TMC_RR_STST = 1 << 7;		// standstill detected

namespace SmartDrivers
{
	void Init(const Pin[NumDirectDrivers], size_t numTmcDrivers)
		pre(numTmcDrivers <= NumDirectDrivers);
	void Spin(bool powered);
	void TurnDriversOff();

	void SetAxisNumber(size_t driver, uint32_t axisNumber);
	void SetCurrent(size_t driver, float current);
	void EnableDrive(size_t driver, bool en);
	uint32_t GetLiveStatus(size_t driver);
	uint32_t GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep);
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation);
	bool SetDriverMode(size_t driver, unsigned int mode);
	DriverMode GetDriverMode(size_t driver);
	void SetStallThreshold(size_t driver, int sgThreshold);
	void SetStallFilter(size_t driver, bool sgFilter);
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond);
	void AppendStallConfig(size_t driver, const StringRef& reply);
	void AppendDriverStatus(size_t driver, const StringRef& reply);
	float GetStandstillCurrentPercent(size_t driver);
	void SetStandstillCurrentPercent(size_t driver, float percent);
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal);
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg);
};

#endif

#endif /* TMC2660_H_ */
