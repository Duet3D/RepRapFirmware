/*
 * TMC22xx.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

// NB - must #include some file that includes Pins.h before including this one, so that SUPPORT_TMC22xx is defined

#ifndef TMC22xx_H_
#define TMC22xx_H_

#if SUPPORT_TMC22xx

#ifndef TMC22xx_HAS_MUX
# error TMC22xx_HAS_MUX not defined
#endif

#include "RepRapFirmware.h"
#include "DriverMode.h"
#include "Pins.h"
#include "MessageType.h"

// TMC22xx DRV_STATUS register bit assignments
const uint32_t TMC_RR_OT = 1 << 1;			// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1 << 0;		// over temperature warning
const uint32_t TMC_RR_S2G = 15 << 2;		// short to ground counter (4 bits)
const uint32_t TMC_RR_OLA = 1 << 6;			// open load A
const uint32_t TMC_RR_OLB = 1 << 7;			// open load B
const uint32_t TMC_RR_STST = 1 << 31;		// standstill detected
const uint32_t TMC_RR_OPW_120 = 1 << 8;		// temperature threshold exceeded
const uint32_t TMC_RR_OPW_143 = 1 << 9;		// temperature threshold exceeded
const uint32_t TMC_RR_OPW_150 = 1 << 10;	// temperature threshold exceeded
const uint32_t TMC_RR_OPW_157 = 1 << 11;	// temperature threshold exceeded
const uint32_t TMC_RR_TEMPBITS = 15 << 8;	// all temperature threshold bits

namespace SmartDrivers
{
	void Init(const Pin[NumDirectDrivers], size_t numTmcDrivers)
		pre(numTmcDrivers <= NumDirectDrivers);
	void SetAxisNumber(size_t drive, uint32_t axisNumber);
	uint32_t GetAxisNumber(size_t drive);
	void SetCurrent(size_t drive, float current);
	void EnableDrive(size_t drive, bool en);
	uint32_t GetLiveStatus(size_t drive);
	uint32_t GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep);
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation);
	bool SetDriverMode(size_t driver, unsigned int mode);
	DriverMode GetDriverMode(size_t driver);
	void Spin(bool powered);
	void TurnDriversOff();
	void AppendDriverStatus(size_t drive, const StringRef& reply);
	float GetStandstillCurrentPercent(size_t drive);
	void SetStandstillCurrentPercent(size_t drive, float percent);
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal);
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg);
};

#endif

#endif /* TMC22xx_H_ */
