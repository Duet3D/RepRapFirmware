/*
 * TMC22xx.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef TMC22xx_H_
#define TMC22xx_H_

#include "RepRapFirmware.h"

#if SUPPORT_TMC22xx

#include "DriverMode.h"

// TMC22xx DRV_STATUS register bit assignments
const uint32_t TMC_RR_OT = 1u << 1;			// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1u << 0;		// over temperature warning
const uint32_t TMC_RR_S2G = 15u << 2;		// short to ground counter (4 bits)
const uint32_t TMC_RR_OLA = 1u << 6;		// open load A
const uint32_t TMC_RR_OLB = 1u << 7;		// open load B
const uint32_t TMC_RR_STST = 1u << 31;		// standstill detected
const uint32_t TMC_RR_OPW_120 = 1u << 8;	// temperature threshold exceeded
const uint32_t TMC_RR_OPW_143 = 1u << 9;	// temperature threshold exceeded
const uint32_t TMC_RR_OPW_150 = 1u << 10;	// temperature threshold exceeded
const uint32_t TMC_RR_OPW_157 = 1u << 11;	// temperature threshold exceeded
const uint32_t TMC_RR_TEMPBITS = 15u << 8;	// all temperature threshold bits

const uint32_t TMC_RR_RESERVED = (15u << 12) | (0x01FF << 21);	// reserved bits
const uint32_t TMC_RR_SG = 1u << 12;		// this is a reserved bit, which we use to signal a stall

namespace SmartDrivers
{
#if TMC22xx_VARIABLE_NUM_DRIVERS
	void Init(size_t numTmcDrivers) noexcept
	pre(numTmcDrivers <= MaxSmartDrivers);
#else
	void Init() noexcept;
#endif

	void Exit() noexcept;
	void SetAxisNumber(size_t drive, uint32_t axisNumber) noexcept;
	uint32_t GetAxisNumber(size_t drive) noexcept;
	void SetCurrent(size_t drive, float current) noexcept;
	void EnableDrive(size_t drive, bool en) noexcept;
	uint32_t GetLiveStatus(size_t drive) noexcept;
	uint32_t GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep) noexcept;
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation) noexcept;
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) noexcept;
	bool SetDriverMode(size_t driver, unsigned int mode) noexcept;
	DriverMode GetDriverMode(size_t driver) noexcept;
	void Spin(bool powered) noexcept;
	void TurnDriversOff() noexcept;
	void SetStallThreshold(size_t driver, int sgThreshold) noexcept;
	void SetStallFilter(size_t driver, bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(size_t driver, const StringRef& reply) noexcept;
	void AppendDriverStatus(size_t drive, const StringRef& reply) noexcept;
	float GetStandstillCurrentPercent(size_t drive) noexcept;
	void SetStandstillCurrentPercent(size_t drive, float percent) noexcept;
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg) noexcept;
	GCodeResult GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;

#if HAS_STALL_DETECT
	DriversBitmap GetStalledDrivers(DriversBitmap driversOfInterest) noexcept;
#endif
};

#endif

#endif /* TMC22xx_H_ */
