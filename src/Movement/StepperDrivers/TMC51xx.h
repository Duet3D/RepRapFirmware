/*
 * TMC51xx.h
 *
 *  Created on: 26 Aug 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_TMC51XX_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_TMC51XX_H_

#include "RepRapFirmware.h"

#if SUPPORT_TMC51xx

#include "DriverMode.h"

// TMC51xx DRV_STATUS register bit assignments
const uint32_t TMC_RR_SG = 1 << 24;					// stall detected
const uint32_t TMC_RR_OT = 1 << 25;					// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1 << 26;				// over temperature warning
const uint32_t TMC_RR_S2G = (3 << 27) | (3 << 12);	// short to ground indicator (1 bit for each phase) + short to VS indicator
const uint32_t TMC_RR_OLA = 1 << 29;				// open load A
const uint32_t TMC_RR_OLB = 1 << 30;				// open load B
const uint32_t TMC_RR_STST = 1 << 31;				// standstill detected
const uint32_t TMC_RR_SGRESULT = 0x3FF;				// 10-bit stallGuard2 result

namespace SmartDrivers
{
	void Init() noexcept;
	void Exit() noexcept;
	void Spin(bool powered) noexcept;
	void TurnDriversOff() noexcept;

	void SetAxisNumber(size_t driver, uint32_t axisNumber) noexcept;
	uint32_t GetAxisNumber(size_t drive) noexcept;
	void SetCurrent(size_t driver, float current) noexcept;
	void EnableDrive(size_t driver, bool en) noexcept;
	uint32_t GetLiveStatus(size_t driver) noexcept;
	uint32_t GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep) noexcept;
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation) noexcept;
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) noexcept;
	bool SetDriverMode(size_t driver, unsigned int mode) noexcept;
	DriverMode GetDriverMode(size_t driver) noexcept;
	void SetStallThreshold(size_t driver, int sgThreshold) noexcept;
	void SetStallFilter(size_t driver, bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(size_t driver, const StringRef& reply) noexcept;
	void AppendDriverStatus(size_t driver, const StringRef& reply) noexcept;
	float GetStandstillCurrentPercent(size_t driver) noexcept;
	void SetStandstillCurrentPercent(size_t driver, float percent) noexcept;
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg) noexcept;
	GCodeResult GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;
};

#endif

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_TMC51XX_H_ */
