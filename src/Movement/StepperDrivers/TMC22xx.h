/*
 * TMC22xx.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef TMC22xx_H_
#define TMC22xx_H_

#include <RepRapFirmware.h>

#if SUPPORT_TMC22xx

#ifndef SUPPORT_TMC2240
# define SUPPORT_TMC2240	0
#endif

#ifndef SUPPORT_TMC2209
# define SUPPORT_TMC2209	1
#endif

#ifndef SUPPORT_TMC2208
# define SUPPORT_TMC2208	0
#endif

#include "DriverMode.h"

namespace SmartDrivers
{
#if TMC22xx_VARIABLE_NUM_DRIVERS
	void Init(size_t numTmcDrivers) noexcept
	pre(numTmcDrivers <= MaxSmartDrivers);
#elif SUPPORT_TMC2240 && defined(DUET3MINI)
	void Init(bool hasTmc2240Expansion) noexcept;
#else
	void Init() noexcept;
#endif

	void Exit() noexcept;
	void SetAxisNumber(size_t drive, uint32_t axisNumber) noexcept;
	uint32_t GetAxisNumber(size_t drive) noexcept;
	void SetCurrent(size_t drive, float current) noexcept;
	void EnableDrive(size_t drive, bool en) noexcept;
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
	StandardDriverStatus GetStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept;
#if HAS_STALL_DETECT
	DriversBitmap GetStalledDrivers(DriversBitmap driversOfInterest) noexcept;
#endif
#if SUPPORT_TMC2240 && !(SUPPORT_TMC2208 || SUPPORT_TMC2209)
	float GetDriverTemperature(size_t driver) noexcept;
#endif
};

#endif

#endif /* TMC22xx_H_ */
