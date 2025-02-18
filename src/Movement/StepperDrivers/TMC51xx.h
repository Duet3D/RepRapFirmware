/*
 * TMC51xx.h
 *
 *  Created on: 26 Aug 2018
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_TMC51XX_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_TMC51XX_H_

#include <RepRapFirmware.h>

#if SUPPORT_TMC51xx

#include "DriverMode.h"
#include <Endstops/EndstopDefs.h>
#include <atomic>

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
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation) noexcept;
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) noexcept;

#if SUPPORT_PHASE_STEPPING
	bool EnablePhaseStepping(size_t driver, bool enable) noexcept;
	bool IsPhaseSteppingEnabled(size_t driver) noexcept;
	float GetCurrent(size_t driver) noexcept;
	unsigned int GetMicrostepShift(size_t driver) noexcept;
	uint16_t GetMicrostepPosition(size_t driver) noexcept;
	void SetTmcExternalClock(uint32_t frequency) noexcept;
	bool SetMotorPhases(size_t driver, uint32_t regVal) noexcept;
#endif

	bool SetDriverMode(size_t driver, unsigned int mode) noexcept;
	DriverMode GetDriverMode(size_t driver) noexcept;
	void SetStallThreshold(size_t driver, int sgThreshold) noexcept;
	void SetStallFilter(size_t driver, bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(size_t driver, const StringRef& reply) noexcept;
	void AppendDriverStatus(size_t driver, const StringRef& reply) noexcept;
	float GetStandstillCurrentPercent(size_t driver) noexcept;
	void SetStandstillCurrentPercent(size_t driver, float percent) noexcept;
	bool SetCurrentScaler(size_t driver, int8_t cs) noexcept;
	uint8_t GetIRun(size_t driver) noexcept;
	uint8_t GetIHold(size_t driver) noexcept;
	uint32_t GetGlobalScaler(size_t driver) noexcept;
	float GetCalculatedCurrent(size_t driver) noexcept;
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg) noexcept;
	GCodeResult GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;
	StandardDriverStatus GetStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept;
	const char *_ecv_array _ecv_null CheckStallDetectionEnabled(size_t driver, float speed) noexcept;

#if SUPPORT_REMOTE_COMMANDS
	GCodeResult SetStallEndstopReporting(uint16_t driverNumber, float speed, const StringRef& reply) noexcept;
	extern std::atomic<uint16_t> driverStallsToNotify;
#endif
}

#endif

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_TMC51XX_H_ */
