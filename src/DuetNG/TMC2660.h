/*
 * ExternalDrivers.h
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifndef TMC2660_H_
#define TMC2660_H_

#include "RepRapFirmware.h"
#include "GCodes/DriverMode.h"
#include "Pins.h"
#include "MessageType.h"
#include "Libraries/General/StringRef.h"

// The Platform class needs to know which USART we are using when assigning interrupt priorities
#define USART_TMC_DRV			USART1
#define SERIAL_TMC_DRV_IRQn		USART1_IRQn
#define ID_USART_TMC_DRV		ID_USART1
#define USART_TMC_DRV_Handler	USART1_Handler

// TMC2660 Read response. The microstep counter can also be read, but we don't include that here.
const uint32_t TMC_RR_SG = 1 << 0;			// stall detected
const uint32_t TMC_RR_OT = 1 << 1;			// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1 << 2;		// over temperature warning
const uint32_t TMC_RR_S2G = 3 << 3;			// short to ground counter (2 bits)
const uint32_t TMC_RR_OLA = 1 << 5;			// open load A
const uint32_t TMC_RR_OLB = 1 << 6;			// open load B
const uint32_t TMC_RR_STST = 1 << 7;		// standstill detected
const uint32_t TMC_RR_SG_LOAD_SHIFT = 10;	// shift to get stallguard load register

namespace SmartDrivers
{
	void Init(const Pin[DRIVES], size_t numTmcDrivers)
		pre(numTmcDrivers <= DRIVES);
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
	bool SetChopperControlRegister(size_t driver, uint32_t ccr);
	uint32_t GetChopperControlRegister(size_t driver);
	void SetStallThreshold(size_t driver, int sgThreshold);
	void SetStallFilter(size_t driver, bool sgFilter);
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond);
	void SetCoolStep(size_t driver, uint16_t coolStepConfig);
	void AppendStallConfig(size_t driver, const StringRef& reply);
	void AppendDriverStatus(size_t driver, const StringRef& reply);
	float GetStandstillCurrentPercent(size_t driver);
	void SetStandstillCurrentPercent(size_t driver, float percent);
};

#endif /* TMC2660_H_ */
