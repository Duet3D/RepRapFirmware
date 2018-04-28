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

// The Platform class needs to know which USART we are using when assigning interrupt priorities (SERIAL_TMC_DRV_IRQn), so we define the UART we are using here
#define UART_TMC_DRV			UART0
#define SERIAL_TMC_DRV_IRQn		UART0_IRQn
#define ID_UART_TMC_DRV			ID_UART0
#define UART_TMC_DRV_Handler	UART0_Handler

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
	void Init(const Pin[DRIVES], size_t numTmcDrivers)
		pre(numTmcDrivers <= DRIVES);
	void SetAxisNumber(size_t drive, uint32_t axisNumber);
	void SetCurrent(size_t drive, float current);
	void EnableDrive(size_t drive, bool en);
	uint32_t GetLiveStatus(size_t drive);
	uint32_t GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep);
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation);
	unsigned int GetMicrostepping(size_t drive, bool& interpolation);
	bool SetDriverMode(size_t driver, unsigned int mode);
	DriverMode GetDriverMode(size_t driver);
	bool SetChopperControlRegister(size_t driver, uint32_t ccr);
	uint32_t GetChopperControlRegister(size_t driver);
	void Spin(bool powered);
	void TurnDriversOff();
	void SetCoolStep(size_t drive, uint16_t coolStepConfig);
	void AppendDriverStatus(size_t drive, const StringRef& reply);
	float GetStandstillCurrentPercent(size_t drive);
	void SetStandstillCurrentPercent(size_t drive, float percent);
};

#endif /* TMC2660_H_ */
