/*
 * CanMotion.h
 *
 *  Created on: 11 Aug 2019
 *      Author: David
 */

#ifndef SRC_CAN_CANMOTION_H_
#define SRC_CAN_CANMOTION_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

#include <Movement/DDA.h>

class CanMessageBuffer;

namespace CanMotion
{
	void Init() noexcept;
	void StartMovement() noexcept;
	void AddLinearAxisMovement(const PrepParams& params, DriverId canDriver, int32_t steps) noexcept;
	void AddExtruderMovement(const PrepParams& params, DriverId canDriver, float extrusion, bool usePressureAdvance) noexcept;
	uint32_t FinishMovement(const DDA& dda, uint32_t moveStartTime, bool simulating) noexcept;
	bool CanPrepareMove() noexcept;
	CanMessageBuffer *GetUrgentMessage() noexcept;

	// The next 4 functions may be called from the step ISR, so they can't send CAN messages directly
	void InsertHiccup(uint32_t numClocks) noexcept;
	void StopDriverWhenProvisional(DriverId driver) noexcept
		pre(driver.IsRemote());
	bool StopDriverWhenExecuting(DriverId driver, int32_t netStepsTaken) noexcept
		pre(driver.IsRemote());
	void FinishedStoppingDrivers() noexcept;
	bool RevertStoppedDrivers() noexcept;
}

#endif

#endif /* SRC_CAN_CANMOTION_H_ */
