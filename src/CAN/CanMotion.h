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

namespace CanMotion
{
	void Init() noexcept;
	void StartMovement() noexcept;
	void AddMovement(const PrepParams& params, DriverId canDriver, int32_t steps, bool usePressureAdvance) noexcept;
	uint32_t FinishMovement(uint32_t moveStartTime) noexcept;
	bool CanPrepareMove() noexcept;
	CanMessageBuffer *GetUrgentMessage() noexcept;

	// The next 4 functions may be called from the step ISR, so they can't send CAN messages directly
	void InsertHiccup(uint32_t numClocks) noexcept;
	void StopDriver(bool isBeingPrepared, DriverId driver) noexcept;
	void StopAxis(bool isBeingPrepared, size_t axis) noexcept;
	void StopAll(bool isBeingPrepared) noexcept;
}

#endif

#endif /* SRC_CAN_CANMOTION_H_ */
