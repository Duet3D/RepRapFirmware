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
#if USE_REMOTE_INPUT_SHAPING
	void AddMovement(const PrepParams& params, DriverId canDriver, int32_t steps) noexcept;
	void AddExtruderMovement(const PrepParams& params, DriverId canDriver, float extrusion, bool usePressureAdvance) noexcept;
#else
	void AddMovement(const PrepParams& params, DriverId canDriver, int32_t steps, bool usePressureAdvance) noexcept;
#endif
	uint32_t FinishMovement(const DDA& dda, uint32_t moveStartTime, bool simulating) noexcept;
	bool CanPrepareMove() noexcept;
	CanMessageBuffer *GetUrgentMessage() noexcept;

	// The next 4 functions may be called from the step ISR, so they can't send CAN messages directly
	void InsertHiccup(uint32_t numClocks) noexcept;
	void StopAll(const DDA& dda) noexcept;
	void StopAxis(const DDA& dda, size_t axis) noexcept;
	void StopDriver(const DDA& dda, size_t axis, DriverId driver) noexcept
		pre(driver.IsRemote());
	void FinishMoveUsingEndstops() noexcept;
	bool FinishedReverting() noexcept;
}

#endif

#endif /* SRC_CAN_CANMOTION_H_ */
