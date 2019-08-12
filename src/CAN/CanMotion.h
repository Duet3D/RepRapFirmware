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
	void Init();
	void StartMovement(const DDA& dda);
	void AddMovement(const DDA& dda, const PrepParams& params, DriverId canDriver, int32_t steps);
	void FinishMovement(uint32_t moveStartTime);
	bool CanPrepareMove();
	void InsertHiccup(uint32_t numClocks);
}

#endif

#endif /* SRC_CAN_CANMOTION_H_ */
