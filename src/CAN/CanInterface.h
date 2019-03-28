/*
 * CanInterface.h
 *
 *  Created on: 19 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"

#if SUPPORT_CAN_EXPANSION

class DDA;
class DriveMovement;
struct PrepParams;

namespace CanInterface
{
	void Init();
	void StartMovement(const DDA& dda);
	void AddMovement(const DDA& dda, const PrepParams& params, size_t canDriver, int32_t steps);
	void FinishMovement(uint32_t moveStartTime);
	bool CanPrepareMove();
	void InsertHiccup(uint32_t numClocks);
}

#endif

#endif /* SRC_CAN_CANINTERFACE_H_ */
