/*
 * InputShaperPlan.h
 *
 *  Created on: 7 Jun 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_INPUTSHAPERPLAN_H_
#define SRC_MOVEMENT_INPUTSHAPERPLAN_H_

#include <cstdint>

union InputShaperPlan
{
	struct
	{
		uint32_t
				 // The first 8 bits are sent in CAN messages, so don't change them
				 shapeAccelStart : 1,
				 shapeAccelEnd : 1,
				 shapeAccelOverlapped : 1,
				 shapeDecelStart : 1,
				 shapeDecelEnd : 1,
				 shapeDecelOverlapped : 1,
				 spare1 : 2,

				 // The remaining bits are only used locally
				 debugPrint : 1,
				 spare : 23;
	};
	struct
	{
		uint32_t condensedPlan : 8,
				 spare3 : 24;
	};
	uint32_t all;

	InputShaperPlan() noexcept : all(0) { }

	void Clear() noexcept { all = 0; }

	bool IsShaped() const noexcept { return condensedPlan != 0; }
};


#endif /* SRC_MOVEMENT_INPUTSHAPERPLAN_H_ */
