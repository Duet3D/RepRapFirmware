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
		uint32_t shapeAccelStart : 1,
				 shapeAccelEnd : 1,
				 shapeAccelOverlapped : 1,
				 shapeDecelStart : 1,
				 shapeDecelEnd : 1,
				 shapeDecelOverlapped : 1,
				 debugPrint : 1;
	};
	uint32_t all;

	InputShaperPlan() noexcept : all(0) { }

	void Clear() noexcept { all = 0; }

	bool IsShaped() const noexcept { return shapeAccelStart || shapeAccelEnd || shapeAccelOverlapped || shapeDecelStart || shapeDecelEnd || shapeDecelOverlapped; }
};


#endif /* SRC_MOVEMENT_INPUTSHAPERPLAN_H_ */
