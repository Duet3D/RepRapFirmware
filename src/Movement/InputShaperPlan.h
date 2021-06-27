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
				 shapeDecelStart : 1,
				 shapeDecelEnd : 1,
				 accelSegments : 4,
				 decelSegments : 4;
	};
	uint32_t all;

	InputShaperPlan() noexcept : all(0) { }

	void SetNoShaping() noexcept { all = 0; }
};


#endif /* SRC_MOVEMENT_INPUTSHAPERPLAN_H_ */
