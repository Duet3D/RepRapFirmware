/*
 * SimulationMode.h
 *
 *  Created on: 12 Dec 2023
 *      Author: David
 */

#ifndef SRC_GCODES_SIMULATIONMODE_H_
#define SRC_GCODES_SIMULATIONMODE_H_

#include <cstdint>

enum class SimulationMode : uint8_t
{	off = 0,				// not simulating
	debug,					// simulating step generation
	normal,					// not generating steps, just timing
	partial,				// generating DDAs but doing nothing with them
	highest = partial
};

#endif /* SRC_GCODES_SIMULATIONMODE_H_ */
