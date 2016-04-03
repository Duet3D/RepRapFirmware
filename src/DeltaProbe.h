/*
 * DeltaProbe.h
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#ifndef DELTAPROBE_H_
#define DELTAPROBE_H_

// Class to hold the parameters for my new Z probing method
class DeltaProbe
{
	enum class State { normal, stopping, stopped, overran };

	// Fixed parameters
	static const unsigned int MaxSteps = 30;		// 15 corresponds to 0.375mm p-p movement @ 80 steps/mm

	// Static parameters, set up before we start probing and unchanged during probing
	unsigned int normalSteps;							// the number of steps we use to achieve the requested amplitude
	unsigned int halfCyclesPerIncrement;				// how many half cycles between lowering the head by 1 step
	unsigned int maxIncrements;							// max number of steps we lower the head
	uint32_t halfCycleTime;								// how many interrupt clocks per quarter cycle
	uint32_t normalStepTable[MaxSteps];					// table of step times for the first half cycle, in interrupt clocks from start
	uint32_t incStepTable[MaxSteps + 1];				// table of step times for the first half cycle, when we are moving down a step

	// Dynamic parameters, to track the progress of the probe
	unsigned int stepsDone;								// how many steps since the start of this quarter cycle
	unsigned int halfCycleCount;						// how many quarter cycles since we started or lowered the head
	unsigned int numIncrements;							// how many steps we have lowered the head since we started
	bool incrementing;									// true if we are lowering the head 2 step in this half cycle
	State state;										// what state the probe is in

public:
	bool Init(float frequency, float amplitude, float rate, float height);	// Get ready to probe
	uint32_t Start();									// start the process, return the next step time
	bool GetDirection() const;							// get the direction for the current step
	uint32_t CalcNextStepTime();						// calculate when the next step is due
	void Trigger();										// cease probing
	bool Finished() const { return state == State::stopped || state == State::overran; }
	bool Overran() const { return state == State::overran; }
};

#endif /* DELTAPROBE_H_ */
