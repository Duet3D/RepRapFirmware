/*
 * DeltaProbe.cpp
 *
 *  Created on: 20 Apr 2015
 *      Author: David
 */

#include "RepRapFirmware.h"

// Set up to probe
bool DeltaProbe::Init(float frequency, float amplitude, float rate, float height)
{
debugPrintf("Start probe f=%.1f a=%.2f r=%.2f h=%.1f\n", frequency, amplitude, rate, height);
	// Sanity check the inputs (we check the max amplitude later)
	if (frequency < 50.0 || frequency > 1000.0 || amplitude < 0.02 || rate < 0.1 || rate > 10.0 || height < 0.5)
	{
		return false;
	}

debugPrintf("ok so far\n");
	// Calculate the number of steps for the peak to peak amplitude
	const float zRate = reprap.GetPlatform()->DriveStepsPerUnit(Z_AXIS);
	normalSteps = (size_t)(amplitude * zRate);
	if (normalSteps > MaxSteps)
	{
		return false;
	}

debugPrintf("normalSteps=%u\n", normalSteps);
	// Build the tables of step times for sinusoidal motion
	const float recipOmega = (float)DDA::stepClockRate/(frequency * 2.0 * PI);

	for (size_t i = 0; i < normalSteps - 1; ++i)
	{
		normalStepTable[i] = acos(1.0 - (float)(2 * (i + 1))/(float)normalSteps) * recipOmega;
	}

	for (size_t i = 0; i < normalSteps; ++i)
	{
		incStepTable[i] = acos(1.0 - (float)(2 * (i + 1))/(float)(normalSteps + 1)) * recipOmega;
	}

	halfCycleTime = (uint32_t)((float)DDA::stepClockRate/(2.0 * frequency));
	incStepTable[normalSteps] = normalStepTable[normalSteps - 1] = halfCycleTime;

	halfCyclesPerIncrement = 2 * (unsigned int)((frequency / (rate * zRate)) + 0.5);
	if (halfCyclesPerIncrement < 4)
	{
		halfCyclesPerIncrement = 4;
	}
	maxIncrements = height * zRate;

const float peakAccel = fsquare(2.0 * PI * frequency) * amplitude * 0.5;
debugPrintf("halfCycleTime=%u halfCyclesPerIncrement=%u peak accel=%.1f\n", halfCycleTime, halfCyclesPerIncrement, peakAccel);
debugPrintf("normalTable=");
for (unsigned int i = 0; i < normalSteps; ++i)
{
	debugPrintf(" %u", normalStepTable[i]);
}
debugPrintf(" incStepTable=");
for (unsigned int i = 0; i <= normalSteps; ++i)
{
	debugPrintf(" %u", incStepTable[i]);
}
debugPrintf("\n");
	return true;
}

// Start probing, and return the time that the next step is due
uint32_t DeltaProbe::Start()
{
	// Initialise the dynamic values
	stepsDone = 0;
	halfCycleCount = 0;
	numIncrements = 0;
	incrementing = false;
	state = State::normal;
	return normalStepTable[0];
}

bool DeltaProbe::GetDirection() const
{
	return (halfCycleCount & 1) ? FORWARDS : BACKWARDS;
}

// Calculate the next step time. Returns 0xFFFFFFFF to stop.
uint32_t DeltaProbe::CalcNextStepTime()
{
	if (state == State::stopped || state == State::overran)
	{
		return 0xFFFFFFFF;
	}

	++stepsDone;
	if (stepsDone == ((incrementing) ? normalSteps + 1 : normalSteps))
	{
		stepsDone = 0;
		++halfCycleCount;
		if (state == State::stopping && (halfCycleCount & 1) == 0)
		{
			state = State::stopped;
			return 0xFFFFFFFF;
		}

		if (incrementing)
		{
			++numIncrements;
			incrementing = false;
		}

		if (halfCycleCount == halfCyclesPerIncrement)
		{
			if (numIncrements == maxIncrements)
			{
				state = State::overran;		// another increment is due, but we have already gone down as far as we were asked to
				return 0xFFFFFFFF;
			}
			halfCycleCount = 0;
			incrementing = true;
		}
	}

	return (incrementing)
			? (halfCyclesPerIncrement * numIncrements * halfCycleTime) + incStepTable[stepsDone]
			: (halfCyclesPerIncrement * numIncrements * halfCycleTime) + normalStepTable[stepsDone];
}

void DeltaProbe::Trigger()
{
	if (state == State::normal)
	{
		state = State::stopping;
	}
}

// End
