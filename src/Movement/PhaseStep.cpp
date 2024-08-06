/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

/*
 * Some observations on closed loop control of motors:
 * 1. We can generate steps in a step ISR as we do in open loop ode, then add each microstep to the desired position.
 *    But it is probably better to calculate the position directly from the move parameters.
 *    We can calculate the current velocity and acceleration as well, which should enable more accurate positioning.
 * 2. Currently we allow the user to set microstepping as usual, because that works in both open and closed loop mode.
 *    If we required that the users uses microstepping equal to the encoder resolution, then it would be difficult to operate in open loop mode.
 * 3. We can write the control loop to aim for a particular encoder reading (by first converting the required step position to an encoder reading),
 *    or we can convert encoder readings to step positions and aim for a particular step position. If we do the latter then the control loop
 *    will typically hunt between two encoder positions that straddle the required step position. We could perhaps avoid that by treating
 *    a motor position error that is less than about 0.6 encoder steps as zero error. However, targeting encoder count has the advantage that
 *    we could use mostly integer maths, which would be better if we want to implement this on SAMC21 or RP2040 platforms.
 * 3. We need to be sure that we can count encoder steps and motor positions without loss of accuracy when they get high.
 *    The encoder with greatest resolution is currently the AS5047, which has 16384 counts/rev = 81.92 counts/full step. TLI5012B has the same resolution.
 *    If the machine has 100 microsteps/mm at x16 microstepping then this corresponds to 512 counts/mm. If stored as a 32-bit signed integer
 *    it will overflow at about +/-4.2km which should be OK for axes but perhaps not for extruders.
 * 4. If we represent motor step position as a float then to be accurate to the nearest encoder count we have only 24 bits available,
 *    so the highest we can go without loss of resolution is +/- 32.768 metres.
 * 5. Since all motion commands sent to the board are relative, we could reset the step count and encoder count periodically by a whole number of revolutions,
 *    as long as we are careful to do it atomically. If/when we support linear encoders, this will not be necessary for those.
 * 6. When using an absolute encoder, although it could be treated like a relative encoder for the purposes of motor control,
 *    the angle information may be useful to compensate for leadscrew nut irregularities; so we should preserve the angle information.
 */

#include "PhaseStep.h"

#if SUPPORT_PHASE_STEPPING

using std::atomic;
using std::numeric_limits;

# include <math.h>
# include <Platform/Platform.h>
# include <Movement/Move.h>
# include <General/Bitmap.h>
# include <Platform/TaskPriorities.h>
# include <AppNotifyIndices.h>

# if SUPPORT_TMC51xx
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support phase stepping with the specified hardware
# endif

#define BASIC_TUNING_DEBUG	0

static uint16_t phaseOffset[MaxSmartDrivers] = { 0 };			// The amount by which the phase should be offset for each driver

const char* TranslateStepMode(const StepMode mode)
{
	switch (mode)
	{
	case StepMode::stepDir:
		return "step and direction";
	case StepMode::phase:
		return "phase stepping";
	default:
		return "unknown";
	}
}

// Helper function to reset the 'monitoring variables' as defined above
void PhaseStep::ResetMonitoringVariables() noexcept
{
	PhaseStep::minControlLoopRuntime = numeric_limits<StepTimer::Ticks>::max();
	PhaseStep::maxControlLoopRuntime = 1;
	PhaseStep::minControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::max();
	PhaseStep::maxControlLoopCallInterval = 1;
}

// Set the motor currents and update desiredStepPhase
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192.
// We must take it modulo 4096 when computing the currents. Function Trigonometry::FastSinCos does that.
// 'magnitude' must be in range 0.0..1.0
void PhaseStep::SetMotorPhase(size_t driver, uint16_t phase, float magnitude) noexcept
{
	desiredStepPhase = phase;
	float sine, cosine;
	Trigonometry::FastSinCos(phase, sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);
	SmartDrivers::SetMotorCurrents(driver, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
}

// Update the standstill current fraction for this drive.
void PhaseStep::UpdateStandstillCurrent() noexcept
{
	// TODO
#if 0
# error Multi driver code not implemented
	holdCurrentFraction = SmartDrivers::GetStandstillCurrentPercent(0) * 0.01;
#endif
}

void PhaseStep::InstanceControlLoop(size_t driver) noexcept
{
	if (!reprap.GetMove().IsPhaseSteppingEnabled())
	{
		return;
	}

	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	const StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
	prevControlLoopCallTime = loopCallTime;
	minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
	maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);

	const float currentFraction = CalculateMotorCurrents(driver, loopCallTime - StepTimer::GetMovementDelay());

	// Update the statistics
	if (currentFraction > periodMaxCurrentFraction)
	{
		periodMaxCurrentFraction = currentFraction;
	}
	periodSumOfCurrentFractions += currentFraction;
	++periodNumSamples;

	// Record how long this has taken to run
	const StepTimer::Ticks loopRuntime = StepTimer::GetTimerTicks() - loopCallTime;
	minControlLoopRuntime = min<StepTimer::Ticks>(minControlLoopRuntime, loopRuntime);
	maxControlLoopRuntime = max<StepTimer::Ticks>(maxControlLoopRuntime, loopRuntime);
}

bool PhaseStep::IsEnabled() const noexcept
{
	return reprap.GetMove().IsPhaseSteppingEnabled();
}

void PhaseStep::UpdatePhaseOffset(size_t driver) noexcept
{
	AtomicCriticalSectionLocker lock;
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	uint16_t calculatedStepPhase = CalculateStepPhase(driver, loopCallTime - StepTimer::GetMovementDelay());
	phaseOffset[driver] = (desiredStepPhase - (calculatedStepPhase - phaseOffset[driver])) % 4096u;
}

inline uint16_t PhaseStep::CalculateStepPhase(size_t driver, uint32_t when) noexcept
{
	reprap.GetMove().GetCurrentMotion(driver, when, mParams);
	const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);		// we use llrintf so that we can guarantee to convert the float operand to integer. We only care about the lowest 12 bits.
	return (stepPhase + phaseOffset[driver]) % 4096u;
}

// Control the motor phase currents, returning the fraction of maximum current that we commanded
inline float PhaseStep::CalculateMotorCurrents(size_t driver, uint32_t when) noexcept
{
	// Driver is in assisted open loop mode
	// In this mode the PID terms are not used and the A and V terms are independent of the loop time.
	const uint16_t commandedStepPhase = CalculateStepPhase(driver, when);			// do this first because it sets up mparams
	constexpr float scalingFactor = 100.0;
	PIDVTerm = mParams.speed * Kv * scalingFactor;
	PIDATerm = mParams.acceleration * Ka * fsquare(scalingFactor);
	PIDControlSignal = min<float>(fabsf(PIDVTerm) + fabsf(PIDATerm), 256.0);

	const float currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(PIDControlSignal * (1.0/256.0), 1.0);
	SetMotorPhase(driver, commandedStepPhase, currentFraction);
	return currentFraction;
}

#endif

// End
