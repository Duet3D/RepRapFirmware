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
# include <Movement/MoveDebugFlags.h>
# include <General/Bitmap.h>
# include <Platform/TaskPriorities.h>
# include <AppNotifyIndices.h>

# if SUPPORT_TMC51xx
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support phase stepping with the specified hardware
# endif

#define BASIC_TUNING_DEBUG	0

static uint16_t currentPhase[MaxSmartDrivers] = { 0 };
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

// Set the motor currents and update desiredStepPhase
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192.
// We must take it modulo 4096 when computing the currents. Function Trigonometry::FastSinCos does that.
// 'magnitude' must be in range 0.0..1.0
void PhaseStep::SetMotorPhase(size_t driver, uint16_t phase, float magnitude) noexcept
{
	currentPhase[driver] = phase;
	float sine, cosine;
	Trigonometry::FastSinCos(phase, sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);
	SmartDrivers::SetMotorPhases(driver, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
//	debugPrintf("Set driver %" PRIu16 " phase to %" PRIu16 " %.2f%%, v=%.5f, a=%.5f %lu\n", driver, phase, (double)magnitude * 100, (double)mParams.speed, (double)mParams.acceleration, StepTimer::GetTimerTicks());
}

// Set the standstill current fraction for this drive.
void PhaseStep::SetStandstillCurrent(float percent) noexcept
{
	holdCurrentFraction = percent * 0.01;
}

void PhaseStep::InstanceControlLoop(size_t driver) noexcept
{
	if (unlikely(!enabled))
	{
		return;
	}

	const uint16_t commandedStepPhase = CalculateStepPhase(driver);
	SetMotorPhase(driver, commandedStepPhase, currentFraction);
}

void PhaseStep::UpdatePhaseOffset(size_t driver) noexcept
{
	AtomicCriticalSectionLocker lock;
	const uint16_t calculatedStepPhase = CalculateStepPhase(driver);
//	const uint16_t oldOffset = phaseOffset[driver];
	phaseOffset[driver] = (currentPhase[driver] - (calculatedStepPhase - phaseOffset[driver])) % 4096u;
//	debugPrintf("Updated phaseOffset[%u]=%u, desired=%u, calculated=%u, oldOffset=%u\n", driver, phaseOffset[driver], currentPhase[driver], calculatedStepPhase, oldOffset);
}

void PhaseStep::SetPhaseOffset(size_t driver, uint16_t offset) noexcept
{
	AtomicCriticalSectionLocker lock;
	phaseOffset[driver] = (offset) % 4096u;
//	debugPrintf("Set phaseOffset[%u]=%u\n", driver, phaseOffset[driver]);
}

uint16_t PhaseStep::GetPhaseOffset(size_t driver)
{
	return phaseOffset[driver];
}

uint16_t PhaseStep::CalculateStepPhase(size_t driver) noexcept
{
	const float multiplier = reprap.GetMove().GetDirectionValue(driver) ? 1.0 : -1.0;
	const uint16_t calculatedStepPhase = (uint16_t)llrintf(mParams.position * multiplier * 1024.0);		// we use llrintf so that we can guarantee to convert the float operand to integer. We only care about the lowest 12 bits.
	return (calculatedStepPhase + phaseOffset[driver]) % 4096u;
}

// Control the motor phase currents, returning the fraction of maximum current that we commanded
float PhaseStep::CalculateCurrentFraction() noexcept
{
	// Driver is in assisted open loop mode
	// In this mode the PID terms are not used and the A and V terms are independent of the loop time.
	constexpr float scalingFactor = 100.0;
	constexpr float scalingFactorSqr = scalingFactor * scalingFactor;
	PIDVTerm = mParams.speed * Kv * scalingFactor;
	PIDATerm = mParams.acceleration * Ka * scalingFactorSqr;
	PIDControlSignal = min<float>(fabsf(PIDVTerm) + fabsf(PIDATerm), 256.0);

	currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(PIDControlSignal * (1.0/256.0), 1.0);
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PhaseStep))
	{
		debugPrintf("v=%f, a=%f, cf=%f\n", (double)(mParams.speed * scalingFactor), (double)(mParams.acceleration * scalingFactorSqr), (double)currentFraction);
	}
	return currentFraction;
}

#endif

// End
