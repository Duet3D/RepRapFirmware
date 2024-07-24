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

#if USE_PHASE_STEPPING

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
void PhaseStep::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
	desiredStepPhase = phase;
	float sine, cosine;
	Trigonometry::FastSinCos(phase, sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);

# if SUPPORT_TMC51xx
#  error Multi driver code not implemented
	SmartDrivers::SetMotorCurrents(0, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
# endif
}

// Update the standstill current fraction for this drive.
void PhaseStep::UpdateStandstillCurrent() noexcept
{
#if SINGLE_DRIVER
	holdCurrentFraction = SmartDrivers::GetStandstillCurrentPercent(0) * 0.01;
#else
# error Multi driver code not implemented
#endif
}

void PhaseStep::InstanceControlLoop() noexcept
{
	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	const StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
	prevControlLoopCallTime = loopCallTime;
	minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
	maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);

	// Read the current state of the drive. Do this even if we are not in closed loop mode.
	if (encoder != nullptr && !encoder->TakeReading())
	{
		// Calculate and store the current error in full steps
		hasMovementCommand = moveInstance->GetCurrentMotion(0, loopCallTime - StepTimer::GetMovementDelay(), mParams);
		if (hasMovementCommand)
		{
			if (inTorqueMode)
			{
				ExitTorqueMode();
			}
			if (samplingMode == RecordingMode::OnNextMove)
			{
				dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
				samplingMode = RecordingMode::Immediate;
			}
		}

		const float targetEncoderReading = rintf(mParams.position * encoder->GetCountsPerStep());
		currentPositionError = (float)(targetEncoderReading - encoder->GetCurrentCount()) * encoder->GetStepsPerCount();
		errorDerivativeFilter.ProcessReading(currentPositionError, loopCallTime);
		speedFilter.ProcessReading(encoder->GetCurrentCount() * encoder->GetStepsPerCount(), loopCallTime);

		float currentFraction = 0.0;
		if (currentMode != StepMode::open)
		{
			if (tuning != 0)														// if we need to tune, do it
			{
				// Limit the rate at which we command tuning steps. We need to do signed comparison because initially, whenLastTuningStepTaken is in the future.
				const int32_t timeSinceLastTuningStep = (int32_t)(loopCallTime - whenLastTuningStepTaken);
				if (timeSinceLastTuningStep >= (int32_t)stepTicksPerTuningStep)
				{
					whenLastTuningStepTaken = loopCallTime;
					PerformTune();
				}
				else if (samplingMode == RecordingMode::OnNextMove && timeSinceLastTuningStep + (int32_t)DataCollectionIdleStepTicks >= 0)
				{
					dataCollectionStartTicks = whenNextSampleDue = loopCallTime;
					samplingMode = RecordingMode::Immediate;
				}
			}
			else if (tuningError == 0)
			{
				currentFraction = ControlMotorCurrents(timeElapsed);				// otherwise control those motor currents!
				if (inTorqueMode)
				{
					stall = preStall = false;
				}
				else
				{
					// Look for a stall or pre-stall
					const float positionErr = fabsf(currentPositionError);
					if (stall)
					{
						// Reset the stall flag when the position error falls to below half the tolerance, to avoid generating too many stall events
						//TODO do we need a minimum delay before resetting too?
						if (errorThresholds[1] <= 0 || positionErr < errorThresholds[1]/2)
						{
							stall = false;
						}
					}
					else
					{
						stall = errorThresholds[1] > 0 && positionErr > errorThresholds[1];
						if (stall)
						{
							Heat::NewDriverFault();
						}
						else
						{
							preStall = errorThresholds[0] > 0 && positionErr > errorThresholds[0];
						}
					}
				}
			}
		}

		// Collect a sample, if we need to
		if (samplingMode == RecordingMode::Immediate && (int32_t)(loopCallTime - whenNextSampleDue) >= 0)
		{
			// It's time to take a sample
			CollectSample();
			whenNextSampleDue += dataCollectionIntervalTicks;
		}

		// Update the statistics
		TaskCriticalSectionLocker lock;						// prevent a race with the Heat task that sends the statistics

		const float absPositionError = fabsf(currentPositionError);
		if (absPositionError > periodMaxAbsPositionError)
		{
			periodMaxAbsPositionError = absPositionError;
		}
		periodSumOfPositionErrorSquares += fsquare(currentPositionError);
		if (currentFraction > periodMaxCurrentFraction)
		{
			periodMaxCurrentFraction = currentFraction;
		}
		periodSumOfCurrentFractions += currentFraction;
		++periodNumSamples;

	}

	// Record how long this has taken to run
	const StepTimer::Ticks loopRuntime = StepTimer::GetTimerTicks() - loopCallTime;
	minControlLoopRuntime = min<StepTimer::Ticks>(minControlLoopRuntime, loopRuntime);
	maxControlLoopRuntime = max<StepTimer::Ticks>(maxControlLoopRuntime, loopRuntime);
}

// Control the motor phase currents, returning the fraction of maximum current that we commanded
inline float PhaseStep::ControlMotorCurrents(StepTimer::Ticks ticksSinceLastCall) noexcept
{
	uint16_t commandedStepPhase;
	float currentFraction;

	if (inTorqueMode)
	{
		const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
#if 1
		// Limit the velocity by limiting the rate of rotation of the field (we could reduce the current too)
		if (torqueModeDirection)		// reverse movement
		{
			commandedStepPhase = (uint16_t)(((3 * 1024u) + measuredStepPhase) % 4096u);
			if (torqueModeMaxSpeed > 0.0 && speedFilter.GetDerivative() <= 0.0)
			{
				const uint32_t maxPhaseDecrement = (uint32_t)(torqueModeMaxSpeed * (1024 * ticksSinceLastCall)) % 4096;
				if ((desiredStepPhase - commandedStepPhase) % 4096u > maxPhaseDecrement)
				{
					commandedStepPhase = (uint16_t)((desiredStepPhase - maxPhaseDecrement) % 4096);
				}
			}
		}
		else						// forward movement
		{
			commandedStepPhase = (uint16_t)((measuredStepPhase + 1024u) % 4096u);
			if (torqueModeMaxSpeed > 0.0 && speedFilter.GetDerivative() >= 0.0)
			{
				const uint32_t maxPhaseIncrement = (uint32_t)(torqueModeMaxSpeed * (1024 * ticksSinceLastCall)) % 4096;
				if ((commandedStepPhase - desiredStepPhase) % 4096u > maxPhaseIncrement)
				{
					commandedStepPhase = (uint16_t)((desiredStepPhase + maxPhaseIncrement) % 4096);
				}
			}
		}

		currentFraction = torqueModeCommandedCurrentFraction;
#else
		// Limit the velocity by reducing the current if we are going too fast
		commandedStepPhase = (uint16_t)((((torqueModeDirection) ? (3 * 1024) : 1024) + measuredStepPhase) % 4096u);
		// For now we use a crude form of proportional control; we may need to improve it later.
		// If the speed is lower than the limit, increase the torque unless it is already at the requested torque.
		// If the speed is too high then reduce the torque.
		// It's likely that we will need to add a derivative term to prevent the speed oscillating.
		if (torqueModeMaxSpeed > 0.0)
		{
			const float rawVelocity = speedFilter.GetDerivative();
			const float speed = (torqueModeDirection) ? -rawVelocity : rawVelocity;
			const float speedErrorFraction = (speed - torqueModeMaxSpeed)/torqueModeMaxSpeed;
			const float torqueFactor = constrain<float>(VelocityLimitGainFactor * (1.0 - speedErrorFraction), 0.0, 1.0);
			currentFraction = torqueModeCommandedCurrentFraction * torqueFactor;
		}
		else
		{
			currentFraction = torqueModeCommandedCurrentFraction;
		}
#endif
	}
	else
	{
		// Use a PID controller to calculate the required 'torque' - the control signal
		// We choose to use a PID control signal in the range -256 to +256. This is arbitrary.
		PIDPTerm = constrain<float>(Kp * currentPositionError, -256.0, 256.0);
		PIDDTerm = constrain<float>(Kd * errorDerivativeFilter.GetDerivative() * StepTimer::StepClockRate, -256.0, 256.0);	// constrain D so that we can graph it more sensibly after a sudden step input

		if (currentMode == StepMode::closed)
		{
			const float timeDelta = (float)ticksSinceLastCall * (1.0/(float)StepTimer::StepClockRate);						// get the time delta in seconds
			PIDITerm = constrain<float>(PIDITerm + Ki * currentPositionError * timeDelta, -PIDIlimit, PIDIlimit);			// constrain I to prevent it running away
			PIDVTerm = mParams.speed * Kv * ticksSinceLastCall;
			PIDATerm = mParams.acceleration * Ka * fsquare(ticksSinceLastCall);
			PIDControlSignal = constrain<float>(PIDPTerm + PIDITerm + PIDDTerm + PIDVTerm + PIDATerm, -256.0, 256.0);		// clamp the sum between +/- 256

			// Calculate the offset required to produce the torque in the correct direction
			// i.e. if we are moving in the positive direction, we must apply currents with a positive phase shift
			// The max abs value of phase shift we want is 1 full step i.e. 25%.
			// Given that PIDControlSignal is -256 .. 256 and phase is 0 .. 4095
			// and that 25% of 4096 = 1024, our max phase shift = 4 * PIDControlSignal

			// New algorithm: phase of motor current is always +/- 1 full step relative to current position, but motor current is adjusted according to the PID result
			// The following assumes that signed arithmetic is 2's complement
			const float PhaseFeedForwardFactor = 1000.0;
			const int16_t phaseFeedForward = lrintf(constrain<float>(speedFilter.GetDerivative() * ticksSinceLastCall * PhaseFeedForwardFactor, -256.0, 256.0));
			const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
			const uint16_t adjustedStepPhase = (uint16_t)((int16_t)measuredStepPhase + phaseFeedForward) % 4096u;
			commandedStepPhase = (((PIDControlSignal < 0.0) ? (3 * 1024) : 1024) + adjustedStepPhase) % 4096u;
			currentFraction = fabsf(PIDControlSignal) * (1.0/256.0);
		}
		else
		{
			// Driver is in assisted open loop mode
			// In this mode the I term is not used and the A and V terms are independent of the loop time.
			constexpr float scalingFactor = 100.0;
			PIDVTerm = mParams.speed * Kv * scalingFactor;
			PIDATerm = mParams.acceleration * Ka * fsquare(scalingFactor);
			PIDControlSignal = min<float>(fabsf(PIDPTerm + PIDDTerm) + fabsf(PIDVTerm) + fabsf(PIDATerm), 256.0);

			const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);		// we use llrintf so that we can guarantee to convert the float operand to integer. We only care about the lowest 12 bits.
			commandedStepPhase = (stepPhase + phaseOffset) % 4096u;
			currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(PIDControlSignal * (1.0/256.0), 1.0);
		}
	}
	SetMotorPhase(commandedStepPhase, currentFraction);
	return currentFraction;
}

const char *_ecv_array PhaseStep::GetModeText() const noexcept
{
	return (currentMode == StepMode::stepDir) ? "step direction"
			: (currentMode == StepMode::phase) ? "phase stepping"
				: "unknown";
}

// This is called before the driver mode is changed. Return true if success. Always succeeds if we are disabling closed loop.
bool PhaseStep::SetClosedLoopEnabled(StepMode mode, const StringRef &reply) noexcept
{
	// Trying to enable closed loop
	if (mode == StepMode::phase)
	{
		if (currentMode == StepMode::stepDir)
		{
			// Switching from open to closed loop mode, so set the motor phase to match the current microstep position
			delay(10);													// delay long enough for the TMC driver to have read the microstep counter since the end of the last movement
			const uint16_t initialStepPhase = SmartDrivers::GetMicrostepPosition(0) * 4;	// get the current coil A microstep position as 0..4095

			// Temporarily calibrate the encoder zero position
			// We assume that the motor is at the position given by its microstep counter. This may not be true e.g. if it has a brake that has not been disengaged.
			const bool err = encoder->TakeReading();
			if (err)
			{
				reply.copy("Error reading encoder");
				return false;
			}
			desiredStepPhase = initialStepPhase;						// set this to be picked up later in DriverSwitchedToClosedLoop
		}

		if (encoder->UsesBasicTuning() && (tuningError & TuningError::NeedsBasicTuning) != 0)
		{
			encoder->SetTuningBackwards(false);
			encoder->SetKnownPhaseAtCurrentCount(desiredStepPhase);
		}

		PIDITerm = 0.0;
		errorDerivativeFilter.Reset();
		speedFilter.Reset();
		SetTargetToCurrentPosition();

		// Set the target position to the current position
		ResetError();													// this calls ReadState again and sets up targetMotorSteps

		ResetMonitoringVariables();										// to avoid getting stupid values
		prevControlLoopCallTime = StepTimer::GetTimerTicks();			// to avoid huge integral term windup
	}

	// If we are disabling closed loop mode, we should ideally send steps to get the microstep counter to match the current phase here
	currentMode = mode;

	return true;
}

// This is called just after the driver has switched into closed loop mode (it may have been in closed loop mode already)
void PhaseStep::DriverSwitchedToClosedLoop() noexcept
{
	delay(3);														// allow time for the switch to complete and a few control loop iterations to be done
	const uint16_t currentPhasePosition = (uint16_t)encoder->GetCurrentPhasePosition();
	if (currentMode == StepMode::assistedOpen)
	{
		const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);
		phaseOffset = (currentPhasePosition - stepPhase) & 4095;
	}
	desiredStepPhase = currentPhasePosition;
	SetMotorPhase(currentPhasePosition, SmartDrivers::GetStandstillCurrentPercent(0) * 0.01);	// set the motor currents to match the initial position using the open loop standstill current
	PIDITerm = 0.0;													// clear the integral term accumulator
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	ResetMonitoringVariables();										// the first loop iteration will have recorded a higher than normal loop call interval, so start again
}

#endif

// End
