/*
 * PhaseStep.h
 *
 *  Created on: 9 Jun 2020
 *      Author: David & Andy E
 */

#ifndef SRC_MOVE_PHASESTEP_H_
#define SRC_MOVE_PHASESTEP_H_

#include <RepRapFirmware.h>

#if SUPPORT_PHASE_STEPPING

# include <Platform/RepRap.h>
# include <General/NamedEnum.h>
# include <Movement/StepTimer.h>
# include <Movement/Trigonometry.h>

constexpr float MaxSafeBacklash = 0.22;					// the maximum backlash in full steps that we can use - error if there is more
constexpr float MaxGoodBacklash = 0.15;					// the maximum backlash in full steps that we are happy with - warn if there is more
constexpr unsigned int LinearEncoderIncreaseFactor = 4;	// this should be a power of 2. Allowed backlash is increased by this amount for linear composite encoders.
constexpr float VelocityLimitGainFactor = 5.0;			// the gain of the P loop when in torque mode


// Struct to pass data back to the ClosedLoop module
struct MotionParameters
{
	float position = 0.0;
	float speed = 0.0;
	float acceleration = 0.0;
};

enum class StepMode
{
	stepDir = 0,
	phase,
	unknown
};

enum class PhaseStepConfig
{
	kv = 0,
	ka,
};

struct PhaseStepParams
{
	float Kv;
	float Ka;
};

const char* TranslateStepMode(const StepMode mode);

class PhaseStep
{
public:
	friend class Move;
	friend class DriveMovement;

	// Phase step public methods
	void SetStandstillCurrent(float percent) noexcept;

	// Methods called by the motion system
	void Calculate() noexcept; // Calculate the phase and current before the per driver offset is applied
	void InstanceControlLoop(size_t driver) noexcept;
	void SetEnabled(bool enable) { enabled = enable; }
	bool IsEnabled() const noexcept { return enabled; }
	void UpdatePhaseOffset(size_t driver) noexcept;
	void SetPhaseOffset(size_t driver, uint16_t offset) noexcept;

	// Configuration methods
	void SetKv(float newKv) noexcept { Kv = newKv; }
	void SetKa(float newKa) noexcept { Ka = newKa; }
	float GetKv() const noexcept { return Kv; }
	float GetKa() const noexcept { return Ka; }

  private:
	// Constants private to this module
	static constexpr float DefaultHoldCurrentFraction =
		0.71; // the minimum fraction of the requested current that we apply when holding position

	// Methods used only by closed loop and by the tuning module
	void SetMotorPhase(size_t driver, uint16_t phase, float magnitude) noexcept;

	bool enabled = false;

	// Holding current, and variables derived from it
	float holdCurrentFraction = DefaultHoldCurrentFraction; // The minimum holding current when stationary
	float Kv = 1000.0;										// The velocity feedforward constant
	float Ka = 50000.0;										// The acceleration feedforward constant

	// Working variables
	// These variables are all used to calculate the required motor currents. They are declared here so they can be reported on by the data collection task
	MotionParameters mParams; // the target position, speed and acceleration

	float	PIDVTerm;									// Velocity feedforward term
	float	PIDATerm;									// Acceleration feedforward term
	float	PIDControlSignal;							// The overall signal from the PID controller

	float currentFraction;
	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

	// Functions private to this module
	uint16_t CalculateStepPhase(size_t driver) noexcept;
	float CalculateCurrentFraction() noexcept;
	void ResetMonitoringVariables() noexcept;
};

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */