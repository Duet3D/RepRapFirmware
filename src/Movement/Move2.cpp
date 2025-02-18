/*
 * Move2.cpp
 *
 *  Created on: 8 Nov 2024
 *      Author: David
 *
 *  This file contains the movement system configuration and reporting functions
 */

#include "Move.h"
#include <Platform/RepRap.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Tools/Tool.h>
#include <CAN/CanInterface.h>
#include <CAN/CanDriversData.h>
#include "StepperDrivers/SmartDrivers.h"

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
# include <CanMessageGenericTables.h>
#endif

// Set the microstepping for local drivers, returning true if successful. All drivers for the same axis must use the same microstepping.
// Caller must deal with remote drivers.
bool Move::SetMicrostepping(size_t drive, unsigned int microsteps, bool interp, const StringRef& reply) noexcept
{
	const bool ret = SetDriversMicrostepping(drive, microsteps, interp, reply);
	if (ret)
	{
		microstepping[drive] = (interp) ? microsteps | 0x8000 : microsteps;
		reprap.MoveUpdated();
	}
	return ret;
}

// Get the microstepping for an axis or extruder
unsigned int Move::GetMicrostepping(size_t drive, bool& interpolation) const noexcept
{
	interpolation = (microstepping[drive] & 0x8000) != 0;
	return microstepping[drive] & 0x7FFF;
}

// Set the drive steps per mm. Called when processing M92. Return the new steps/mm divided by the old one.
float Move::SetDriveStepsPerMm(size_t axisOrExtruder, float value, uint32_t requestedMicrostepping) noexcept
{
	if (requestedMicrostepping != 0)
	{
		const uint32_t currentMicrostepping = microstepping[axisOrExtruder] & 0x7FFF;
		if (currentMicrostepping != requestedMicrostepping)
		{
			value = value * (float)currentMicrostepping / (float)requestedMicrostepping;
		}
	}

	value = max<float>(value, MinimumStepsPerMm);					// don't allow zero or negative
	const float ret = value/driveStepsPerMm[axisOrExtruder];
	driveStepsPerMm[axisOrExtruder] = value;
	reprap.MoveUpdated();
	return ret;
}

// Process M205 or M566
void Move::SetInstantDv(size_t drive, float value, bool includingMax) noexcept
{
	const float val = max<float>(value, ConvertSpeedFromMmPerSec(MinimumJerk));			// don't allow zero or negative values, they causes Move to loop indefinitely
	if (includingMax)
	{
		printingInstantDvs[drive] = maxInstantDvs[drive] = val;
	}
	else
	{
		printingInstantDvs[drive] = min<float>(val, maxInstantDvs[drive]);
	}
}

// Process M208
void Move::SetAxisMaximum(size_t axis, float value, bool byProbing) noexcept
{
	axisMaxima[axis] = value;
	if (byProbing)
	{
		axisMaximaProbed.SetBit(axis);
	}
	reprap.MoveUpdated();
}

// process M208
void Move::SetAxisMinimum(size_t axis, float value, bool byProbing) noexcept
{
	axisMinima[axis] = value;
	if (byProbing)
	{
		axisMinimaProbed.SetBit(axis);
	}
	reprap.MoveUpdated();
}

// Process M425
GCodeResult Move::ConfigureBacklashCompensation(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;
	size_t totalAxes = reprap.GetGCodes().GetTotalAxes();
	for (size_t i = 0; i < totalAxes; ++i)
	{
		if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[i]))
		{
			seen = true;
			backlashMm[i] = gb.GetNonNegativeFValue();
		}
	}

	if (gb.Seen('S'))
	{
		seen = true;
		backlashCorrectionDistanceFactor = gb.GetLimitedUIValue('S', 1, 101);
	}

	if (seen)
	{
		UpdateBacklashSteps();
		reprap.MoveUpdated();
	}
	else
	{
		reply.copy("Backlash correction (mm)");
		for (size_t i = 0; i < totalAxes; ++i)
		{
			reply.catf(" %c: %.3f", reprap.GetGCodes().GetAxisLetters()[i], (double)backlashMm[i]);
		}
		reply.catf(", correction distance multiplier %" PRIu32, backlashCorrectionDistanceFactor);
	}
	return GCodeResult::ok;
}

// Process M595
GCodeResult Move::ConfigureMovementQueue(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const size_t ringNumber = (gb.Seen('Q')) ? gb.GetLimitedUIValue('Q', ARRAY_SIZE(rings)) : 0;
	return rings[ringNumber].ConfigureMovementQueue(gb, reply);
}

// Process M572
GCodeResult Move::ConfigurePressureAdvance(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (gb.Seen('S'))
	{
		const float advance = gb.GetNonNegativeFValue();
		if (!reprap.GetGCodes().LockCurrentMovementSystemAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}

		GCodeResult rslt = GCodeResult::ok;

#if SUPPORT_CAN_EXPANSION
		CanDriversData<float> canDriversToUpdate;
#endif
		if (gb.Seen('D'))
		{
			uint32_t eDrive[MaxExtruders];
			size_t eCount = MaxExtruders;
			gb.GetUnsignedArray(eDrive, eCount, false);
			for (size_t i = 0; i < eCount; i++)
			{
				const uint32_t extruder = eDrive[i];
				if (extruder >= reprap.GetGCodes().GetNumExtruders())
				{
					reply.printf("Invalid extruder number '%" PRIu32 "'", extruder);
					rslt = GCodeResult::error;
					break;
				}
				GetExtruderShaperForExtruder(extruder).SetKseconds(advance);
#if SUPPORT_CAN_EXPANSION
				const DriverId did = GetExtruderDriver(extruder);
				if (did.IsRemote())
				{
					canDriversToUpdate.AddEntry(did, advance);
				}
#endif
			}
		}
		else
		{
			const Tool *_ecv_null const ct = reprap.GetGCodes().GetConstMovementState(gb).currentTool;
			if (ct == nullptr)
			{
				reply.copy("No tool selected");
				rslt = GCodeResult::error;
			}
			else
			{
#if SUPPORT_CAN_EXPANSION
				ct->IterateExtruders([this, advance, &canDriversToUpdate](unsigned int extruder)
										{
											GetExtruderShaperForExtruder(extruder).SetKseconds(advance);
											const DriverId did = GetExtruderDriver(extruder);
											if (did.IsRemote())
											{
												canDriversToUpdate.AddEntry(did, advance);
											}
										}
									);
#else
				ct->IterateExtruders([this, advance](unsigned int extruder)
										{
											GetExtruderShaperForExtruder(extruder).SetKseconds(advance);
										}
									);
#endif
			}
		}

		reprap.MoveUpdated();

#if SUPPORT_CAN_EXPANSION
		return max<GCodeResult>(rslt, CanInterface::SetRemotePressureAdvance(canDriversToUpdate, reply));
#else
		return rslt;
#endif
	}

	reply.copy("Extruder pressure advance");
	char c = ':';
	for (size_t i = 0; i < reprap.GetGCodes().GetNumExtruders(); ++i)
	{
		reply.catf("%c %.3f", c, (double)GetExtruderShaperForExtruder(i).GetKseconds());
		c = ',';
	}
	return GCodeResult::ok;
}

// Process M208
GCodeResult Move::ConfigureAxisLimits(GCodeBuffer& gb, const StringRef& reply, const char *_ecv_array axisLetters, size_t numTotalAxes, bool inM501) THROWS(GCodeException)
{
	bool setMin = (gb.Seen('S') ? (gb.GetIValue() == 1) : false);
	bool seen = false;
	for (size_t axis = 0; axis < numTotalAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			seen = true;
			float values[2];
			size_t numValues = 2;
			gb.GetFloatArray(values, numValues, false);
			bool ok;
			if (numValues == 2)
			{
				ok = values[1] > values[0];
				if (ok)
				{
					SetAxisMinimum(axis, values[0], gb.LatestMachineState().runningM501);
					SetAxisMaximum(axis, values[1], gb.LatestMachineState().runningM501);
				}
			}
			else if (setMin)
			{
				ok = AxisMaximum(axis) > values[0];
				if (ok)
				{
					SetAxisMinimum(axis, values[0], gb.LatestMachineState().runningM501);
				}
			}
			else
			{
				ok = values[0] > AxisMinimum(axis);
				if (ok)
				{
					SetAxisMaximum(axis, values[0], gb.LatestMachineState().runningM501);
				}
			}

			if (!ok)
			{
				reply.printf("%c axis maximum must be greater than minimum", axisLetters[axis]);
				return GCodeResult::error;
			}
		}
	}

	if (!seen)
	{
		reply.copy("Axis limits (mm");
		char sep = ')';
		for (size_t axis = 0; axis < numTotalAxes; axis++)
		{
			reply.catf("%c %c%.1f:%.1f", sep, axisLetters[axis], (double)AxisMinimum(axis), (double)AxisMaximum(axis));
			sep = ',';
		}
	}
	return GCodeResult::ok;
}

#if SUPPORT_NONLINEAR_EXTRUSION

// Process M592
GCodeResult Move::ConfigureNonlinearExtrusion(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const unsigned int extruder = gb.GetLimitedUIValue('D', MaxExtruders);
	bool seen = false;
	float a = 0.0, b = 0.0, limit = DefaultNonlinearExtrusionLimit;
	gb.TryGetFValue('A', a, seen);
	gb.TryGetFValue('B', b, seen);
	gb.TryGetNonNegativeFValue('L', limit, seen);
	if (seen)
	{
		nonlinearExtrusion[extruder].limit = limit;
		nonlinearExtrusion[extruder].A = a;
		nonlinearExtrusion[extruder].B = b;
		reprap.MoveUpdated();
	}
	else
	{
		const NonlinearExtrusion& nl = GetExtrusionCoefficients(extruder);
		reply.printf("Drive %u nonlinear extrusion coefficients: A=%.3g, B=%.3g, limit=%.2f", extruder, (double)nl.A, (double)nl.B, (double)nl.limit);
	}
	return GCodeResult::ok;
}

#endif

// Return the position of an endstop in steps. Only called for kinematics that home drives individually.
int32_t Move::GetEndstopPositionSteps(size_t drive, bool highEnd) noexcept
{
	return kinematics->GetEndstopPosition(drive, highEnd) * driveStepsPerMm[drive];
}

// This is called from the step ISR as well as other places, so keep it fast
// If drive >= MaxAxesPlusExtruders then we are setting an individual motor direction
// It is the responsibility of the caller to ensure that minimum timings between step pulses and direction changes are observed.
void Move::SetDriversDirection(size_t axisOrExtruder, bool direction) noexcept
{
	IterateLocalDrivers(axisOrExtruder, [this, direction](uint8_t driver) { this->SetOneDriverDirection(driver, direction); });
}

// Enable a driver. Must not be called from an ISR, or with interrupts disabled.
void Move::EnableOneLocalDriver(size_t driver, float requiredCurrent) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
		motorOffTimers[driver].Stop();

#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
		if (driver < numSmartDrivers && !reprap.GetPlatform().HasDriverPower())
		{
			reprap.GetPlatform().WarnDriverNotPowered();
		}
		else
		{
#endif
			UpdateMotorCurrent(driver, requiredCurrent);

#if defined(DUET3) && HAS_SMART_DRIVERS
			SmartDrivers::EnableDrive(driver, true);	// all drivers driven directly by the main board are smart
#elif HAS_SMART_DRIVERS
			if (driver < numSmartDrivers)
			{
				SmartDrivers::EnableDrive(driver, true);
			}
# if !defined(DUET3MINI)								// no enable pins on 5LC
			else
			{
				digitalWrite(ENABLE_PINS[driver], enableValues[driver] > 0);
			}
# endif
#else
			digitalWrite(ENABLE_PINS[driver], enableValues[driver] > 0);
#endif
#if HAS_SMART_DRIVERS && (HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR)
		}
#endif
		if (brakePorts[driver].IsValid() && !brakePorts[driver].ReadDigital())
		{
			if (brakeOffDelays[driver] != 0)
			{
				brakeOffTimers[driver].Start();
			}
			else
			{
				DisengageBrake(driver);
			}
		}
	}
}

// Disable a driver
void Move::DisableOneLocalDriver(size_t driver) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
		brakeOffTimers[driver].Stop();
		EngageBrake(driver);
		if (motorOffDelays[driver] != 0 && brakePorts[driver].IsValid() && brakePorts[driver].ReadDigital())
		{
			motorOffTimers[driver].Start();
		}
		else
		{
			InternalDisableDriver(driver);
		}
	}
}

void Move::InternalDisableDriver(size_t driver) noexcept
{
#if defined(DUET3) && HAS_SMART_DRIVERS
	SmartDrivers::EnableDrive(driver, false);		// all drivers driven directly by the main board are smart
#elif HAS_SMART_DRIVERS
	if (driver < numSmartDrivers)
	{
		SmartDrivers::EnableDrive(driver, false);
	}
# if !defined(DUET3MINI)		// Duet 5LC has no enable pins
	else
	{
		digitalWrite(ENABLE_PINS[driver], enableValues[driver] <= 0);
	}
# endif
#else
	digitalWrite(ENABLE_PINS[driver], enableValues[driver] <= 0);
#endif
}

// Enable the local drivers for a drive. Must not be called from an ISR, or with interrupts disabled.
void Move::EnableDrivers(size_t axisOrExtruder, bool unconditional) noexcept
{
	if (unconditional || driverState[axisOrExtruder] != DriverStatus::enabled)
	{
		driverState[axisOrExtruder] = DriverStatus::enabled;
		const float requiredCurrent = motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder];
#if SUPPORT_CAN_EXPANSION
		CanDriversList canDriversToEnable;
		IterateDrivers(axisOrExtruder,
						[this, requiredCurrent](uint8_t driver) { EnableOneLocalDriver(driver, requiredCurrent); },
						[&canDriversToEnable](DriverId driver) { canDriversToEnable.AddEntry(driver); }
					  );
		CanInterface::EnableRemoteDrivers(canDriversToEnable);
#else
		IterateDrivers(axisOrExtruder,
						[this, requiredCurrent](uint8_t driver) { EnableOneLocalDriver(driver, requiredCurrent); }
					  );
#endif
	}
}

// Disable the drivers for a drive
void Move::DisableDrivers(size_t axisOrExtruder) noexcept
{
#if SUPPORT_CAN_EXPANSION
	CanDriversList canDriversToDisable;

	IterateDrivers(axisOrExtruder,
					[this](uint8_t driver) { DisableOneLocalDriver(driver); },
					[&canDriversToDisable](DriverId driver) { canDriversToDisable.AddEntry(driver); }
				  );
	CanInterface::DisableRemoteDrivers(canDriversToDisable);
#else
	IterateDrivers(axisOrExtruder,
					[this](uint8_t driver) { DisableOneLocalDriver(driver); }
				  );
#endif
	driverState[axisOrExtruder] = DriverStatus::disabled;
}

// Disable all drives in an emergency. Called from emergency stop and the tick ISR.
// This is only called in an emergency, so we don't update the driver status
void Move::EmergencyDisableDrivers() noexcept
{
	for (size_t drive = 0; drive < GetNumActualDirectDrivers(); drive++)
	{
		if (!inInterrupt())		// on the Duet 06/085 we need interrupts running to send the I2C commands to set motor currents
		{
			UpdateMotorCurrent(drive, 0.0);
		}
		DisableOneLocalDriver(drive);
	}
}

void Move::DisableAllDrivers() noexcept
{
	for (size_t axisOrExtruder = 0; axisOrExtruder < MaxAxesPlusExtruders; axisOrExtruder++)
	{
		DisableDrivers(axisOrExtruder);
	}
}

void Move::EngageBrake(size_t driver) noexcept
{
#if SUPPORT_BRAKE_PWM
	currentBrakePwm[driver] = 0.0;
	brakePorts[driver].WriteAnalog(0.0);
#else
	brakePorts[driver].WriteDigital(false);			// turn the brake solenoid off to engage the brake
#endif
}

void Move::DisengageBrake(size_t driver) noexcept
{
#if SUPPORT_BRAKE_PWM
	// Set the PWM to deliver the requested voltage regardless of the VIN voltage
	currentBrakePwm[driver] = min<float>(brakeVoltages[driver]/max<float>(reprap.GetPlatform().GetVinVoltage(), 1.0), 1.0);
	brakePorts[driver].WriteAnalog(currentBrakePwm[driver]);
#else
	brakePorts[driver].WriteDigital(true);			// turn the brake solenoid on to disengage the brake
#endif
}

StandardDriverStatus Move::GetLocalDriverStatus(size_t driver) const noexcept
{
#if defined(DUET3_MB6XD)
	return StandardDriverStatus((HasDriverError(driver)) ? (uint32_t)1u << StandardDriverStatus::ExternDriverErrorBitPos : 0);
#else
	return SmartDrivers::GetStatus(driver, false, false);		// it's safe to call this even when driver >= MaxSmartDrivers
#endif
}

// Set drives to idle hold if they are enabled. If a drive is disabled, leave it alone.
// Must not be called from an ISR, or with interrupts disabled.
void Move::SetDriversIdle() noexcept
{
	if (idleCurrentFactor == 0.0)
	{
		DisableAllDrivers();
		reprap.GetGCodes().SetAllAxesNotHomed();
	}
	else
	{
#if SUPPORT_CAN_EXPANSION
		CanDriversList canDriversToSetIdle;
#endif
		for (size_t axisOrExtruder = 0; axisOrExtruder < MaxAxesPlusExtruders; ++axisOrExtruder)
		{
			if (driverState[axisOrExtruder] == DriverStatus::enabled)
			{
				driverState[axisOrExtruder] = DriverStatus::idle;
				const float current = motorCurrents[axisOrExtruder] * idleCurrentFactor;
				IterateDrivers(axisOrExtruder,
								[this, current](uint8_t driver) { UpdateMotorCurrent(driver, current); }
#if SUPPORT_CAN_EXPANSION
								, [&canDriversToSetIdle](DriverId driver) { canDriversToSetIdle.AddEntry(driver); }
#endif
							  );
			}
		}
#if SUPPORT_CAN_EXPANSION
		CanInterface::SetRemoteDriversIdle(canDriversToSetIdle, idleCurrentFactor);
#endif
	}
}

// Configure the brake port for a driver
GCodeResult Move::ConfigureDriverBrakePort(GCodeBuffer& gb, const StringRef& reply, size_t driver) THROWS(GCodeException)
{
# if !SUPPORT_BRAKE_PWM
	if (gb.Seen('V'))
	{
		// Don't allow a brake port to be configured if the user specified a voltage and we don't support PWM
		reply.copy("Brake PWM not supported by this board");
		return GCodeResult::error;
	}
# endif

	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;
		if (!brakePorts[driver].AssignPort(gb, reply, PinUsedBy::gpout, PinAccess::write0))
		{
			return GCodeResult::error;
		}
# if SUPPORT_BRAKE_PWM
		brakePorts[driver].SetFrequency(BrakePwmFrequency);
# endif
		motorOffDelays[driver] = brakeOffDelays[driver] = DefaultDelayAfterBrakeOn;
	}

	uint32_t val;
	if (gb.TryGetLimitedUIValue('S', val, seen, 1000))
	{
		seen = true;
		motorOffDelays[driver] = brakeOffDelays[driver] = (uint16_t)val;
	}

# if SUPPORT_BRAKE_PWM
	gb.TryGetNonNegativeFValue('V', brakeVoltages[driver], seen);
# endif

	if (!seen)
	{
		reply.printf("Driver %u uses brake port ", driver);
		brakePorts[driver].AppendPinName(reply);
# if SUPPORT_BRAKE_PWM
		if (brakeVoltages[driver] < FullyOnBrakeVoltage)
		{
			reply.catf(" with voltage limited to %.1f by PWM", (double)brakeVoltages[driver]);
		}
# endif
		reply.catf(", brake delay %ums", motorOffDelays[driver]);
	}
	return GCodeResult::ok;
}

// Set the current for all drivers on an axis or extruder. Current is in mA.
GCodeResult Move::SetMotorCurrent(size_t axisOrExtruder, float currentOrPercent, int code, const StringRef& reply) noexcept
{
	switch (code)
	{
	case 906:
		motorCurrents[axisOrExtruder] = currentOrPercent;
		break;

	case 913:
		motorCurrentFraction[axisOrExtruder] = constrain<float>(0.01 * currentOrPercent, 0.0, 1.0);
		break;

#if HAS_SMART_DRIVERS || SUPPORT_CAN_EXPANSION
	case 917:
		standstillCurrentPercent[axisOrExtruder] = constrain<float>(currentOrPercent, 0.0, 100.0);
		break;
#endif

	default:
		return GCodeResult::error;
	}

#if SUPPORT_CAN_EXPANSION
	CanDriversData<float> canDriversToUpdate;

	IterateDrivers(axisOrExtruder,
							[this, axisOrExtruder, code](uint8_t driver)
							{
								if (code == 917)
								{
# if HAS_SMART_DRIVERS
									SmartDrivers::SetStandstillCurrentPercent(driver, standstillCurrentPercent[axisOrExtruder]);
# endif
								}
								else
								{
									UpdateMotorCurrent(driver, motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder]);
								}
							},
							[this, axisOrExtruder, code, &canDriversToUpdate](DriverId driver)
							{
								if (code == 917)
								{
									canDriversToUpdate.AddEntry(driver, standstillCurrentPercent[axisOrExtruder]);
								}
								else
								{
									canDriversToUpdate.AddEntry(driver, motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder]);
								}
							}
						);
	if (code == 917)
	{
# if SUPPORT_PHASE_STEPPING
		dms[axisOrExtruder].phaseStepControl.SetStandstillCurrent(standstillCurrentPercent[axisOrExtruder]);
# endif
		return CanInterface::SetRemoteStandstillCurrentPercent(canDriversToUpdate, reply);
	}
	else
	{
		return CanInterface::SetRemoteDriverCurrents(canDriversToUpdate, reply);
	}
#else
	IterateDrivers(axisOrExtruder,
							[this, axisOrExtruder, code](uint8_t driver)
							{
								if (code == 917)
								{
# if HAS_SMART_DRIVERS
									SmartDrivers::SetStandstillCurrentPercent(driver, standstillCurrentPercent[axisOrExtruder]);
# endif
								}
								else
								{
									UpdateMotorCurrent(driver, motorCurrents[axisOrExtruder] * motorCurrentFraction[axisOrExtruder]);
								}
							}
	);
	return GCodeResult::ok;
#endif
}

#ifdef DUET3_MB6XD

// Fetch the worst (longest) timings of any driver, set up the step pulse width timer, and convert the other timings from microseconds to step clocks
void Move::UpdateDriverTimings() noexcept
{
	float worstTimings[4] = { 0.1, 0.1, 0.0, 0.0 };					// minimum 100ns step high/step low time, zero direction setup/hold time
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		for (size_t i = 0; i < 4; ++i)
		{
			if (driverTimingMicroseconds[driver][i] > worstTimings[i])
			{
				worstTimings[i] = driverTimingMicroseconds[driver][i];
			}
		}
	}

	// Convert the step pulse width to clocks of the step pulse gate timer. First define some constants.
	constexpr uint32_t StepGateTcBaseClockFrequency = (SystemCoreClockFreq/2)/8;										// the step gate T clock frequency when we use a prescaler of 8
	constexpr float StepGateBaseClocksPerMicrosecond = (float)StepGateTcBaseClockFrequency * 1.0e-6;
	const float fclocks = min<float>(ceilf(worstTimings[0] * StepGateBaseClocksPerMicrosecond), (float)(4 * 65535));	// the TC is only 16 bits wide, but we increase the prescaler to 32 if necessary

	uint32_t iclocks = (uint32_t)fclocks;
	uint32_t clockPrescaler = TC_CMR_TCCLKS_TIMER_CLOCK2;								// divide MCLK (150MHz) by 8 = 18.75MHz
	if (iclocks > 65535)
	{
		clockPrescaler = TC_CMR_TCCLKS_TIMER_CLOCK3;									// divide MCLK (150MHz) by 32 = 4.6875MHz
		iclocks >>= 2;
	}

	STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_CCR = TC_CCR_CLKDIS;
	STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_CMR =  TC_CMR_BSWTRG_SET				// software trigger sets TIOB
														| TC_CMR_BCPC_CLEAR				// RC compare clears TIOB
														| TC_CMR_WAVE					// waveform mode
														| TC_CMR_WAVSEL_UP				// count up
														| TC_CMR_CPCSTOP				// counter clock is stopped when counter reaches RC
														| TC_CMR_EEVT_XC0   			// set external events from XC0 (this allows TIOB to be an output)
														| clockPrescaler;				// divide MCLK (150MHz) by 8 or 32
	STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_RC = iclocks;
	STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_CCR = TC_CCR_CLKEN;

	// Convert the quantised step pulse width back to microseconds
	const float actualStepPulseMicroseconds = fclocks/StepGateBaseClocksPerMicrosecond;

	// Now convert the other values from microseconds to step clocks
	stepPulseMinimumPeriodClocks = MicrosecondsToStepClocks(worstTimings[1] + actualStepPulseMicroseconds);
	directionSetupClocks = MicrosecondsToStepClocks(worstTimings[2]);
	directionHoldClocksFromLeadingEdge = MicrosecondsToStepClocks(worstTimings[3] + actualStepPulseMicroseconds);
//DEBUG
//	debugPrintf("Clocks: %" PRIu32 " %" PRIu32 " %" PRIu32 "\n", stepPulseMinimumPeriodClocks, directionSetupClocks, directionHoldClocksFromLeadingEdge);
}

void Move::GetActualDriverTimings(float timings[4]) noexcept
{
	uint32_t StepGateTcClockFrequency = (SystemCoreClockFreq/2)/8;
	if ((STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_CMR & TC_CMR_TCCLKS_Msk) == TC_CMR_TCCLKS_TIMER_CLOCK3)
	{
		StepGateTcClockFrequency >>= 2;;
	}
	const float MicrosecondsPerStepGateClock = 1.0e6/(float)StepGateTcClockFrequency;
	constexpr float StepClocksToMicroseconds = 1.0e6/(float)StepClockRate;
	timings[0] = (float)STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_RC * MicrosecondsPerStepGateClock;
	timings[1] = stepPulseMinimumPeriodClocks * StepClocksToMicroseconds - timings[0];
	timings[2] = directionSetupClocks * StepClocksToMicroseconds;
	timings[3] = directionHoldClocksFromLeadingEdge * StepClocksToMicroseconds - timings[0];
}

#endif

// This must not be called from an ISR, or with interrupts disabled.
void Move::UpdateMotorCurrent(size_t driver, float current) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if HAS_SMART_DRIVERS
		if (driver < numSmartDrivers)
		{
			SmartDrivers::SetCurrent(driver, current);
		}
#else
		// otherwise we can't set the motor current
#endif
	}
}

// Get the configured motor current for an axis or extruder
int Move::GetMotorCurrent(size_t drive, int code) const noexcept
{
	float rslt;
	switch (code)
	{
	case 906:
		rslt = motorCurrents[drive];
		break;

	case 913:
		rslt = motorCurrentFraction[drive] * 100.0;
		break;

#if HAS_SMART_DRIVERS || SUPPORT_CAN_EXPANSION
	case 917:
		rslt = standstillCurrentPercent[drive];
		break;
#endif
	default:
		rslt = 0.0;
		break;
	}

	return lrintf(rslt);
}

// Set the motor idle current factor
void Move::SetIdleCurrentFactor(float f) noexcept
{
	idleCurrentFactor = constrain<float>(f, 0.0, 1.0);
	reprap.MoveUpdated();

#if SUPPORT_CAN_EXPANSION
	CanDriversList canDriversToUpdate;
#endif
	for (size_t axisOrExtruder = 0; axisOrExtruder < MaxAxesPlusExtruders; ++axisOrExtruder)
	{
		if (driverState[axisOrExtruder] == DriverStatus::idle)
		{
			IterateDrivers(axisOrExtruder,
							[this, axisOrExtruder](uint8_t driver){ UpdateMotorCurrent(driver, motorCurrents[axisOrExtruder] * idleCurrentFactor); }
#if SUPPORT_CAN_EXPANSION
							, [this, &canDriversToUpdate](DriverId driver) { canDriversToUpdate.AddEntry(driver); }
#endif
						  );
		}
	}
#if SUPPORT_CAN_EXPANSION
	CanInterface::SetRemoteDriversIdle(canDriversToUpdate, idleCurrentFactor);
#endif
}

bool Move::SetDriversMicrostepping(size_t axisOrExtruder, unsigned int microsteps, bool interp, const StringRef& reply) noexcept
{
	bool ok = true;
	IterateLocalDrivers(axisOrExtruder,
					[this, microsteps, interp, &ok, reply](uint8_t driver) noexcept
					{
						if (!SetDriverMicrostepping(driver, microsteps, interp))
						{
							reply.lcatf("Driver %u does not support x%u microstepping", driver, microsteps);
							if (interp)
							{
								reply.cat(" with interpolation");
							}
							ok = false;
						}
					}
				  );
	return ok;
}

// Set the microstepping for a driver, returning true if successful
bool Move::SetDriverMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
#if HAS_SMART_DRIVERS
		if (driver < numSmartDrivers)
		{
			return SmartDrivers::SetMicrostepping(driver, microsteps, interpolate);
		}
		else
		{
			// Other drivers only support x16 microstepping.
			// We ignore the interpolation on/off parameter so that e.g. M350 I1 E16:128 won't give an error if E1 supports interpolation but E0 doesn't.
			return microsteps == 16;
		}
#else
		// Assume only x16 microstepping supported
		return microsteps == 16;
#endif
	}
	return false;
}

void Move::SetEnableValue(size_t driver, int8_t eVal) noexcept
{
	if (driver < GetNumActualDirectDrivers())
	{
		enableValues[driver] = eVal;
		DisableOneLocalDriver(driver);				// disable the drive, because the enable polarity may have been wrong before
#if HAS_SMART_DRIVERS
		if (eVal == -1)
		{
			// User has asked to disable status monitoring for this driver, so clear its error bits
			const LocalDriversBitmap mask = ~LocalDriversBitmap::MakeFromBits(driver);
			temperatureShutdownDrivers &= mask;
			temperatureWarningDrivers &= mask;
			shortToGroundDrivers &= mask;
			if (driver < MaxSmartDrivers)
			{
				openLoadTimers[driver].Stop();
			}
		}
#endif
	}
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult Move::EutSetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
# if HAS_SMART_DRIVERS
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) noexcept
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SetMotorCurrent(driver, msg.values[count], 906, reply);
							}
						}
				   );
	return rslt;
# else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
# endif
}

GCodeResult Move::EutSetStepsPerMmAndMicrostepping(const CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping>& msg, size_t dataLength, const StringRef& reply) noexcept
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) noexcept
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								(void)SetDriveStepsPerMm(driver, msg.values[count].GetStepsPerUnit(), 0);
#if HAS_SMART_DRIVERS
								const uint16_t rawMicrostepping = msg.values[count].GetMicrostepping();
								const uint16_t microsteppingOnly = rawMicrostepping & 0x03FF;
								const bool interpolate = (rawMicrostepping & 0x8000) != 0;
								SetMicrostepping(driver, microsteppingOnly, interpolate, reply);
#endif
							}
						}
					);
	return rslt;
}

GCodeResult Move::EutHandleSetDriverStates(const CanMessageMultipleDrivesRequest<DriverStateControl>& msg, const StringRef& reply) noexcept
{
	//TODO check message is long enough for the number of drivers specified
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	drivers.Iterate([this, &msg](unsigned int driver, unsigned int count) noexcept
		{
			switch (msg.values[count].mode)
			{
			case DriverStateControl::driverActive:
				EnableOneLocalDriver(driver, motorCurrents[driver]);
				driverState[driver] = DriverStatus::enabled;
				break;

			case DriverStateControl::driverIdle:
				UpdateMotorCurrent(driver, motorCurrents[driver] * (float)msg.values[count].idlePercent * 0.01);
				driverState[driver] = DriverStatus::idle;
				break;

			case DriverStateControl::driverDisabled:
			default:
				DisableOneLocalDriver(driver);
				driverState[driver] = DriverStatus::disabled;
				break;
			}
		});
	return GCodeResult::ok;
}

GCodeResult Move::EutProcessM569(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDirectDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	bool seen = false;
	bool warn = false;
	uint8_t direction;
	if (parser.GetUintParam('S', direction))
	{
		seen = true;
		SetDirectionValue(drive, direction != 0);
	}
	int8_t rValue;
	if (parser.GetIntParam('R', rValue))
	{
		seen = true;
		SetEnableValue(drive, rValue);
	}

	float timings[4];
	size_t numTimings = 4;
	if (parser.GetFloatArrayParam('T', numTimings, timings))
	{
		seen = true;
		if (numTimings == 1)
		{
			timings[1] = timings[2] = timings[3] = timings[0];
		}
		else if (numTimings != 4)
		{
			reply.copy("bad timing parameter, expected 1 or 4 values");
			return GCodeResult::error;
		}
		SetDriverStepTiming(drive, timings);
	}

#if HAS_SMART_DRIVERS
	{
		uint32_t val;
# if SUPPORT_TMC51xx
		int32_t ival;
# endif
		if (parser.GetUintParam('D', val))	// set driver mode
		{
			seen = true;
			if (!SmartDrivers::SetDriverMode(drive, val))
			{
				reply.printf("Driver %u.%u does not support mode '%s'", CanInterface::GetCanAddress(), drive, TranslateDriverMode(val));
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('F', val))		// set off time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::toff, val))
			{
				reply.printf("Bad off time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('B', val))		// set blanking time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tblank, val))
			{
				reply.printf("Bad blanking time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('V', val))		// set microstep interval for changing from stealthChop to spreadCycle
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tpwmthrs, val))
			{
				reply.printf("Bad mode change microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}

#if SUPPORT_TMC51xx
		if (parser.GetUintParam('H', val))		// set coolStep threshold
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::thigh, val))
			{
				reply.printf("Bad high speed microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetIntParam('U', ival))
		{
			seen = true;
			if (!SmartDrivers::SetCurrentScaler(drive, ival))
			{
				reply.printf("Bad current scaler for driver %u", drive);
				return GCodeResult::error;
			}
			if (ival >= 0 && ival < 16)
			{
				reply.printf("Current scaler = %ld for driver %u might result in poor microstep performance. Recommended minimum is 16.", ival, drive);
				warn = true;
			}
		}
#endif
	}

	size_t numHvalues = 3;
	const uint8_t *hvalues;
	if (parser.GetArrayParam('Y', ParamDescriptor::ParamType::uint8_array, numHvalues, hvalues))		// set spread cycle hysteresis
	{
		seen = true;
		if (numHvalues == 2 || numHvalues == 3)
		{
			// There is a constraint on the sum of HSTRT and HEND, so set HSTART then HEND then HSTART again because one may go up and the other down
			(void)SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			bool ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hend, hvalues[1]);
			if (ok)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			}
			if (ok && numHvalues == 3)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hdec, hvalues[2]);
			}
			if (!ok)
			{
				reply.printf("Bad hysteresis setting for driver %u", drive);
				return GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Expected 2 or 3 Y values");
			return GCodeResult::error;
		}
	}
#endif

	if (warn)
	{
		return GCodeResult::warning;
	}

	if (!seen)
	{
		reply.printf("Driver %u.%u runs %s, active %s enable",
						CanInterface::GetCanAddress(),
						drive,
						(GetDirectionValue(drive)) ? "forwards" : "in reverse",
						(GetEnableValue(drive)) ? "high" : "low");

		float timings[4];
		const bool isSlowDriver = GetDriverStepTiming(drive, timings);
		if (isSlowDriver)
		{
			reply.catf(", step timing %.1f:%.1f:%.1f:%.1fus", (double)timings[0], (double)timings[1], (double)timings[2], (double)timings[3]);
		}
		else
		{
			reply.cat(", step timing fast");
		}

#if HAS_SMART_DRIVERS
		// It's a smart driver, so print the parameters common to all modes, except for the position
		reply.catf(", mode %s, ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32,
				TranslateDriverMode(SmartDrivers::GetDriverMode(drive)),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank)
			);

# if SUPPORT_TMC51xx
		{
			const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * DriveStepsPerMm(drive));
			const uint8_t iRun = SmartDrivers::GetIRun(drive);
			const uint8_t iHold = SmartDrivers::GetIHold(drive);
			const uint32_t gs = SmartDrivers::GetGlobalScaler(drive);
			const float current = SmartDrivers::GetCalculatedCurrent(drive);
			reply.catf(", thigh %" PRIu32 " (%.1f mm/sec), gs=%lu, iRun=%u, iHold=%u, current=%.3f", thigh, (double)mmPerSec, gs, iRun, iHold, (double)current);
		}
# endif

		// Print the additional parameters that are relevant in the current mode
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::spreadCycle)
		{
			reply.catf(", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
					  );
		}

# if SUPPORT_TMC22xx || SUPPORT_TMC51xx
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::stealthChop)
		{
			const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * DriveStepsPerMm(drive));
			const uint32_t pwmScale = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmScale);
			const uint32_t pwmAuto = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmAuto);
			const unsigned int pwmScaleSum = pwmScale & 0xFF;
			const int pwmScaleAuto = (int)((((pwmScale >> 16) & 0x01FF) ^ 0x0100) - 0x0100);
			const unsigned int pwmOfsAuto = pwmAuto & 0xFF;
			const unsigned int pwmGradAuto = (pwmAuto >> 16) & 0xFF;
			reply.catf(", tpwmthrs %" PRIu32 " (%.1f mm/sec), pwmScaleSum %u, pwmScaleAuto %d, pwmOfsAuto %u, pwmGradAuto %u",
						tpwmthrs, (double)mmPerSec, pwmScaleSum, pwmScaleAuto, pwmOfsAuto, pwmGradAuto);
		}
# endif
		// Finally, print the microstep position
		{
			const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
			if (mstepPos < 1024)
			{
				reply.catf(", pos %" PRIu32, mstepPos);
			}
			else
			{
				reply.cat(", pos unknown");
			}
		}
#endif

	}
	return GCodeResult::ok;
}

GCodeResult Move::EutProcessM569Point2(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
#if SUPPORT_TMC22xx || SUPPORT_TMC51xx
	CanMessageGenericParser parser(msg, M569Point2Params);
	uint8_t drive;
	uint8_t regNum;
	if (!parser.GetUintParam('P', drive) || !parser.GetUintParam('R', regNum))
	{
		reply.copy("Missing P or R parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDirectDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	uint32_t regVal;
	if (parser.GetUintParam('V', regVal))
	{
		return SmartDrivers::SetAnyRegister(drive, reply, regNum, regVal);
	}

	const uint32_t startTime = millis();
	GCodeResult rslt;
	while ((rslt = SmartDrivers::GetAnyRegister(drive, reply, regNum)) == GCodeResult::notFinished)
	{
		if (millis() - startTime >= 50)
		{
			reply.copy("Failed to read register");
			return GCodeResult::error;
		}
	}

	return rslt;
#else
	return GCodeResult::errorNotSupported;
#endif
}

GCodeResult Move::EutProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point7Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDirectDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

# if !SUPPORT_BRAKE_PWM
	if (parser.HasParameter('V'))
	{
		// Don't allow a brake port to be configured if the user specified a voltage and we don't support PWM
		reply.copy("Brake PWM not supported by this board");
		return GCodeResult::error;
	}
# endif

	bool seen = false;
	if (parser.HasParameter('C'))
	{
		seen = true;
		String<StringLength20> portName;
		parser.GetStringParam('C', portName.GetRef());
		//TODO use the following instead when we track the enable state of each driver individually
		//if (!brakePorts[drive].AssignPort(portName.c_str(), reply, PinUsedBy::gpout, (driverDisabled[drive]) ? PinAccess::write0 : PinAccess::write1)) ...
		if (!brakePorts[drive].AssignPort(portName.c_str(), reply, PinUsedBy::gpout, PinAccess::write0))
		{
			return GCodeResult::error;
		}
# if SUPPORT_BRAKE_PWM
		brakePorts[drive].SetFrequency(BrakePwmFrequency);
# endif
	}

# if SUPPORT_BRAKE_PWM
	if (parser.GetFloatParam('V', brakeVoltages[drive]))
	{
		seen = true;
	}
# endif

	if (!seen)
	{
		reply.printf("Driver %u.%u uses brake port ", CanInterface::GetCanAddress(), drive);
		brakePorts[drive].AppendPinName(reply);
# if SUPPORT_BRAKE_PWM
		if (brakeVoltages[drive] < FullyOnBrakeVoltage)
		{
			reply.catf(" with voltage limited to %.1f by PWM", (double)brakeVoltages[drive]);
		}
# endif
	}
	return GCodeResult::ok;
}

GCodeResult Move::EutProcessM915(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
#if HAS_SMART_DRIVERS
	CanMessageGenericParser parser(msg, M915Params);
	uint16_t driverBits;
	if (!parser.GetUintParam('d', driverBits))
	{
		reply.copy("missing parameter in M915 message");
		return GCodeResult::error;
	}

	const auto drivers = LocalDriversBitmap::MakeFromRaw(driverBits);

	bool seen = false;
	{
		int8_t sgThreshold;
		if (parser.GetIntParam('S', sgThreshold))
		{
			seen = true;
			drivers.Iterate([sgThreshold](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallThreshold(drive, sgThreshold); });
		}
	}

	{
		uint16_t stepsPerSecond;
		if (parser.GetUintParam('H', stepsPerSecond))
		{
			seen = true;
			drivers.Iterate([stepsPerSecond](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallMinimumStepsPerSecond(drive, stepsPerSecond); });
		}
	}

	{
		uint16_t coolStepConfig;
		if (parser.GetUintParam('T', coolStepConfig))
		{
			seen = true;
			drivers.Iterate([coolStepConfig](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetRegister(drive, SmartDriverRegister::coolStep, coolStepConfig); } );
		}
	}

	{
		uint8_t rParam;
		if (parser.GetUintParam('R', rParam))
		{
			seen = true;
			if (rParam != 0)
			{
				eventOnStallDrivers |= drivers;
			}
			else
			{
				eventOnStallDrivers &= ~drivers;
			}
		}
	}

	if (!seen)
	{
		drivers.Iterate([&reply, this](unsigned int drive, unsigned int) noexcept
									{
										reply.lcatf("Driver %u.%u: ", CanInterface::GetCanAddress(), drive);
										SmartDrivers::AppendStallConfig(drive, reply);
										reply.cat(", event on stall: ");
										reply.cat((eventOnStallDrivers.IsBitSet(drive)) ? "yes" : "no");
									}
					   );
	}

	return GCodeResult::ok;
#else
	reply.copy("stall detection not supported by this board");
	return GCodeResult::error;
#endif
}

void Move::SendDriversStatus(CanMessageBuffer& buf) noexcept
{
	CanMessageDriversStatus * const msg = buf.SetupRequestMessageNoRid<CanMessageDriversStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
# if HAS_SMART_DRIVERS
	msg->SetStandardFields(MaxSmartDrivers, false);
	for (size_t driver = 0; driver < MaxSmartDrivers; ++driver)
	{
		msg->openLoopData[driver].status = SmartDrivers::GetStatus(driver, false, false).AsU32();
	}
# else
	msg->SetStandardFields(NumDirectDrivers, false);
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		msg->openLoopData[driver].status = HasDriverError(driver) ? (uint32_t)1u << StandardDriverStatus::ExternDriverErrorBitPos : 0u;
	}
# endif
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::SendMessageNoReplyNoFree(&buf);
}

// Stop some drivers and update the corresponding motor positions
void Move::StopDriversFromRemote(uint16_t whichDrives) noexcept
{
	LocalDriversBitmap dr(whichDrives);
	dr.Iterate([this](size_t drive, unsigned int) noexcept
				{
					StopDriveFromRemote(drive);
				}
			  );
}

// Stall endstops
GCodeResult Move::SetStallEndstopReporting(const CanMessageEnableStallEndstop& msg, const StringRef& reply) noexcept
{
#if HAS_SMART_DRIVERS
	return SmartDrivers::SetStallEndstopReporting(msg.driverNumber, msg.speed, reply);
#else
	reply.printf("stall detection not supported on board %u", CanInterface::GetCanAddress());
	return GCodeResult::error;
#endif
}

#endif	// SUPPORT_REMOTE_COMMANDS

// End
