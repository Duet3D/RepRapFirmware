/*
 * LocalFan.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "LocalFan.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/TemperatureSensor.h>

void FanInterrupt(CallbackParameter cb) noexcept
{
	static_cast<LocalFan *>(cb.vp)->Interrupt();
}

LocalFan::LocalFan(unsigned int fanNum) noexcept
	: Fan(fanNum),
	  lastPwm(-1.0),									// force a refresh
	  lastVal(-1.0),
	  fanInterruptCount(0), fanLastResetTime(0), fanInterval(0),
	  blipping(false)
{
}

LocalFan::~LocalFan() noexcept
{
	port.WriteAnalog(0.0);
	port.Release();
	tachoPort.Release();
}

GCodeResult LocalFan::ReportPortDetails(const StringRef& str) const noexcept
{
	str.printf("Fan %u", fanNumber);
	port.AppendFullDetails(str);
	if (tachoPort.IsValid())
	{
		str.cat(" tacho");
		tachoPort.AppendBasicDetails(str);
	}
	return GCodeResult::ok;
}

GCodeResult LocalFan::SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept
{
	port.SetFrequency(freq);
	return GCodeResult::ok;
}

// Set the hardware PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
void LocalFan::SetHardwarePwm(float pwmVal) noexcept
{
	// Only set the PWM if it has changed, to avoid a lot of I2C traffic when we have a DueX5 connected
	if (pwmVal != lastPwm)
	{
		lastPwm = pwmVal;
		port.WriteAnalog(pwmVal);
	}
}

// Refresh the fan PWM
// Checking all the sensors is expensive, so only do this if checkSensors is true.
void LocalFan::InternalRefresh(bool checkSensors) noexcept
{
	float reqVal;

	if (sensorsMonitored.IsEmpty())
	{
		reqVal = (val <= 0.0) ? 0.0 : max<float>(val * maxVal, minVal);		// scale the requested PWM by the maximum, enforce the minimum
	}
	else if (!checkSensors)
	{
		reqVal = lastVal;
	}
	else
	{
		reqVal = 0.0;
		const bool bangBangMode = (triggerTemperatures[1] <= triggerTemperatures[0]);
		sensorsMonitored.Iterate
		([&reqVal, bangBangMode, this](unsigned int sensorNum, unsigned int) noexcept
			{
				const auto sensor = reprap.GetHeat().FindSensor(sensorNum);
				if (sensor.IsNotNull())
				{
					//TODO we used to turn the fan on if the associated heater was being tuned
					float ht;
					const TemperatureError err = sensor->GetLatestTemperature(ht);
					if (err != TemperatureError::success || ht < BadLowTemperature || ht >= triggerTemperatures[1])
					{
						reqVal = maxVal;
					}
					else if (!bangBangMode && ht >= triggerTemperatures[0])
					{
						// We already know that ht < triggerTemperatures[1], therefore unless we have NaNs it is safe to divide by (triggerTemperatures[1] - triggerTemperatures[0])
						const float newVal = minVal + (maxVal - minVal) * (ht - triggerTemperatures[0])/(triggerTemperatures[1] - triggerTemperatures[0]);
						if (newVal > reqVal)
						{
							reqVal = newVal;
						}
					}
					else if (lastVal > 0.0 && ht + ThermostatHysteresis > triggerTemperatures[0])		// if the fan is on, add a hysteresis before turning it off
					{
						const float newVal = (bangBangMode) ? maxVal : minVal;
						if (newVal > reqVal)
						{
							reqVal = newVal;
						}
					}
				}
			}
		);
	}

	if (reqVal > 0.0)
	{
		if (lastVal <= 0.0)
		{
			// We are turning this fan on
			if (reqVal < 1.0 && blipTime != 0)
			{
				// Starting the fan from standstill, so blip the fan
				blipping = true;
				blipStartTime = millis();
			}
		}
		else if (blipping && millis() - blipStartTime >= blipTime)
		{
			blipping = false;
		}
	}
	else
	{
		blipping = false;
	}

	lastVal = reqVal;
	SetHardwarePwm((blipping) ? 1.0 : reqVal);
}

GCodeResult LocalFan::Refresh(const StringRef& reply) noexcept
{
	InternalRefresh(true);
	return GCodeResult::ok;
}

bool LocalFan::UpdateFanConfiguration(const StringRef& reply) noexcept
{
	InternalRefresh(true);
	return true;
}

// Update the fan, returning true if the fan is thermostatic and running.
// Checking the sensors is expensive, so only check them if checkSensors is true.
bool LocalFan::Check(bool checkSensors) noexcept
{
	InternalRefresh(checkSensors);
	return sensorsMonitored.IsNonEmpty() && lastVal > 0.0;
}

bool LocalFan::AssignPorts(const char *pinNames, const StringRef& reply) noexcept
{
	IoPort* const ports[] = { &port, &tachoPort };
	const PinAccess access1[] = { PinAccess::pwm, PinAccess::read};
	if (IoPort::AssignPorts(pinNames, reply, PinUsedBy::fan, 2, ports, access1) == 0)
	{
		const PinAccess access2[] = { PinAccess::write0, PinAccess::read};
		if (IoPort::AssignPorts(pinNames, reply, PinUsedBy::fan, 2, ports, access2) == 0)
		{
			return false;
		}
	}

	// Tacho initialisation
	if (tachoPort.IsValid())
	{
		tachoPort.AttachInterrupt(FanInterrupt, InterruptMode::falling, CallbackParameter(this));
	}

	InternalRefresh(true);
	return true;
}

// Tacho support
int32_t LocalFan::GetRPM() const noexcept
{
	// The ISR sets fanInterval to the number of step interrupt clocks it took to get fanMaxInterruptCount interrupts.
	// We get 2 tacho pulses per revolution, hence 2 interrupts per revolution.
	// When the fan stops, we get no interrupts and fanInterval stops getting updated. We must recognise this and return zero.
	return (!tachoPort.IsValid())
			? -1																			// we return -1 if there is no tacho configured
			: (fanInterval != 0 && StepTimer::GetTimerTicks() - fanLastResetTime < 3 * StepClockRate)	// if we have a reading and it is less than 3 seconds old
			  ? (StepClockRate * fanMaxInterruptCount * (60/2))/fanInterval					// then calculate RPM assuming 2 interrupts per rev
			  : 0;																			// else assume fan is off or tacho not connected
}

void LocalFan::Interrupt() noexcept
{
	++fanInterruptCount;
	if (fanInterruptCount == fanMaxInterruptCount)
	{
		const uint32_t now = StepTimer::GetTimerTicks();
		fanInterval = now - fanLastResetTime;
		fanLastResetTime = now;
		fanInterruptCount = 0;
	}
}

// End
