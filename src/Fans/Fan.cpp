/*
 * Fan.cpp
 *
 *  Created on: 29 Jun 2016
 *      Author: David
 */

#include "Fan.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "Heating/Heat.h"
#include "Movement/StepTimer.h"

void FanInterrupt(CallbackParameter cb)
{
	static_cast<Fan *>(cb.vp)->Interrupt();
}

Fan::Fan() : fanInterruptCount(0), fanLastResetTime(0), fanInterval(0)
{
}

bool Fan::AssignPorts(GCodeBuffer& gb, const StringRef& reply)
{
	IoPort* const ports[] = { &port, &tachoPort };
	PinAccess access[] = { PinAccess::pwm, PinAccess::readWithPullup };
	if (IoPort::AssignPorts(gb, reply, PinUsedBy::fan, 2, ports, access) == 0)
	{
		access[0] = PinAccess::write0;					// if it isn't a PWM port, an on/off port will do
		gb.Seen('C');									// re-prime the search
		if (IoPort::AssignPorts(gb, reply, PinUsedBy::fan, 2, ports, access) == 0)
		{
			return false;
		}
	}

	// Tacho initialisation
	if (tachoPort.IsValid())
	{
		tachoPort.AttachInterrupt(FanInterrupt, INTERRUPT_MODE_FALLING, this);
	}

	Refresh();
	return true;
}

bool Fan::AssignPorts(const char *pinNames, const StringRef& reply)
{
	IoPort* const ports[] = { &port, &tachoPort };
	const PinAccess access1[] = { PinAccess::pwm, PinAccess::readWithPullup };
	if (IoPort::AssignPorts(pinNames, reply, PinUsedBy::fan, 2, ports, access1) == 0)
	{
		const PinAccess access2[] = { PinAccess::write0, PinAccess::readWithPullup };
		if (IoPort::AssignPorts(pinNames, reply, PinUsedBy::fan, 2, ports, access2) == 0)
		{
			return false;
		}
	}

	// Tacho initialisation
	if (tachoPort.IsValid())
	{
		tachoPort.AttachInterrupt(FanInterrupt, INTERRUPT_MODE_FALLING, this);
	}

	Refresh();
	return true;
}

void Fan::Init()
{
	// Fan output initialisation
	isConfigured = false;
	val = lastVal = 0.0;
	minVal = 0.1;				// 10% minimum fan speed
	maxVal = 1.0;				// 100% maximum fan speed
	blipTime = 100;				// 100ms fan blip
	blipping = false;
	heatersMonitored = 0;
	triggerTemperatures[0] = triggerTemperatures[1] = HotEndFanTemperature;
	lastPwm = -1.0;				// force a refresh
	Refresh();
}

// Set or report the parameters for this fan
// If 'mcode' is an M-code used to set parameters for the f (which should only ever be 106)
// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
// Exceptions:
// 1. Only process the S parameter if other values were processed.
// 2. Don't process the R parameter, but if it is present don't print the existing configuration.
bool Fan::Configure(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error)
{
	bool seen = false;
	if (mcode == 106)
	{
		// Check that the fan is enabled
		if (!IsEnabled())
		{
			reply.printf("Fan %u is disabled", fanNum);
			error = true;
			return true;											// say we have processed it
		}

		if (gb.Seen('T'))
		{
			seen = true;
			size_t numTemps = 2;
			gb.GetFloatArray(triggerTemperatures, numTemps, true);
		}

		if (gb.Seen('B'))										// Set blip time
		{
			seen = true;
			blipTime = (uint32_t)(max<float>(gb.GetFValue(), 0.0) * SecondsToMillis);
		}

		if (gb.Seen('L'))		// Set minimum speed
		{
			seen = true;
			float speed = gb.GetFValue();
			if (speed > 1.0)
			{
				speed /= 255.0;
			}
			minVal = constrain<float>(speed, 0.0, maxVal);
		}

		if (gb.Seen('X'))		// Set maximum speed
		{
			seen = true;
			float speed = gb.GetFValue();
			if (speed > 1.0)
			{
				speed /= 255.0;
			}
			maxVal = constrain<float>(speed, minVal, 1.0);
		}

		if (gb.Seen('H'))		// Set thermostatically-controlled heaters
		{
			seen = true;
			int32_t heaters[NumTotalHeaters + MaxVirtualHeaters];		// signed because we use H-1 to disable thermostatic mode
			size_t numH = ARRAY_SIZE(heaters);
			gb.GetIntArray(heaters, numH, false);

			// Note that M106 H-1 disables thermostatic mode. The following code implements that automatically.
			heatersMonitored = 0;
			for (size_t h = 0; h < numH; ++h)
			{
				const int hnum = heaters[h];
				if (hnum >= 0 && hnum < (int)NumTotalHeaters)
				{
					SetBit(heatersMonitored, (unsigned int)hnum);
				}
				else if (hnum >= (int)FirstVirtualHeater && hnum < (int)(FirstVirtualHeater + MaxVirtualHeaters))
				{
					// Heaters 100, 101... are virtual heaters i.e. CPU and driver temperatures
					SetBit(heatersMonitored, NumTotalHeaters + (unsigned int)hnum - FirstVirtualHeater);
				}
			}
			if (heatersMonitored != 0)
			{
				SetPwm(1.0);			// default the fan speed to full for safety
			}
		}

		if (gb.Seen('C') && gb.GetQuotedString(name.GetRef()))
		{
			seen = true;
		}

		// We only act on the 'S' parameter here if we have processed other parameters
		if (seen && gb.Seen('S'))		// Set new fan value - process this after processing 'H' or it may not be acted on
		{
			SetPwm(gb.GetPwmValue());
		}

		if (seen)
		{
			isConfigured = true;
			Refresh();
		}
		else if (!gb.Seen('R') && !gb.Seen('S'))
		{
			// Report the configuration of the specified fan
			reply.printf("Fan %u", fanNum);
			if (name.strlen() != 0)
			{
				reply.catf(" (%s)", name.c_str());
			}
			reply.catf(", speed: %d%%, min: %d%%, max: %d%%, blip: %.2f",
						(int)(val * 100.0),
						(int)(minVal * 100.0),
						(int)(maxVal * 100.0),
						(double)(blipTime * MillisToSeconds)
					  );
			if (heatersMonitored != 0)
			{
				reply.catf(", temperature: %.1f:%.1fC, heaters:", (double)triggerTemperatures[0], (double)triggerTemperatures[1]);
				for (unsigned int i = 0; i < NumTotalHeaters + MaxVirtualHeaters; ++i)
				{
					if (IsBitSet(heatersMonitored, i))
					{
						reply.catf(" %u", (i < NumTotalHeaters) ? i : FirstVirtualHeater + i - NumTotalHeaters);
					}
				}
				reply.catf(", current speed: %d%%:", (int)(lastVal * 100.0));
			}
		}
	}

	return seen;
}

// Set the PWM. 'speed' is in the interval 0.0..1.0.
void Fan::SetPwm(float speed)
{
	val = speed;
	Refresh();
}

// Set the hardware PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
void Fan::SetHardwarePwm(float pwmVal)
{
	// Only set the PWM if it has changed, to avoid a lot of I2C traffic when we have a DueX5 connected
	if (pwmVal != lastPwm)
	{
		lastPwm = pwmVal;
		port.WriteAnalog(pwmVal);
	}
}

void Fan::SetHeatersMonitored(HeatersMonitoredBitmap h)
{
	heatersMonitored = h;
	Refresh();
}

// Refresh the fan PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
void Fan::Refresh()
{
	float reqVal;
#if HAS_SMART_DRIVERS
	uint32_t driverChannelsMonitored = 0;
#endif

	if (heatersMonitored == 0)
	{
		reqVal = val;
	}
	else
	{
		reqVal = 0.0;
		const bool bangBangMode = (triggerTemperatures[1] <= triggerTemperatures[0]);
		for (size_t h = 0; h < NumTotalHeaters + MaxVirtualHeaters; ++h)
		{
			// Check if this heater is both monitored by this fan and in use
			if (   IsBitSet(heatersMonitored, h)
				&& (h < reprap.GetToolHeatersInUse() || (h >= NumTotalHeaters && h < NumTotalHeaters + MaxVirtualHeaters) || reprap.GetHeat().IsBedOrChamberHeater(h))
			   )
			{
				// This heater is both monitored and potentially active
				if (h < NumTotalHeaters && reprap.GetHeat().IsTuning(h))
				{
					reqVal = 1.0;			// when turning the PID for a monitored heater, turn the fan on
				}
				else
				{
					const size_t heaterHumber = (h >= NumTotalHeaters) ? (h - NumTotalHeaters) + FirstVirtualHeater : h;
					TemperatureError err;
					const float ht = reprap.GetHeat().GetTemperature(heaterHumber, err);
					if (err != TemperatureError::success || ht < BadLowTemperature || ht >= triggerTemperatures[1])
					{
						reqVal = max<float>(reqVal, (bangBangMode) ? max<float>(0.5, val) : 1.0);
					}
					else if (!bangBangMode && ht > triggerTemperatures[0])
					{
						// We already know that ht < triggerTemperatures[1], therefore unless we have NaNs it is safe to divide by (triggerTemperatures[1] - triggerTemperatures[0])
						reqVal = max<float>(reqVal, (ht - triggerTemperatures[0])/(triggerTemperatures[1] - triggerTemperatures[0]));
					}
					else if (lastVal != 0.0 && ht + ThermostatHysteresis > triggerTemperatures[0])		// if the fan is on, add a hysteresis before turning it off
					{
						const float minFanSpeed = (bangBangMode) ? max<float>(0.5, val) : minVal;
						reqVal = constrain<float>(reqVal, minFanSpeed, maxVal);
					}
#if HAS_SMART_DRIVERS
					const unsigned int channel = reprap.GetHeat().GetHeaterChannel(heaterHumber);
					if (channel >= FirstTmcDriversSenseChannel && channel < FirstTmcDriversSenseChannel + NumTmcDriversSenseChannels)
					{
						driverChannelsMonitored |= 1 << (channel - FirstTmcDriversSenseChannel);
					}
#endif
				}
			}
		}
	}

	if (reqVal > 0.0)
	{
		reqVal = max<float>(reqVal * maxVal, minVal);		// scale the requested PWM by the maximum, enforce the minimum
		if (lastVal == 0.0)
		{
			// We are turning this fan on
#if HAS_SMART_DRIVERS
			if (driverChannelsMonitored != 0)
			{
				reprap.GetPlatform().DriverCoolingFansOnOff(driverChannelsMonitored, true);		// tell Platform that we have started a fan that cools drivers
			}
#endif
			if (reqVal < 1.0 && blipTime != 0)
			{
				// Starting the fan from standstill, so blip the fan
				blipping = true;
				blipStartTime = millis();
			}
		}

		if (blipping)
		{
			if (millis() - blipStartTime < blipTime)
			{
				reqVal = 1.0;
			}
			else
			{
				blipping = false;
			}
		}
	}
#if HAS_SMART_DRIVERS
	else if (driverChannelsMonitored != 0 && lastVal != 0.0)
	{
		reprap.GetPlatform().DriverCoolingFansOnOff(driverChannelsMonitored, false);	// tell Platform that we have stopped a fan that cools drivers
	}
#endif

	SetHardwarePwm(reqVal);
	lastVal = reqVal;
}

bool Fan::Check()
{
	if (heatersMonitored != 0 || blipping)
	{
		Refresh();
	}
	return heatersMonitored != 0 && lastVal != 0.0;
}

void Fan::Disable()
{
	lastPwm = -1;
	SetHardwarePwm(0.0);
	port.Release();
}

// Save the settings of this fan if it isn't thermostatic
bool Fan::WriteSettings(FileStore *f, size_t fanNum) const
{
	if (heatersMonitored == 0)
	{
		String<StringLength20> fanCommand;
		fanCommand.printf("M106 P%u S%.2f\n", fanNum, (double)val);
		return f->Write(fanCommand.c_str());
	}
	return true;
}

// Tacho support
int32_t Fan::GetRPM() const
{
	// The ISR sets fanInterval to the number of step interrupt clocks it took to get fanMaxInterruptCount interrupts.
	// We get 2 tacho pulses per revolution, hence 2 interrupts per revolution.
	// When the fan stops, we get no interrupts and fanInterval stops getting updated. We must recognise this and return zero.
	return (!tachoPort.IsValid())
			? -1																			// we return -1 if there is no tacho configured
			: (fanInterval != 0 && StepTimer::GetInterruptClocks() - fanLastResetTime < 3 * StepTimer::StepClockRate)	// if we have a reading and it is less than 3 seconds old
			  ? (StepTimer::StepClockRate * fanMaxInterruptCount * (60/2))/fanInterval		// then calculate RPM assuming 2 interrupts per rev
			  : 0;																			// else assume fan is off or tacho not connected
}

void Fan::Interrupt()
{
	++fanInterruptCount;
	if (fanInterruptCount == fanMaxInterruptCount)
	{
		const uint32_t now = StepTimer::GetInterruptClocks();
		fanInterval = now - fanLastResetTime;
		fanLastResetTime = now;
		fanInterruptCount = 0;
	}
}

// End
