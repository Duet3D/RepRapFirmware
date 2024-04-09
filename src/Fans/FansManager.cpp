/*
 * FansManager.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "FansManager.h"

#include "LocalFan.h"
#include "RemoteFan.h"
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
#endif

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericTables.h>
# include <CanMessageGenericParser.h>
#endif

#include <utility>

ReadWriteLock FansManager::fansLock;

FansManager::FansManager() noexcept
{
	for (Fan*& f : fans)
	{
		f = nullptr;
	}
}

// Retrieve the pointer to a fan, or nullptr if it doesn't exist.
// Lock the fan system before calling this, so that the fan can't be deleted while we are accessing it.
ReadLockedPointer<Fan> FansManager::FindFan(size_t fanNum) const noexcept
{
	return ReadLockedPointer<Fan>(fansLock, (fanNum < ARRAY_SIZE(fans)) ? fans[fanNum] : nullptr);
}

// Create and return a local fan. if it fails, return nullptr with the error message in 'reply'.
LocalFan *FansManager::CreateLocalFan(uint32_t fanNum, const char *_ecv_array pinNames, PwmFrequency freq, float pulsesPerRev, const StringRef& reply) noexcept
{
	LocalFan *newFan = new LocalFan(fanNum);
	if (!newFan->AssignPorts(pinNames, reply))
	{
		delete newFan;
		return nullptr;
	}
	(void)newFan->SetFanParameters(true, freq, true, pulsesPerRev, reply);
	return newFan;
}

// Check and if necessary update all fans. Return true if a thermostatic fan is running.
bool FansManager::CheckFans(bool checkSensors) noexcept
{
	ReadLocker lock(fansLock);
	bool thermostaticFanRunning = false;
	for (Fan* fan : fans)
	{
		if (fan != nullptr && fan->Check(checkSensors))
		{
			thermostaticFanRunning = true;
		}
	}
	return thermostaticFanRunning;
}

// Return the number of fans to report on. Used by RepRap.cpp to shorten responses by omitting unused trailing fan numbers.
size_t FansManager::GetNumFansToReport() const noexcept
{
	size_t numFans = ARRAY_SIZE(fans);
	while (numFans != 0 && fans[numFans - 1] == nullptr)
	{
		--numFans;
	}
	return numFans;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

bool FansManager::WriteFanSettings(FileStore *f) const noexcept
{
	ReadLocker lock(fansLock);
	bool ok = true;
	for (size_t fanNum = 0; ok && fanNum < MaxFans; ++fanNum)
	{
		ok = fans[fanNum] == nullptr || fans[fanNum]->WriteSettings(f, fanNum);
	}
	return ok;
}

#endif

// This is called by M950 to create a fan or change its PWM frequency
GCodeResult FansManager::ConfigureFanPort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const uint32_t fanNum = gb.GetLimitedUIValue('F', MaxFans);

	const bool seenFreq = gb.Seen('Q');
	const PwmFrequency newFreq = (seenFreq) ? gb.GetPwmFrequency() : DefaultFanPwmFreq;

	const bool seenPpr = gb.Seen('K');
	const float newPpr = (seenPpr) ? gb.GetLimitedFValue('K', MinFanPulsesPerRev, MaxFanPulsesPerRev) : DefaultFanTachoPulsesPerRev;

	const bool seenPin = gb.Seen('C');
	if (seenPin)
	{
		String<StringLength50> pinName;
		gb.GetReducedString(pinName.GetRef());

		WriteLocker lock(fansLock);
		DeleteObject(fans[fanNum]);

#if SUPPORT_CAN_EXPANSION
		const CanAddress board = IoPort::RemoveBoardAddress(pinName.GetRef());
#endif
		if (StringEqualsIgnoreCase(pinName.c_str(), NoPinName))
		{
			return GCodeResult::ok;				// just deleting an existing fan
		}

#if SUPPORT_CAN_EXPANSION
		if (board != CanInterface::GetCanAddress())
		{
			auto *newFan = new RemoteFan(fanNum, board);
			const GCodeResult rslt = newFan->ConfigurePort(pinName.c_str(), newFreq, newPpr, reply);
			if (rslt == GCodeResult::ok)
			{
				fans[fanNum] = newFan;
			}
			else
			{
				delete newFan;
			}
			return rslt;
		}
#endif
		fans[fanNum] = CreateLocalFan(fanNum, pinName.c_str(), newFreq, newPpr, reply);
		reprap.FansUpdated();
		return (fans[fanNum] == nullptr) ? GCodeResult::error : GCodeResult::ok;
	}

	const auto fan = FindFan(fanNum);
	if (fan.IsNull())
	{
		reply.printf("Fan %u not found", (unsigned int)fanNum);
		return GCodeResult::error;
	}

	// If we get here we're setting the PWM frequency or tacho pulses/rev, or neither
	return (seenFreq || seenPpr)
			? fan->SetFanParameters(seenFreq, newFreq, seenPpr, newPpr, reply)
				: fan->ReportPortDetails(reply);
}

// Set or report the parameters for the specified fan
// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 106 or 107)
// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
bool FansManager::ConfigureFan(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException)
{
	auto fan = FindFan(fanNum);
	if (fan.IsNull())
	{
		reply.printf("Fan number %u not found", fanNum);
		error = true;
		return false;
	}

	return fan->Configure(mcode, fanNum, gb, reply, error);
}

float FansManager::GetFanValue(size_t fanNum) const noexcept
{
	auto fan = FindFan(fanNum);
	return (fan.IsNull()) ? -1 : fan->GetConfiguredPwm();
}

GCodeResult FansManager::SetFanValue(size_t fanNum, float speed, const StringRef& reply) noexcept
{
	auto fan = FindFan(fanNum);
	if (fan.IsNotNull())
	{
		return fan->SetPwm(speed, reply);
	}
	reply.printf("Fan number %u not found", fanNum);
	return GCodeResult::error;
}

// Update the PWM of the specified fan, returning the PWM change
float FansManager::SetFanValue(size_t fanNum, float speed) noexcept
{
	auto fan = FindFan(fanNum);
	if (fan.IsNotNull())
	{
		const float oldPwm = fan->GetPwm();
		String<1> dummy;
		(void)fan->SetPwm(speed, dummy.GetRef());
		return fan->GetPwm() - oldPwm;
	}
	return 0.0;
}

// Update the PWM of the specified fans, returning the total PWM change divided by the number of fans
float FansManager::SetFansValue(FansBitmap whichFans, float speed) noexcept
{
	float pwmChange = 0;
	if (!whichFans.IsEmpty())
	{
		whichFans.Iterate([speed, this, &pwmChange](unsigned int i, unsigned int) noexcept { pwmChange += SetFanValue(i, speed); });
		pwmChange /= whichFans.CountSetBits();
	}
	return pwmChange;
}

// Check if the given fan can be controlled manually so that DWC can decide whether or not to show the corresponding fan
// controls. This is the case if no thermostatic control is enabled and if the fan was configured at least once before.
bool FansManager::IsFanControllable(size_t fanNum) const noexcept
{
	auto fan = FindFan(fanNum);
	return fan.IsNotNull() && !fan->HasMonitoredSensors();
}

// Return the fan's name
const char *_ecv_array FansManager::GetFanName(size_t fanNum) const noexcept
{
	auto fan = FindFan(fanNum);
	return (fan.IsNull()) ? "" : fan->GetName();
}

// Get current fan RPM, or -1 if the fan is invalid or doesn't have a tacho pin
int32_t FansManager::GetFanRPM(size_t fanNum) const noexcept
{
	auto fan = FindFan(fanNum);
	return (fan.IsNull()) ? -1 : fan->GetRPM();
}

// Initialise fans. Call this only once, and only during initialisation.
void FansManager::Init() noexcept
{
#if ALLOCATE_DEFAULT_PORTS
	for (size_t i = 0; i < ARRAY_SIZE(DefaultFanPinNames); ++i)
	{
		String<1> dummy;
		fans[i] = CreateLocalFan(i,
									DefaultFanPinNames[i],
									(i < ARRAY_SIZE(DefaultFanPwmFrequencies) && DefaultFanPwmFrequencies[i] != 0) ? DefaultFanPwmFrequencies[i] : DefaultFanPwmFreq,
									DefaultFanTachoPulsesPerRev,
									dummy.GetRef()
								);
	}

# if defined(PCCB)
	// Fan 3 needs to be set explicitly to zero PWM, otherwise it turns on because the MCU output pin isn't set low
	if (fans[3] != nullptr)
	{
		String<1> dummy;
		(void)fans[3]->SetPwm(0.0, dummy.GetRef());
	}
# endif
#endif
}

// Shut down the fans system, in particular stop any interrupts into Fan objects. Called before loading IAP into the last 64K of RAM.
// The simplest way is to delete all the fans.
void FansManager::Exit() noexcept
{
	WriteLocker lock(fansLock);
	for (Fan*& fan : fans)
	{
		DeleteObject(fan);
	}
}

#if SUPPORT_CAN_EXPANSION

void FansManager::ProcessRemoteFanRpms(CanAddress src, const CanMessageFansReport& msg) noexcept
{
	size_t numFansProcessed = 0;
	uint64_t whichFans = msg.whichFans;
	while (whichFans != 0)
	{
		const unsigned int fanNum = LowestSetBit(whichFans);
		auto fan = FindFan(fanNum);
		if (fan.IsNotNull())
		{
			fan->UpdateFromRemote(src, msg.fanReports[numFansProcessed]);
		}
		++numFansProcessed;
		whichFans &= ~((uint64_t)1 << fanNum);
	}
}

#endif

#if SUPPORT_REMOTE_COMMANDS

// This is called by M950 to create a fan or change its PWM frequency or report its port
GCodeResult FansManager::ConfigureFanPort(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M950FanParams);
	uint16_t fanNum;
	if (!parser.GetUintParam('F', fanNum))
	{
		reply.copy("Missing F parameter");
		return GCodeResult::error;
	}

	if (fanNum >= MaxFans)
	{
		reply.printf("Fan number %u too high", (unsigned int)fanNum);
		return GCodeResult::error;
	}

	PwmFrequency freq = DefaultFanPwmFreq;
	const bool seenFreq = parser.GetUintParam('Q', freq);
	float pulsesPerRev = DefaultFanTachoPulsesPerRev;
	const bool seenPpr = parser.GetFloatParam('K', pulsesPerRev);

	String<StringLength50> pinNames;
	if (parser.GetStringParam('C', pinNames.GetRef()))
	{
		WriteLocker lock(fansLock);

		Fan *oldFan = nullptr;
		std::swap(oldFan, fans[fanNum]);
		delete oldFan;

		fans[fanNum] = CreateLocalFan(fanNum, pinNames.c_str(), freq, pulsesPerRev, reply);
		return (fans[fanNum] == nullptr) ? GCodeResult::error : GCodeResult::ok;
	}

	const auto fan = FindFan(fanNum);
	if (fan.IsNull())
	{
		reply.printf("Board %u doesn't have fan %u", CanInterface::GetCanAddress(), fanNum);
		return GCodeResult::error;
	}

	return (seenFreq || seenPpr)
			? fan->SetFanParameters(seenFreq, freq, seenPpr, pulsesPerRev, reply)
				: fan->ReportPortDetails(reply);
}

// Set or report the parameters for the specified fan
// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 106 or 107)
// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
GCodeResult FansManager::ConfigureFan(const CanMessageFanParameters& msg, const StringRef& reply) noexcept
{
	auto fan = FindFan(msg.fanNumber);
	if (fan.IsNull())
	{
		reply.printf("Board %u doesn't have fan %u", CanInterface::GetCanAddress(), msg.fanNumber);
		return GCodeResult::error;
	}

	return fan->Configure(msg, reply);
}

GCodeResult FansManager::SetFanSpeed(const CanMessageSetFanSpeed& msg, const StringRef& reply) noexcept
{
	auto fan = FindFan(msg.fanNumber);
	if (fan.IsNull())
	{
		reply.printf("Board %u doesn't have fan %u", CanInterface::GetCanAddress(), msg.fanNumber);
		return GCodeResult::error;
	}

	return fan->SetPwm(msg.pwm, reply);
}

// Construct a fan RPM report message. Returns the number of fans reported in it.
unsigned int FansManager::PopulateFansReport(CanMessageFansReport& msg) noexcept
{
	ReadLocker locker(fansLock);

	msg.whichFans = 0;
	unsigned int numReported = 0;
	for (Fan* f : fans)
	{
		if (f != nullptr && f->IsLocal())
		{
			msg.fanReports[numReported].actualPwm = (uint16_t)(f->GetPwm() * 65535);
			msg.fanReports[numReported].rpm = f->GetRPM();
			msg.whichFans |= (uint64_t)1 << f->GetNumber();
			++numReported;
		}
	}
	return numReported;
}

#endif

// End
