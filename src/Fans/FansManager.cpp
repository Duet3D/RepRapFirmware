/*
 * FansManager.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "FansManager.h"

#include "LocalFan.h"
#include "RemoteFan.h"
#include <RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
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
	ReadLocker locker(fansLock);
	return ReadLockedPointer<Fan>(locker, (fanNum < ARRAY_SIZE(fans)) ? fans[fanNum] : nullptr);
}

// Create and return a local fan. if it fails, return nullptr with the error message in 'reply'.
LocalFan *FansManager::CreateLocalFan(uint32_t fanNum, const char *pinNames, PwmFrequency freq, const StringRef& reply) noexcept
{
	LocalFan *newFan = new LocalFan(fanNum);
	if (!newFan->AssignPorts(pinNames, reply))
	{
		delete newFan;
		return nullptr;
	}
	(void)newFan->SetPwmFrequency(freq, reply);
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

#if HAS_MASS_STORAGE

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
	const bool seenPin = gb.Seen('C');
	if (seenPin)
	{
		String<StringLength50> pinName;
		gb.GetReducedString(pinName.GetRef());

		WriteLocker lock(fansLock);

		Fan *oldFan = nullptr;
		std::swap<Fan*>(oldFan, fans[fanNum]);
		delete oldFan;

		const PwmFrequency freq = (gb.Seen('Q')) ? gb.GetPwmFrequency() : DefaultFanPwmFreq;

#if SUPPORT_CAN_EXPANSION
		const CanAddress board = IoPort::RemoveBoardAddress(pinName.GetRef());
		if (board != CanId::MasterAddress)
		{
			auto *newFan = new RemoteFan(fanNum, board);
			const GCodeResult rslt = newFan->ConfigurePort(pinName.c_str(), freq, reply);
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
		fans[fanNum] = CreateLocalFan(fanNum, pinName.c_str(), freq, reply);
		reprap.FansUpdated();
		return (fans[fanNum] == nullptr) ? GCodeResult::error : GCodeResult::ok;
	}

	const auto fan = FindFan(fanNum);
	if (fan.IsNull())
	{
		reply.printf("Fan %u does not exist", (unsigned int)fanNum);
		return GCodeResult::error;
	}

	if (gb.Seen('Q'))
	{
		const GCodeResult rslt = fan->SetPwmFrequency(gb.GetPwmFrequency(), reply);
		reprap.FansUpdated();
		return rslt;
	}

	return fan->ReportPortDetails(reply);
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

void FansManager::SetFanValue(size_t fanNum, float speed) noexcept
{
	String<1> dummy;
	(void)SetFanValue(fanNum, speed, dummy.GetRef());
}

// Check if the given fan can be controlled manually so that DWC can decide whether or not to show the corresponding fan
// controls. This is the case if no thermostatic control is enabled and if the fan was configured at least once before.
bool FansManager::IsFanControllable(size_t fanNum) const noexcept
{
	auto fan = FindFan(fanNum);
	return fan.IsNotNull() && !fan->HasMonitoredSensors();
}

// Return the fan's name
const char *FansManager::GetFanName(size_t fanNum) const noexcept
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
		fans[i] = CreateLocalFan(i, DefaultFanPinNames[i], i < ARRAY_SIZE(DefaultFanPwmFrequencies) ? DefaultFanPwmFrequencies[i] : DefaultFanPwmFreq, dummy.GetRef());
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

// End
