/*
 * FansManager.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_FANSMANAGER_H_
#define SRC_FANS_FANSMANAGER_H_

#include <RepRapFirmware.h>
#include "Fan.h"
#include <RTOSIface/RTOSIface.h>

#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
struct CanMessageFansReport;
#endif

class GCodeBuffer;
class LocalFan;

class FansManager
{
public:
	FansManager() noexcept;
	FansManager(const FansManager&) = delete;

	void Init() noexcept;
	void Exit() noexcept;
	bool CheckFans(bool checkSensors) noexcept;
	GCodeResult ConfigureFanPort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	bool ConfigureFan(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException);
	float GetFanValue(size_t fanNum) const noexcept;
	GCodeResult SetFanValue(size_t fanNum, float speed, const StringRef& reply) noexcept;
	float SetFanValue(size_t fanNum, float speed) noexcept;
	float SetFansValue(FansBitmap whichFans, float speed) noexcept;
	bool IsFanControllable(size_t fanNum) const noexcept;
	const char *GetFanName(size_t fanNum) const noexcept;
	int32_t GetFanRPM(size_t fanNum) const noexcept;
#if SUPPORT_CAN_EXPANSION
	void ProcessRemoteFanRpms(CanAddress src, const CanMessageFansReport& msg) noexcept;
#endif
#if SUPPORT_REMOTE_COMMANDS
	GCodeResult ConfigureFanPort(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ConfigureFan(const CanMessageFanParameters& gb, const StringRef& reply) noexcept;
	GCodeResult SetFanSpeed(const CanMessageSetFanSpeed& msg, const StringRef& reply) noexcept;
	unsigned int PopulateFansReport(CanMessageFansReport& msg) noexcept;
#endif
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteFanSettings(FileStore *f) const noexcept;
#endif

	// These need to be accessed by the OMT in class RepRap
	size_t GetNumFansToReport() const noexcept;
	ReadLockedPointer<Fan> FindFan(size_t fanNum) const noexcept;

	static ReadWriteLock fansLock;

private:
	LocalFan *CreateLocalFan(uint32_t fanNum, const char *pinNames, PwmFrequency freq, const StringRef& reply) noexcept;

	Fan *fans[MaxFans];
};

#endif /* SRC_FANS_FANSMANAGER_H_ */
