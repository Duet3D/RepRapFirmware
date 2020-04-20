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
#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
struct CanMessageFanRpms;
#endif

class GCodeBuffer;
class LocalFan;

class FansManager
{
public:
	FansManager() noexcept;
	FansManager(const FansManager&) = delete;

	void Init() noexcept;
	bool CheckFans(bool checkSensors) noexcept;
	GCodeResult ConfigureFanPort(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	bool ConfigureFan(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException);
	float GetFanValue(size_t fanNum) const noexcept;
	GCodeResult SetFanValue(size_t fanNum, float speed, const StringRef& reply) noexcept;
	void SetFanValue(size_t fanNum, float speed) noexcept;
	bool IsFanControllable(size_t fanNum) const noexcept;
	const char *GetFanName(size_t fanNum) const noexcept;
	int32_t GetFanRPM(size_t fanNum) const noexcept;
#if SUPPORT_CAN_EXPANSION
	void ProcessRemoteFanRpms(CanAddress src, const CanMessageFanRpms& msg) noexcept;
#endif
#if HAS_MASS_STORAGE
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
