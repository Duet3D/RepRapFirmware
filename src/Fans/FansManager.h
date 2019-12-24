/*
 * FansManager.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_FANSMANAGER_H_
#define SRC_FANS_FANSMANAGER_H_

#include "RepRapFirmware.h"
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
	void Init() noexcept;
	bool CheckFans() noexcept;
	size_t GetHighestUsedFanNumber() const noexcept;
	GCodeResult ConfigureFanPort(uint32_t fanNum, GCodeBuffer& gb, const StringRef& reply);
	bool ConfigureFan(unsigned int mcode, size_t fanNum, GCodeBuffer& gb, const StringRef& reply, bool& error);
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

private:
	ReadLockedPointer<Fan> FindFan(size_t fanNum) const noexcept;
	LocalFan *CreateLocalFan(uint32_t fanNum, const char *pinNames, PwmFrequency freq, const StringRef& reply) noexcept;

	mutable ReadWriteLock fansLock;
	Fan *fans[MaxFans];
};

#endif /* SRC_FANS_FANSMANAGER_H_ */
