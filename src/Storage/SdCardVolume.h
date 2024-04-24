#pragma once

#include <ObjectModel/ObjectModel.h>
#include "StorageVolume.h"

class SdCardVolume : public StorageVolume
{
public:

	struct Stats
	{
		uint32_t maxReadTime;
		uint32_t maxWriteTime;
		uint32_t maxRetryCount;
	};

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	SdCardVolume(const char *id, uint8_t slot) : StorageVolume(id, slot) {}

	void Init() noexcept override;

	void Spin() noexcept override;

	GCodeResult Mount(const StringRef& reply, bool reportSuccess) noexcept override;

	bool IsUseable(const StringRef& reply) const noexcept override;
	bool IsMounted() const noexcept override { return isMounted; }
	bool IsDetected() const noexcept override { return cardState == CardDetectState::present; }

	uint64_t GetCapacity() const noexcept override;
	uint32_t GetInterfaceSpeed() const noexcept override;

	DRESULT DiskInitialize() noexcept override;
	DRESULT DiskStatus() noexcept override;
	DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskIoctl(BYTE ctrl, void *buff) noexcept override;

# ifdef DUET3_MB6HC
	// Configure additional SD card slots
	// The card detect pin may be NoPin if the SD card slot doesn't support card detect
	static GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
# endif

	static Stats GetStats() noexcept;
	static void ResetStats() noexcept;

	static void SdmmcInit() noexcept;

private:
	enum class CardDetectState : uint8_t
	{
		notPresent = 0,
		inserting,
		present,
		removing
	};

	static SdCardVolume* sdCards[NumSdCards];

	bool mounting;
	bool isMounted;
	uint32_t mountStartTime;
	uint32_t cdChangedTime;
	CardDetectState cardState;
	Pin cdPin;

	static Stats stats;

	void DeviceUnmount() noexcept override;
};
