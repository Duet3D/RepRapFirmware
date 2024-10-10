#pragma once

#include <cstdint>

#include <ObjectModel/ObjectModel.h>

#include "StorageVolume.h"

#if SUPPORT_USB_DRIVE

class UsbVolume : public StorageVolume
{
public:
	UsbVolume(const char *id, uint8_t slot) : StorageVolume(id, slot) {}

	void Init() noexcept override;

	void Spin() noexcept override;

	GCodeResult Mount(const StringRef& reply, bool reportSuccess) noexcept override;

	bool IsUseable(const StringRef& reply) const noexcept override;
	bool IsMounted() const noexcept override
	{
		return (static_cast<uint8_t>(state) &
			(static_cast<uint8_t>(State::mounted) | static_cast<uint8_t>(State::removed)));
	}
	bool IsDetected() const noexcept override
	{
		return (static_cast<uint8_t>(state) &
			(static_cast<uint8_t>(State::inserted) | static_cast<uint8_t>(State::mounted)));
	}

	uint64_t GetCapacity() const noexcept override;
	uint32_t GetInterfaceSpeed() const noexcept override;

	DRESULT DiskInitialize() noexcept override;
	DRESULT DiskStatus() noexcept override;
	DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskIoctl(BYTE ctrl, void *buff) noexcept override;

	static void VolumeInserted(uint8_t address);
	static void VolumeRemoved(uint8_t address);

private:
	enum class State : uint8_t
	{
		free = 0x00,
		inserted = 0x01,
		mounted = 0x02,
		removed = 0x04
	};

	uint8_t address;
	uint8_t lun;
	BinarySemaphore ioDone;

	// The state is read and/or modified in two tasks: UsbTask during tinyUSB device insertion/removal callbacks,
	// and MainTask, during GCode mounting/unmounting commands.
	State state;

	static UsbVolume* usbDrives[NumUsbDrives];

	void DeviceUnmount() noexcept override;

	bool Accept(uint8_t address);
	void Free();
};

#endif // SUPPORT_USB_DRIVE
