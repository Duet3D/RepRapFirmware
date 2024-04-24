
#include <cstdint>

#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#include <TinyUsbInterface.h>

#if SUPPORT_USB_DRIVE

static_assert(CORE_USES_TINYUSB && CFG_TUH_ENABLED, "USB drive support needs tinyUSB host stack"); // implementation only on tinyUSB with host support
#if CORE_USES_TINYUSB && CFG_TUH_ENABLED

#include <tusb.h>
#include <class/msc/msc_host.h>

#include "UsbVolume.h"

static bool disk_io_complete(uint8_t address, tuh_msc_complete_data_t const *cb_data)
{
	(void) address;
	BinarySemaphore *ioDone = reinterpret_cast<BinarySemaphore*>(cb_data->user_arg);
	ioDone->Give();
	return true;
}

void UsbVolume::Init() noexcept
{
	StorageVolume::Init();
	address = 0;

	for (size_t i = 0; i < NumUsbDrives; i++)
	{
		if (usbDrives[i] == nullptr)
		{
			usbDrives[i] = this;
			break;
		}
	}
}

void UsbVolume::Spin() noexcept
{
	if (state == State::removed)
	{
		InternalUnmount();
		address = 0;
		state = State::free;
	}
}

bool UsbVolume::IsUseable(const StringRef& reply) const noexcept
{
	if (!CoreUsbIsHostMode())
	{
		if (&reply != &StorageVolume::noReply)
		{
			reply.copy("USB not configured as host");
		}
		return false;
	}
	return true;
}

GCodeResult UsbVolume::Mount(const StringRef &reply, bool reportSuccess) noexcept
{
	if (!IsDetected())
	{
		reply.copy("No USB storage detected");
		return GCodeResult::error;
	}

	if (IsMounted())
	{
		if (MassStorage::AnyFileOpen(&fileSystem))
		{
			// Don't re-mount the card if any files are open on it
			reply.printf("%s has open file(s)", id);
			return GCodeResult::error;
		}
		(void)InternalUnmount();
	}

	// Mount the file systems
	const FRESULT res = f_mount(&fileSystem, path, 1);
	if (res == FR_NO_FILESYSTEM)
	{
		reply.printf("Cannot mount %s: no FAT filesystem found on card (EXFAT is not supported)", id);
		return GCodeResult::error;
	}
	if (res != FR_OK)
	{
		reply.printf("Cannot mount %s: code %d", id, res);
		return GCodeResult::error;
	}
	state = State::mounted;

	reprap.VolumesUpdated();
	if (reportSuccess)
	{
		float capacity = GetCapacity() / 1000000.0f; // get capacity and convert from Kib to Mbytes
		const char* capUnits = capacity >= 1000.0 ? "Gb" : "Mb";
		reply.printf("%s mounted, capacity %.2f%s", id, static_cast<double>(capacity >= 1000.0 ? capacity / 1000 : capacity), capUnits);
	}
	IncrementSeqNum();

	return GCodeResult::ok;
}

uint64_t UsbVolume::GetCapacity() const noexcept
{
	// Get capacity of device
	uint64_t const block_count = tuh_msc_get_block_count(address, lun);
	uint64_t const block_size = tuh_msc_get_block_size(address, lun);
	return block_count * block_size;
}

uint32_t UsbVolume::GetInterfaceSpeed() const noexcept
{
	tusb_speed_t speed = tuh_speed_get(address);
	return (speed == TUSB_SPEED_HIGH ? 480000000 : 12000000) / 8;
}

DRESULT UsbVolume::DiskInitialize() noexcept
{
	return RES_OK; // nothing to do
}

DRESULT UsbVolume::DiskStatus() noexcept
{
	return static_cast<DRESULT>(tuh_msc_mounted(address) ? 0 : STA_NODISK);
}

DRESULT UsbVolume::DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept
{
	tuh_msc_read10(address, lun, buff, sector, (uint16_t)count, disk_io_complete, reinterpret_cast<uintptr_t>(&ioDone));
	ioDone.Take();
	return RES_OK;
}

DRESULT UsbVolume::DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept
{
	tuh_msc_write10(address, lun, buff, sector, (uint16_t)count, disk_io_complete, reinterpret_cast<uintptr_t>(&ioDone));
	ioDone.Take();
	return RES_OK;
}

DRESULT UsbVolume::DiskIoctl(BYTE cmd, void *buff) noexcept
{
	switch (cmd)
	{
	case CTRL_SYNC:
		// nothing to do since we do blocking
		return RES_OK;

	case GET_SECTOR_COUNT:
		*((DWORD *)buff) = (WORD)tuh_msc_get_block_count(address, lun);
		return RES_OK;

	case GET_SECTOR_SIZE:
		*((WORD *)buff) = (WORD)tuh_msc_get_block_size(address, lun);
		return RES_OK;

	case GET_BLOCK_SIZE:
		*((DWORD *)buff) = 1; // erase block size in units of sector size
		return RES_OK;

	default:
		return RES_PARERR;
	}

	return RES_OK;
}

void UsbVolume::DeviceUnmount() noexcept
{
	switch (state)
	{
	case State::removed:
		state = State::free;
		break;
	case State::mounted:
		state = State::inserted;
	default:
		break;
	}
}

bool UsbVolume::Accept(uint8_t address)
{
	if (state == State::free)
	{
		state = State::inserted;
		this->address = address;
		return true;
	}
	return false;
}

void UsbVolume::Free()
{
	switch (state)
	{
	case State::inserted:
		state = State::free; // immediately set free
		address = 0;
		break;
	case State::mounted:
		state = State::removed; // perform actual freeing in spin function
	default:
		break;
	}
}

/*static*/ void UsbVolume::VolumeInserted(uint8_t address)
{
	for (UsbVolume *drive : usbDrives)
	{
		// Check if there are free ones that can accept
		if (drive->Accept(address))
		{
			break;
		}
	}
}

/*static*/ void UsbVolume::VolumeRemoved(uint8_t address)
{
	for (UsbVolume *drive : usbDrives)
	{
		if (drive->address == address)
		{
			drive->Free();
		}
	}
}

extern "C" void tuh_msc_mount_cb(uint8_t address)
{
	UsbVolume::VolumeInserted(address);
}

extern "C" void tuh_msc_umount_cb(uint8_t address)
{
	UsbVolume::VolumeRemoved(address);
}

/*static*/ UsbVolume *UsbVolume::usbDrives[NumUsbDrives];

#endif // CORE_USES_TINYUSB && CFG_TUH_ENABLED
#endif // SUPPORT_USB_DRIVE
