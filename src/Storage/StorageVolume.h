#pragma once

#include <stdint.h>

#include <General/StringRef.h>
#include <ObjectModel/ObjectModel.h>
#include <Libraries/Fatfs/diskio.h>

class StorageVolume INHERIT_OBJECT_MODEL
{
public:

	StorageVolume(const char *name, uint8_t slot);

	virtual void Init() noexcept;

	virtual void Spin() noexcept = 0;

	virtual GCodeResult Mount(const StringRef& reply, bool reportSuccess) noexcept = 0;
	virtual GCodeResult Unmount(const StringRef& reply) noexcept;

	virtual bool IsUseable(const StringRef& reply = noReply) const noexcept = 0;
	virtual bool IsMounted() const noexcept = 0;
	virtual bool IsDetected() const noexcept = 0;

	virtual uint64_t GetCapacity() const noexcept = 0;
	virtual uint64_t GetFreeSpace() const noexcept;
	virtual uint64_t GetPartitionSize() const noexcept;
	virtual uint64_t GetClusterSize() const noexcept;
	virtual uint32_t GetInterfaceSpeed() const noexcept = 0;

	virtual DRESULT DiskInitialize() noexcept = 0;
	virtual DRESULT DiskStatus() noexcept = 0;
	virtual DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept = 0;
	virtual DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept = 0;
	virtual DRESULT DiskIoctl(BYTE ctrl, void *buff) noexcept = 0;

	const char* GetPathName() const noexcept { return path; }
	Mutex& GetMutex() noexcept { return mutex; }

	int GetSequenceNum() const noexcept { return seqNum; }
	void IncrementSeqNum() noexcept { ++seqNum; }

protected:
	DECLARE_OBJECT_MODEL

	char path[3] = "0:";
	const char *id;
	uint8_t slot;
	Mutex mutex;
	uint16_t seqNum;
	FATFS fileSystem;

	const static StringRef noReply;

	void Clear();
	unsigned int InternalUnmount() noexcept;
	virtual void DeviceUnmount() noexcept = 0;
};
