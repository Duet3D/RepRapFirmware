#include <Platform/RepRap.h>

#include "StorageVolume.h"
#include "MassStorage.h"

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(StorageVolume, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...) OBJECT_MODEL_FUNC_IF_BODY(StorageVolume, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry StorageVolume::objectModelTable[] =
	{
		// Within each group, these entries must be in alphabetical order
		// 0. volumes[] root
		{"capacity", OBJECT_MODEL_FUNC_IF(self->IsMounted(), self->GetCapacity()), ObjectModelEntryFlags::none},
		{"freeSpace", OBJECT_MODEL_FUNC_IF(self->IsMounted(), self->GetFreeSpace()), ObjectModelEntryFlags::none},
		{"mounted", OBJECT_MODEL_FUNC(self->IsMounted()), ObjectModelEntryFlags::none},
		{"openFiles", OBJECT_MODEL_FUNC_IF(self->IsMounted(), MassStorage::AnyFileOpen(&(self->fileSystem))), ObjectModelEntryFlags::none},
		{"partitionSize", OBJECT_MODEL_FUNC_IF(self->IsMounted(), self->GetPartitionSize()), ObjectModelEntryFlags::none},
		{"path", OBJECT_MODEL_FUNC(self->path), ObjectModelEntryFlags::verbose},
		{"speed", OBJECT_MODEL_FUNC_IF(self->IsMounted(), (int32_t)self->GetInterfaceSpeed()), ObjectModelEntryFlags::none},
};

// TODO Add storages here in the format
/*
	openFiles = null
	path = null
*/

constexpr uint8_t StorageVolume::objectModelTableDescriptor[] = {1, 7};

DEFINE_GET_OBJECT_MODEL_TABLE(StorageVolume)

#endif

# if SAME70
alignas(4) static __nocache uint8_t sectorBuffers[FF_VOLUMES][FF_MAX_SS];
#endif

StorageVolume::StorageVolume(const char *id, uint8_t slot)
{
	this->id = id;
	this->slot = slot;
	path[0] += slot;
}

uint64_t StorageVolume::GetFreeSpace() const noexcept
{
	uint64_t res = fileSystem.free_clst * GetClusterSize();
	if (res < GetPartitionSize())
	{
		return res;
	}
	return 0; // free_clst is not valid, full FAT scan might not be worth it (such as on rare FAT16/FAT12 drives)
}

uint64_t StorageVolume::GetPartitionSize() const noexcept
{
	return (fileSystem.n_fatent - 2) * GetClusterSize();
}

void StorageVolume::Init() noexcept
{
	Clear();
	seqNum = 0;
	mutex.Create(id);
}

GCodeResult StorageVolume::Unmount(const StringRef& reply) noexcept
{
	if (MassStorage::AnyFileOpen(&fileSystem))
	{
		// Don't unmount the card if any files are open on it
		reply.printf("%s has open file(s)", id);
		return GCodeResult::error;
	}

	(void)InternalUnmount();

	reply.printf("%s may now be removed", id);
	IncrementSeqNum();

	return GCodeResult::ok;
}

unsigned int StorageVolume::InternalUnmount() noexcept
{
	MutexLocker lock(mutex);
	const unsigned int invalidated = MassStorage::InvalidateFiles(&fileSystem);
	f_mount(nullptr, path, 0);
	Clear();
	DeviceUnmount();
	reprap.VolumesUpdated();
	return invalidated;
}

uint64_t StorageVolume::GetClusterSize() const noexcept
{
	return (fileSystem.csize) * 512;
}

void StorageVolume::Clear()
{
	memset(&fileSystem, 0, sizeof(fileSystem));
#if SAME70
	fileSystem.win = sectorBuffers[slot];
	memset(sectorBuffers[slot], 0, sizeof(sectorBuffers[slot]));
#endif
}

/*static*/ const StringRef StorageVolume::noReply = StringRef(nullptr, 0);
