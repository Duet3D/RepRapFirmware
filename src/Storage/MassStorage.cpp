#include "MassStorage.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>

#if HAS_MASS_STORAGE
# include <Libraries/Fatfs/diskio.h>
# include <Libraries/sd_mmc/sd_mmc.h>
# include <Libraries/sd_mmc/conf_sd_mmc.h>

// Check that the LFN configuration in FatFS is sufficient
static_assert(FF_MAX_LFN >= MaxFilenameLength, "FF_MAX_LFN too small");

// Check that the correct number of SD cards is configured in the library
static_assert(SD_MMC_MEM_CNT == NumSdCards);
#endif

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

#ifdef DUET3_MB6HC
# include <GCodes/GCodeBuffer/GCodeBuffer.h>
#endif

// A note on using mutexes:
// Each SD card volume has its own mutex. There is also one for the file table, and one for the find first/find next buffer.
// The FatFS subsystem locks and releases the appropriate volume mutex when it is called.
// Any function that needs to acquire both the file table mutex and a volume mutex MUST take the file table mutex first, to avoid deadlocks.
// Any function that needs to acquire both the find buffer mutex and a volume mutex MUST take the find buffer mutex first, to avoid deadlocks.
// No function should need to take both the file table mutex and the find buffer mutex.
// No function in here should be called when the caller already owns the shared SPI mutex.

#if HAS_MASS_STORAGE

// Private data and methods

# if SAME70
alignas(4) static __nocache uint8_t sectorBuffers[NumSdCards][512];
alignas(4) static __nocache char writeBufferStorage[NumFileWriteBuffers][FileWriteBufLen];
# endif

enum class CardDetectState : uint8_t
{
	notPresent = 0,
	inserting,
	present,
	removing
};

struct SdCardInfo INHERIT_OBJECT_MODEL
{
	FATFS fileSystem;
	uint32_t cdChangedTime;
	uint32_t mountStartTime;
	Mutex volMutex;
	uint16_t seq;
	Pin cdPin;
	bool mounting;
	bool isMounted;
	CardDetectState cardState;

	void Clear(unsigned int card) noexcept;

protected:
	DECLARE_OBJECT_MODEL
};

void SdCardInfo::Clear(unsigned int card) noexcept
{
	memset(&fileSystem, 0, sizeof(fileSystem));
#if SAME70
	fileSystem.win = sectorBuffers[card];
	memset(sectorBuffers[card], 0, sizeof(sectorBuffers[card]));
#endif
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(SdCardInfo, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(SdCardInfo, _condition,__VA_ARGS__)

// These two functions are only called from one place each in the OM table, hence inlined
static inline uint64_t GetFreeSpace(size_t slot)
{
	MassStorage::SdCardReturnedInfo returnedInfo;
	(void)MassStorage::GetCardInfo(slot, returnedInfo);
	return returnedInfo.freeSpace;
}

static inline uint64_t GetPartitionSize(size_t slot)
{
	MassStorage::SdCardReturnedInfo returnedInfo;
	(void)MassStorage::GetCardInfo(slot, returnedInfo);
	return returnedInfo.partitionSize;
}

static const char * const VolPathNames[] = { "0:/", "1:/" };
static_assert(ARRAY_SIZE(VolPathNames) >= NumSdCards, "Incorrect VolPathNames array");

#ifdef DUET3_MB6HC
static IoPort sd1Ports[2];		// first element is CS port, second is CD port
#endif

constexpr ObjectModelTableEntry SdCardInfo::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. volumes[] root
	{ "capacity",			OBJECT_MODEL_FUNC_IF(self->isMounted, (uint64_t)sd_mmc_get_capacity(context.GetLastIndex()) * 1024u),	ObjectModelEntryFlags::none },
	{ "freeSpace",			OBJECT_MODEL_FUNC_IF(self->isMounted, GetFreeSpace(context.GetLastIndex())),							ObjectModelEntryFlags::none },
	{ "mounted",			OBJECT_MODEL_FUNC(self->isMounted),																		ObjectModelEntryFlags::none },
	{ "openFiles",			OBJECT_MODEL_FUNC_IF(self->isMounted, MassStorage::AnyFileOpen(&(self->fileSystem))),					ObjectModelEntryFlags::none },
	{ "partitionSize",		OBJECT_MODEL_FUNC_IF(self->isMounted, GetPartitionSize(context.GetLastIndex())),						ObjectModelEntryFlags::none },
	{ "path",				OBJECT_MODEL_FUNC_NOSELF(VolPathNames[context.GetLastIndex()]),											ObjectModelEntryFlags::verbose },
	{ "speed",				OBJECT_MODEL_FUNC_IF(self->isMounted, (int32_t)sd_mmc_get_interface_speed(context.GetLastIndex())),		ObjectModelEntryFlags::none },
};

// TODO Add storages here in the format
/*
	openFiles = null
	path = null
*/

constexpr uint8_t SdCardInfo::objectModelTableDescriptor[] = { 1, 7 };

DEFINE_GET_OBJECT_MODEL_TABLE(SdCardInfo)

#endif

static SdCardInfo info[NumSdCards];
static DIR findDir;
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
static Mutex dirMutex;
static FileInfoParser infoParser;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
static FileWriteBuffer *freeWriteBuffers;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
static Mutex fsMutex;
static FileStore files[MAX_FILES];
#endif

// Construct a full path name from a path and a filename. Returns false if error i.e. filename too long
/*static*/ bool MassStorage::CombineName(const StringRef& outbuf, const char* directory, const char* fileName) noexcept
{
	bool hadError = false;
	if (directory != nullptr && directory[0] != 0 && fileName[0] != '/' && (strlen(fileName) < 2 || !isdigit(fileName[0]) || fileName[1] != ':'))
	{
		hadError = outbuf.copy(directory);
		if (!hadError)
		{
			const size_t len = outbuf.strlen();
			if (len != 0 && outbuf[len - 1] != '/')
			{
				hadError = outbuf.cat('/');
			}
		}
	}
	else
	{
		outbuf.Clear();
	}
	if (!hadError)
	{
		hadError = outbuf.cat(fileName);
	}
	if (hadError)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Filename too long: cap=%u, dir=%.12s%s name=%.12s%s\n",
										outbuf.Capacity(),
										directory, (strlen(directory) > 12 ? "..." : ""),
										fileName, (strlen(fileName) > 12 ? "..." : "")
									 );
		outbuf.copy("?????");
	}
	return !hadError;
}

#if HAS_MASS_STORAGE

# ifdef DUET3_MB6HC

// Return the number of volumes, which on the 6HC is normally 1 but can be increased to 2
size_t MassStorage::GetNumVolumes() noexcept
{
	return (sd1Ports[0].IsValid()) ? 2 : 1;		// we have 2 slots if the second one has a valid CS pin, else 1
}

// Configure additional SD card slots
// The card detect pin may be NoPin if the SD card slot doesn't support card detect
GCodeResult MassStorage::ConfigureSdCard(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	(void)gb.GetLimitedUIValue('D', 1, 2);		// only slot 1 may be configured
	IoPort * const portAddresses[2] = { &sd1Ports[0], &sd1Ports[1] };
	if (gb.Seen('C'))
	{
		const PinAccess accessNeeded[2] = { PinAccess::write1, PinAccess::read };
		if (IoPort::AssignPorts(gb, reply, PinUsedBy::sdCard, 2, portAddresses, accessNeeded) == 0)
		{
			return GCodeResult::error;
		}
		sd_mmc_change_cs_pin(1, sd1Ports[0].GetPin());
		info[1].cdPin = sd1Ports[1].GetPin();
		reprap.VolumesUpdated();
	}
	else
	{
		reply.copy("SD card 1 uses pins ");
		IoPort::AppendPinNames(reply, 2, portAddresses);
	}
	return GCodeResult::ok;
}

# endif

// Sequence number management
uint16_t MassStorage::GetVolumeSeq(unsigned int volume) noexcept
{
	return info[volume].seq;
}

// If 'path' is not the name of a temporary file, update the sequence number of its volume
// Return true if we did update the sequence number
static bool VolumeUpdated(const char *path) noexcept
{
	if (!StringEndsWithIgnoreCase(path, ".part")
#if HAS_SBC_INTERFACE
		&& !reprap.UsingSbcInterface()
#endif
	   )
	{
		const unsigned int volume = (isdigit(path[0]) && path[1] == ':') ? path[0] - '0' : 0;
		if (volume < ARRAY_SIZE(info))
		{
			++info[volume].seq;
			return true;
		}
	}
	return false;
}

// Unmount a file system returning the number of open files were invalidated
static unsigned int InternalUnmount(size_t card, bool doClose) noexcept
{
	SdCardInfo& inf = info[card];
	MutexLocker lock1(fsMutex);
	MutexLocker lock2(inf.volMutex);
	const unsigned int invalidated = MassStorage::InvalidateFiles(&inf.fileSystem, doClose);
	const char path[3] = { (char)('0' + card), ':', 0 };
	f_mount(nullptr, path, 0);
	inf.Clear(card);
	sd_mmc_unmount(card);
	inf.isMounted = false;
	reprap.VolumesUpdated();
	return invalidated;
}

static time_t ConvertTimeStamp(uint16_t fdate, uint16_t ftime) noexcept
{
	struct tm timeInfo;
	memset(&timeInfo, 0, sizeof(timeInfo));
	timeInfo.tm_year = (fdate >> 9) + 80;
	const uint16_t month = (fdate >> 5) & 0x0F;
	timeInfo.tm_mon = (month == 0) ? month : month - 1;		// month is 1..12 in FAT but 0..11 in struct tm
	timeInfo.tm_mday = max<int>(fdate & 0x1F, 1);
	timeInfo.tm_hour = (ftime >> 11) & 0x1F;
	timeInfo.tm_min = (ftime >> 5) & 0x3F;
	timeInfo.tm_sec = (ftime & 0x1F) * 2;
	timeInfo.tm_isdst = 0;
	return mktime(&timeInfo);
}

static const char* TranslateCardType(card_type_t ct) noexcept
{
	switch (ct)
	{
		case CARD_TYPE_SD | CARD_TYPE_HC:
			return "SDHC";
		case CARD_TYPE_SD:
			return "SD";
		case CARD_TYPE_MMC | CARD_TYPE_HC:
			return "MMC High Capacity";
		case CARD_TYPE_MMC:
			return "MMC";
		case CARD_TYPE_SDIO:
			return "SDIO";
		case CARD_TYPE_SD_COMBO:
			return "SD COMBO";
		case CARD_TYPE_UNKNOWN:
		default:
			return "Unknown type";
	}
}

static const char* TranslateCardError(sd_mmc_err_t err) noexcept
{
	switch (err)
	{
		case SD_MMC_ERR_NO_CARD:
			return "Card not found";
		case SD_MMC_ERR_UNUSABLE:
			return "Card is unusable";
		case SD_MMC_ERR_SLOT:
			return "Slot unknown";
		case SD_MMC_ERR_COMM:
			return "Communication error";
		case SD_MMC_ERR_PARAM:
			return "Illegal input parameter";
		case SD_MMC_ERR_WP:
			return "Card write protected";
		default:
			return "Unknown error";
	}
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

void MassStorage::Init() noexcept
{
	fsMutex.Create("FileSystem");

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	dirMutex.Create("DirSearch");
#endif

# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	freeWriteBuffers = nullptr;
	for (size_t i = 0; i < NumFileWriteBuffers; ++i)
	{
#  if SAME70
		freeWriteBuffers = new FileWriteBuffer(freeWriteBuffers, writeBufferStorage[i]);
#  else
		freeWriteBuffers = new FileWriteBuffer(freeWriteBuffers);
#  endif
	}
# endif

# if HAS_MASS_STORAGE
	static const char * const VolMutexNames[] = { "SD0", "SD1" };
	static_assert(ARRAY_SIZE(VolMutexNames) >= NumSdCards, "Incorrect VolMutexNames array");

	// Initialise the SD card structs
	for (size_t card = 0; card < NumSdCards; ++card)
	{
		SdCardInfo& inf = info[card];
		inf.Clear(card);
		inf.mounting = inf.isMounted = false;
		inf.seq = 0;
		inf.cdPin = SdCardDetectPins[card];
		inf.cardState = (inf.cdPin == NoPin) ? CardDetectState::present : CardDetectState::notPresent;
		inf.volMutex.Create(VolMutexNames[card]);
	}

	sd_mmc_init(SdWriteProtectPins, SdSpiCSPins);		// initialize SD MMC stack

	// We no longer mount the SD card here because it may take a long time if it fails
# endif
}


void MassStorage::Spin() noexcept
{
# if HAS_MASS_STORAGE
	for (size_t card = 0; card < NumSdCards; ++card)
	{
		SdCardInfo& inf = info[card];
		if (inf.cdPin != NoPin)
		{
			if (IoPort::ReadPin(inf.cdPin))
			{
				// Pin state says no card present
				switch (inf.cardState)
				{
				case CardDetectState::inserting:
				case CardDetectState::present:
					inf.cardState = CardDetectState::removing;
					inf.cdChangedTime = millis();
					break;

				case CardDetectState::removing:
					if (millis() - inf.cdChangedTime > SdCardDetectDebounceMillis)
					{
						inf.cardState = CardDetectState::notPresent;
						if (inf.isMounted)
						{
							const unsigned int numFiles = InternalUnmount(card, false);
							if (numFiles != 0)
							{
								reprap.GetPlatform().MessageF(ErrorMessage, "SD card %u removed with %u file(s) open on it\n", card, numFiles);
							}
						}
					}
					break;

				default:
					break;
				}
			}
			else
			{
				// Pin state says card is present
				switch (inf.cardState)
				{
				case CardDetectState::removing:
				case CardDetectState::notPresent:
					inf.cardState = CardDetectState::inserting;
					inf.cdChangedTime = millis();
					break;

				case CardDetectState::inserting:
					inf.cardState = CardDetectState::present;
					break;

				default:
					break;
				}
			}
		}
	}
# endif

	// Check if any files are supposed to be closed
	{
		MutexLocker lock(fsMutex);
		for (FileStore & fil : files)
		{
			if (fil.IsCloseRequested())
			{
				// We could not close this file in an ISR, so do it here
				fil.Close();
			}
		}
	}
}

FileStore* MassStorage::OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	{
		MutexLocker lock(fsMutex);
		for (size_t i = 0; i < MAX_FILES; i++)
		{
			if (files[i].IsFree())
			{
				FileStore * const ret = (files[i].Open(filePath, mode, preAllocSize)) ? &files[i]: nullptr;
# if HAS_MASS_STORAGE
				if (ret != nullptr && (mode == OpenMode::write || mode == OpenMode::writeWithCrc))
				{
					(void)VolumeUpdated(filePath);
				}
# endif
				return ret;
			}
		}
	}
	reprap.GetPlatform().Message(ErrorMessage, "Max open file count exceeded.\n");
	return nullptr;
}

// Close all files
void MassStorage::CloseAllFiles() noexcept
{
	MutexLocker lock(fsMutex);
	for (FileStore& f : files)
	{
		while (!f.IsFree())
		{
			f.Close();
		}
	}
}

#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Check if the specified directory exists
bool MassStorage::DirectoryExists(const char *path) noexcept
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	String<MaxFilenameLength> loc;
	loc.copy(path);
	return DirectoryExists(loc.GetRef());
}

// Check if the specified directory exists
// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
bool MassStorage::DirectoryExists(const StringRef& path) noexcept
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	const size_t len = path.strlen();
	if (len != 0 && (path[len - 1] == '/' || path[len - 1] == '\\'))
	{
		path.Truncate(len - 1);
	}

#if HAS_MASS_STORAGE
	DIR dir;
	const bool ok = (f_opendir(&dir, path.c_str()) == FR_OK);
	if (ok)
	{
		f_closedir(&dir);
	}
	return ok;
#elif HAS_EMBEDDED_FILES
	return EmbeddedFiles::DirectoryExists(path);
#endif
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Static helper functions
size_t FileWriteBuffer::fileWriteBufLen = FileWriteBufLen;

FileWriteBuffer *MassStorage::AllocateWriteBuffer() noexcept
{
	MutexLocker lock(fsMutex);

	FileWriteBuffer * const buffer = freeWriteBuffers;
	if (buffer != nullptr)
	{
		freeWriteBuffers = buffer->Next();
		buffer->SetNext(nullptr);
		buffer->DataTaken();				// make sure that the write pointer is clear
	}
	return buffer;
}

void MassStorage::ReleaseWriteBuffer(FileWriteBuffer *buffer) noexcept
{
	MutexLocker lock(fsMutex);
	buffer->SetNext(freeWriteBuffers);
	freeWriteBuffers = buffer;
}

# if HAS_SBC_INTERFACE

// Return true if any files are open on the file system
bool MassStorage::AnyFileOpen() noexcept
{
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (!fil.IsFree())
		{
			return true;
		}
	}
	return false;
}

// Invalidate all files
void MassStorage::InvalidateAllFiles() noexcept
{
	MutexLocker lock(fsMutex);
	for (FileStore& f : files)
	{
		while (!f.IsFree())
		{
			f.Invalidate();
		}
	}
}

# endif

# if HAS_MASS_STORAGE

// Delete a file or directory
static bool InternalDelete(const char* filePath, bool messageIfFailed) noexcept
{
	FRESULT unlinkReturn;
	bool isOpen = false;

	// Start new scope to lock the filesystem for the minimum time
	{
		MutexLocker lock(fsMutex);

		// First check whether the file is open - don't allow it to be deleted if it is, because that may corrupt the file system
		FIL file;
		const FRESULT openReturn = f_open(&file, filePath, FA_OPEN_EXISTING | FA_READ);
		if (openReturn == FR_OK)
		{
			for (const FileStore& fil : files)
			{
				if (fil.IsSameFile(file))
				{
					isOpen = true;
					break;
				}
			}
			f_close(&file);
		}

		if (!isOpen)
		{
			unlinkReturn = f_unlink(filePath);
		}
	}

	if (isOpen)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Cannot delete file %s because it is open\n", filePath);
		return false;
	}

	if (unlinkReturn != FR_OK)
	{
		// If the error was that the file or path doesn't exist, don't generate a global error message, but still return false
		if (unlinkReturn != FR_NO_FILE && unlinkReturn != FR_NO_PATH)
		{
			if (messageIfFailed)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Failed to delete file %s\n", filePath);
			}
		}
		return false;
	}
	return true;
}

# endif

// Delete a file or directory and update the volume sequence number returning true if successful
bool MassStorage::Delete(const char* filePath, bool messageIfFailed) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		if (reprap.GetSbcInterface().DeleteFileOrDirectory(filePath))
		{
			return true;
		}

		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to delete file %s\n", filePath);
		}
		return false;
	}
#endif

#if HAS_MASS_STORAGE
	const bool ok = InternalDelete(filePath, messageIfFailed);
	if (ok)
	{
		(void)VolumeUpdated(filePath);
	}
	return ok;
#else
	return false;
#endif
}

#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Open a directory to read a file list. Returns true if it contains any files, false otherwise.
// If this returns true then the file system mutex is owned. The caller must subsequently release the mutex either
// by calling FindNext until it returns false, or by calling AbandonFindNext.
bool MassStorage::FindFirst(const char *directory, FileInfo &file_info) noexcept
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	String<MaxFilenameLength> loc;
	loc.copy(directory);
	const size_t len = loc.strlen();
	if (len != 0 && (loc[len - 1] == '/' || loc[len - 1] == '\\'))
	{
		loc.Truncate(len - 1);
	}

	if (!dirMutex.Take(10000))
	{
		return false;
	}

#if HAS_MASS_STORAGE
	FRESULT res = f_opendir(&findDir, loc.c_str());
	if (res == FR_OK)
	{
		FILINFO entry;

		for (;;)
		{
			res = f_readdir(&findDir, &entry);
			if (res != FR_OK || entry.fname[0] == 0) break;
			if (!StringEqualsIgnoreCase(entry.fname, ".") && !StringEqualsIgnoreCase(entry.fname, ".."))
			{
				file_info.isDirectory = (entry.fattrib & AM_DIR);
				file_info.fileName.copy(entry.fname);
				file_info.size = entry.fsize;
				file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);
				return true;
			}
		}
		f_closedir(&findDir);
	}
#elif HAS_EMBEDDED_FILES
	if (EmbeddedFiles::FindFirst(directory, file_info))
	{
		return true;
	}
#endif

	dirMutex.Release();
	return false;
}

// Find the next file in a directory. Returns true if another file has been read.
// If it returns false then it also releases the mutex.
bool MassStorage::FindNext(FileInfo &file_info) noexcept
{
	if (dirMutex.GetHolder() != RTOSIface::GetCurrentTask())
	{
		return false;		// error, we don't hold the mutex
	}

#if HAS_MASS_STORAGE
	FILINFO entry;

	if (f_readdir(&findDir, &entry) == FR_OK && entry.fname[0] != 0)
	{
		file_info.isDirectory = (entry.fattrib & AM_DIR);
		file_info.size = entry.fsize;
		file_info.fileName.copy(entry.fname);
		file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);
		return true;
	}

	f_closedir(&findDir);
#elif HAS_EMBEDDED_FILES
	if (EmbeddedFiles::FindNext(file_info))
	{
		return true;
	}
#endif
	// TODO implement SBC interface for this

	dirMutex.Release();
	return false;
}

// Quit searching for files. Needed to avoid hanging on to the mutex. Safe to call even if the caller doesn't hold the mutex.
void MassStorage::AbandonFindNext() noexcept
{
	if (dirMutex.GetHolder() == RTOSIface::GetCurrentTask())
	{
		dirMutex.Release();
	}
}

#endif

#if HAS_MASS_STORAGE

// Month names. The first entry is used for invalid month numbers.
static const char *monthNames[13] = { "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

// Returns the name of the specified month or '???' if the specified value is invalid.
const char* MassStorage::GetMonthName(const uint8_t month) noexcept
{
	return (month <= 12) ? monthNames[month] : monthNames[0];
}

// Ensure that the path up to the last '/' (excluding trailing '/' characters) in filePath exists, returning true if successful
bool MassStorage::EnsurePath(const char* filePath, bool messageIfFailed) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		// This isn't needed in SBC mode because DSF does it automatically
		return true;
	}
#endif

	// Try to create the path of this file if we want to write to it
	String<MaxFilenameLength> filePathCopy;
	filePathCopy.copy(filePath);

	size_t i = (isdigit(filePathCopy[0]) && filePathCopy[1] == ':') ? 2 : 0;
	if (filePathCopy[i] == '/')
	{
		++i;
	}

	size_t limit = filePathCopy.strlen();
	while (limit != 0 && filePath[limit - 1] == '/')
	{
		--limit;
	}
	while (i < limit)
	{
		if (filePathCopy[i] == '/')
		{
			filePathCopy[i] = 0;
			if (!MassStorage::DirectoryExists(filePathCopy.GetRef()) && f_mkdir(filePathCopy.c_str()) != FR_OK)
			{
				if (messageIfFailed)
				{
					reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create folder %s in path %s\n", filePathCopy.c_str(), filePath);
				}
				return false;
			}
			filePathCopy[i] = '/';
		}
		++i;
	}
	return true;
}

// Create a new directory
bool MassStorage::MakeDirectory(const char *directory, bool messageIfFailed) noexcept
{
	if (!EnsurePath(directory, messageIfFailed))
	{
		return false;
	}
	if (f_mkdir(directory) != FR_OK)
	{
		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create folder %s\n", directory);
		}
		return false;
	}
	(void)VolumeUpdated(directory);
	return true;
}

// Rename a file or directory, optionally deleting the existing one if it exists
bool MassStorage::Rename(const char *oldFilename, const char *newFilename, bool deleteExisting, bool messageIfFailed) noexcept
{
	// Check the the old file exists before we possibly delete any existing file with the new name
	if (!FileExists(oldFilename) && !DirectoryExists(oldFilename))
	{
		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to rename file %s: file not found\n", oldFilename);
		}
		return false;
	}
	if (newFilename[0] >= '0' && newFilename[0] <= '9' && newFilename[1] == ':')
	{
		// Workaround for DWC 1.13 which sends a volume specification at the start of the new path.
		// f_rename can't handle this, so skip past the volume specification.
		// We are assuming that the user isn't really trying to rename across volumes. This is a safe assumption when the client is DWC.
		newFilename += 2;
	}
	if (!EnsurePath(newFilename, messageIfFailed))
	{
		return false;
	}
	if (deleteExisting && (FileExists(newFilename) || DirectoryExists(newFilename)))
	{
		if (!InternalDelete(newFilename, messageIfFailed))
		{
			return false;
		}
	}
	if (f_rename(oldFilename, newFilename) != FR_OK)
	{
		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to rename file or directory %s to %s\n", oldFilename, newFilename);
		}
		return false;
	}

	if (!VolumeUpdated(oldFilename))				// only update the sequence number once
	{
		(void)VolumeUpdated(newFilename);
	}
	return true;
}
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Check if the specified file exists
bool MassStorage::FileExists(const char *filePath) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		return reprap.GetSbcInterface().FileExists(filePath);
	}
#endif

#if HAS_MASS_STORAGE
	FILINFO fil;
	return (f_stat(filePath, &fil) == FR_OK);
#else
	return false;
#endif
}

#endif

#if HAS_MASS_STORAGE

// Return the last modified time of a file, or zero if failure
time_t MassStorage::GetLastModifiedTime(const char *filePath) noexcept
{
	FILINFO fil;
	if (f_stat(filePath, &fil) == FR_OK)
	{
		return ConvertTimeStamp(fil.fdate, fil.ftime);
	}
	return 0;
}

bool MassStorage::SetLastModifiedTime(const char *filePath, time_t time) noexcept
{
	tm timeInfo;
	gmtime_r(&time, &timeInfo);
	FILINFO fno;
    fno.fdate = (WORD)(((timeInfo.tm_year - 80) * 512U) | (timeInfo.tm_mon + 1) * 32U | timeInfo.tm_mday);
    fno.ftime = (WORD)(timeInfo.tm_hour * 2048U | timeInfo.tm_min * 32U | timeInfo.tm_sec / 2U);
    const bool ok = (f_utime(filePath, &fno) == FR_OK);
    if (!ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to set last modified time for file '%s'\n", filePath);
	}
    return ok;
}

// Check if the drive referenced in the specified path is mounted. Return true if it is.
// Ideally we would try to mount it if it is not, however mounting a drive can take a long time, and the functions that call this are expected to execute quickly.
bool MassStorage::CheckDriveMounted(const char* path) noexcept
{
	const size_t card = (strlen(path) >= 2 && path[1] == ':' && isDigit(path[0]))
						? path[0] - '0'
						: 0;
	return card < GetNumVolumes() && info[card].isMounted;
}

// Return true if any files are open on the file system
bool MassStorage::AnyFileOpen(const FATFS *fs) noexcept
{
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (fil.IsOpenOn(fs))
		{
			return true;
		}
	}
	return false;
}

// Invalidate all open files on the specified file system, returning the number of files that were invalidated
unsigned int MassStorage::InvalidateFiles(const FATFS *fs, bool doClose) noexcept
{
	unsigned int invalidated = 0;
	MutexLocker lock(fsMutex);
	for (FileStore & fil : files)
	{
		if (fil.Invalidate(fs, doClose))
		{
			++invalidated;
		}
	}
	return invalidated;
}

bool MassStorage::IsCardDetected(size_t card) noexcept
{
	return info[card].cardState == CardDetectState::present;
}

#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Mount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
// This may only be called to mount one card at a time.
GCodeResult MassStorage::Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept
{
	if (card >= GetNumVolumes())
	{
		reply.copy("SD card number out of range");
		return GCodeResult::error;
	}

# if HAS_MASS_STORAGE
	SdCardInfo& inf = info[card];
	MutexLocker lock1(fsMutex);
	MutexLocker lock2(inf.volMutex);
	if (!inf.mounting)
	{
		if (inf.isMounted)
		{
			if (AnyFileOpen(&inf.fileSystem))
			{
				// Don't re-mount the card if any files are open on it
				reply.copy("SD card has open file(s)");
				return GCodeResult::error;
			}
			(void)InternalUnmount(card, false);
		}

		inf.mountStartTime = millis();
		inf.mounting = true;
		delay(2);
	}

	if (inf.cardState == CardDetectState::notPresent)
	{
		reply.copy("No SD card present");
		inf.mounting = false;
		return GCodeResult::error;
	}

	if (inf.cardState != CardDetectState::present)
	{
		return GCodeResult::notFinished;						// wait for debounce to finish
	}

	const sd_mmc_err_t err = sd_mmc_check(card);
	if (err != SD_MMC_OK && millis() - inf.mountStartTime < 5000)
	{
		delay(2);
		return GCodeResult::notFinished;
	}

	inf.mounting = false;
	if (err != SD_MMC_OK)
	{
		reply.printf("Cannot initialise SD card %u: %s", card, TranslateCardError(err));
		return GCodeResult::error;
	}

	// Mount the file systems
	const char path[3] = { (char)('0' + card), ':', 0 };
	const FRESULT mounted = f_mount(&inf.fileSystem, path, 1);
	if (mounted == FR_NO_FILESYSTEM)
	{
		reply.printf("Cannot mount SD card %u: no FAT filesystem found on card (EXFAT is not supported)", card);
		return GCodeResult::error;
	}
	if (mounted != FR_OK)
	{
		reply.printf("Cannot mount SD card %u: code %d", card, mounted);
		return GCodeResult::error;
	}

	inf.isMounted = true;
	reprap.VolumesUpdated();
	if (reportSuccess)
	{
		float capacity = ((float)sd_mmc_get_capacity(card) * 1024) / 1000000;		// get capacity and convert from Kib to Mbytes
		const char* capUnits;
		if (capacity >= 1000.0)
		{
			capacity /= 1000;
			capUnits = "Gb";
		}
		else
		{
			capUnits = "Mb";
		}
		reply.printf("%s card mounted in slot %u, capacity %.2f%s", TranslateCardType(sd_mmc_get_type(card)), card, (double)capacity, capUnits);
	}

	++inf.seq;
# endif

	return GCodeResult::ok;
}

// Unmount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
GCodeResult MassStorage::Unmount(size_t card, const StringRef& reply) noexcept
{
	if (card >= GetNumVolumes())
	{
		reply.copy("SD card number out of range");
		return GCodeResult::error;
	}

# if HAS_MASS_STORAGE
	reply.printf("SD card %u may now be removed", card);
	const unsigned int numFilesClosed = InternalUnmount(card, true);
	if (numFilesClosed != 0)
	{
		reply.catf(" (%u file(s) were closed)", numFilesClosed);
	}
	++info[card].seq;
# endif

	return GCodeResult::ok;
}

bool MassStorage::IsDriveMounted(size_t drive) noexcept
{
	return drive < GetNumVolumes()
#if HAS_MASS_STORAGE
		&& info[drive].isMounted
#endif
		;
}

unsigned int MassStorage::GetNumFreeFiles() noexcept
{
	unsigned int numFreeFiles = 0;
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (fil.IsFree())
		{
			++numFreeFiles;
		}
	}
	return numFreeFiles;
}

GCodeResult MassStorage::GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly) noexcept
{
	return infoParser.GetFileInfo(filePath, info, quitEarly);
}

void MassStorage::Diagnostics(MessageType mtype) noexcept
{
	Platform& platform = reprap.GetPlatform();

	// Show the number of free entries in the file table
	platform.MessageF(mtype, "=== Storage ===\nFree file entries: %u\n", MassStorage::GetNumFreeFiles());

# if HAS_MASS_STORAGE
#  if HAS_HIGH_SPEED_SD
	// Show the HSMCI CD pin and speed
	platform.MessageF(mtype, "SD card 0 %s, interface speed: %.1fMBytes/sec\n",
								(IsCardDetected(0) ? "detected" : "not detected"), (double)((float)sd_mmc_get_interface_speed(0) * 0.000001));
#  else
	platform.MessageF(mtype, "SD card 0 %s\n", (MassStorage::IsCardDetected(0) ? "detected" : "not detected"));
#  endif

	// Show the longest SD card write time
	platform.MessageF(mtype, "SD card longest read time %.1fms, write time %.1fms, max retries %u\n",
								(double)DiskioGetAndClearLongestReadTime(), (double)DiskioGetAndClearLongestWriteTime(), DiskioGetAndClearMaxRetryCount());
# endif
}

#endif

#if HAS_MASS_STORAGE

// Append the simulated printing time to the end of the file
void MassStorage::RecordSimulationTime(const char *printingFilePath, uint32_t simSeconds) noexcept
{
	FileStore * const file = OpenFile(printingFilePath, OpenMode::append, 0);
	bool ok = (file != nullptr);
	if (ok)
	{
		// Check whether there is already simulation info at the end of the file, in which case we should replace it
		constexpr size_t BufferSize = 100;
		String<BufferSize> buffer;
		const size_t bytesToRead = (size_t)min<FilePosition>(file->Length(), BufferSize);
		const FilePosition seekPos = file->Length() - bytesToRead;
		ok = file->Seek(seekPos);
		time_t lastModtime = 0;
		if (ok)
		{
			ok = (file->Read(buffer.GetRef().Pointer(), bytesToRead) == (int)bytesToRead);
			if (ok)
			{
				lastModtime = GetLastModifiedTime(printingFilePath);			// save the last modified time to that we can restore it later
				buffer[bytesToRead] = 0;										// this is OK because String<N> has N+1 bytes of storage
				const char* const pos = strstr(buffer.c_str(), FileInfoParser::SimulatedTimeString);
				if (pos != nullptr)
				{
					ok = file->Seek(seekPos + (pos - buffer.c_str()));			// overwrite previous simulation time
				}
				if (ok)
				{
					buffer.printf("%s: %" PRIu32 "\n", FileInfoParser::SimulatedTimeString, simSeconds);
					ok = file->Write(buffer.c_str());
					if (ok)
					{
						ok = file->Truncate();									// truncate file in case we overwrote a previous longer simulation time
					}
				}
			}
		}
		if (!file->Close())
		{
			ok = false;
		}
		if (ok && lastModtime != 0)
		{
			ok = SetLastModifiedTime(printingFilePath, lastModtime);
		}
	}

	if (!ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to append simulated print time to file %s\n", printingFilePath);
	}
}

// Get information about the SD card and interface speed
MassStorage::InfoResult MassStorage::GetCardInfo(size_t slot, SdCardReturnedInfo& returnedInfo) noexcept
{
	if (slot >= GetNumVolumes())
	{
		return InfoResult::badSlot;
	}

	SdCardInfo& inf = info[slot];
	if (!inf.isMounted)
	{
		return InfoResult::noCard;
	}

	returnedInfo.cardCapacity = (uint64_t)sd_mmc_get_capacity(slot) * 1024;
	returnedInfo.speed = sd_mmc_get_interface_speed(slot);
	String<StringLength50> path;
	path.printf("%u:/", slot);
	uint32_t freeClusters;
	FATFS *fs;
	const FRESULT fr = f_getfree(path.c_str(), &freeClusters, &fs);
	if (fr == FR_OK)
	{
		returnedInfo.clSize = fs->csize * 512;
		returnedInfo.partitionSize = (uint64_t)(fs->n_fatent - 2) * returnedInfo.clSize;
		returnedInfo.freeSpace = (uint64_t)freeClusters * returnedInfo.clSize;
	}
	else
	{
		returnedInfo.clSize = 0;
		returnedInfo.cardCapacity = returnedInfo.partitionSize = returnedInfo.freeSpace = 0;
	}
	return InfoResult::ok;
}

Mutex& MassStorage::GetVolumeMutex(size_t vol) noexcept
{
	return info[vol].volMutex;
}

# if SUPPORT_OBJECT_MODEL

const ObjectModel * MassStorage::GetVolume(size_t vol) noexcept
{
	return &info[vol];
}

# endif


// Functions called by FatFS to acquire/release mutual exclusion
extern "C"
{
	// Create a sync object. We already created it, we just need to copy the handle.
	int ff_cre_syncobj (BYTE vol, FF_SYNC_t* psy) noexcept
	{
		*psy = &MassStorage::GetVolumeMutex(vol);
		return 1;
	}

	// Lock sync object
	int ff_req_grant (FF_SYNC_t sy) noexcept
	{
		sy->Take();
		return 1;
	}

	// Unlock sync object
	void ff_rel_grant (FF_SYNC_t sy) noexcept
	{
		sy->Release();
	}

	// Delete a sync object
	int ff_del_syncobj (FF_SYNC_t sy) noexcept
	{
		return 1;		// nothing to do, we never delete the mutex
	}
}

#endif

// End
