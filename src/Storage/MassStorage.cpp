#include "MassStorage.h"
#include "Platform.h"
#include "RepRap.h"
#include "sd_mmc.h"

// Check that the LFN configuration in FatFS is sufficient
static_assert(FF_MAX_LFN >= MaxFilenameLength, "FF_MAX_LFN too small");

// A note on using mutexes:
// Each SD card volume has its own mutex. There is also one for the file table, and one for the find first/find next buffer.
// The FatFS subsystem locks and releases the appropriate volume mutex when it is called.
// Any function that needs to acquire both the file table mutex and a volume mutex MUST take the file table mutex first, to avoid deadlocks.
// Any function that needs to acquire both the find buffer mutex and a volume mutex MUST take the find buffer mutex first, to avoid deadlocks.
// No function should need to take both the file table mutex and the find buffer mutex.
// No function in here should be called when the caller already owns the shared SPI mutex.

// Static helper functions - not declared as class members to avoid having to include sd_mmc.h everywhere
static const char* TranslateCardType(card_type_t ct)
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

static const char* TranslateCardError(sd_mmc_err_t err)
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

// Mass Storage class
MassStorage::MassStorage(Platform* p) : freeWriteBuffers(nullptr)
{
}

void MassStorage::Init()
{
	static const char * const VolMutexNames[] = { "SD0", "SD1" };
	static_assert(ARRAY_SIZE(VolMutexNames) >= NumSdCards, "Incorrect VolMutexNames array");

	// Create the mutexes
	fsMutex.Create("FileSystem");
	dirMutex.Create("DirSearch");

	for (size_t i = 0; i < NumFileWriteBuffers; ++i)
	{
		freeWriteBuffers = new FileWriteBuffer(freeWriteBuffers);
	}

	for (size_t card = 0; card < NumSdCards; ++card)
	{
		SdCardInfo& inf = info[card];
		memset(&inf.fileSystem, 0, sizeof(inf.fileSystem));
		inf.mounting = inf.isMounted = false;
		inf.cdPin = SdCardDetectPins[card];
		inf.cardState = (inf.cdPin == NoPin) ? CardDetectState::present : CardDetectState::notPresent;
		inf.volMutex.Create(VolMutexNames[card]);
	}

	sd_mmc_init(SdWriteProtectPins, SdSpiCSPins);		// initialize SD MMC stack

	// We no longer mount the SD card here because it may take a long time if it fails
}

FileWriteBuffer *MassStorage::AllocateWriteBuffer()
{
	MutexLocker lock(fsMutex);
	if (freeWriteBuffers == nullptr)
	{
		return nullptr;
	}

	FileWriteBuffer * const buffer = freeWriteBuffers;
	freeWriteBuffers = buffer->Next();
	buffer->SetNext(nullptr);
	return buffer;
}

void MassStorage::ReleaseWriteBuffer(FileWriteBuffer *buffer)
{
	MutexLocker lock(fsMutex);
	buffer->SetNext(freeWriteBuffers);
	freeWriteBuffers = buffer;
}

FileStore* MassStorage::OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize)
{
	{
		MutexLocker lock(fsMutex);
		for (size_t i = 0; i < MAX_FILES; i++)
		{
			if (files[i].usageMode == FileUseMode::free)
			{
				return (files[i].Open(filePath, mode, preAllocSize)) ? &files[i]: nullptr;
			}
		}
	}
	reprap.GetPlatform().Message(ErrorMessage, "Max open file count exceeded.\n");
	return nullptr;
}

// Close all files
void MassStorage::CloseAllFiles()
{
	MutexLocker lock(fsMutex);
	for (FileStore& f : files)
	{
		while (f.usageMode != FileUseMode::free)
		{
			f.Close();
		}
	}
}

// Construct a full path name from a path and a filename. Returns false if error i.e. filename too long
/*static*/ bool MassStorage::CombineName(const StringRef& outbuf, const char* directory, const char* fileName)
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

// Open a directory to read a file list. Returns true if it contains any files, false otherwise.
// If this returns true then the file system mutex is owned. The caller must subsequently release the mutex either
// by calling FindNext until it returns false, or by calling AbandonFindNext.
bool MassStorage::FindFirst(const char *directory, FileInfo &file_info)
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

	dirMutex.Release();
	return false;
}

// Find the next file in a directory. Returns true if another file has been read.
// If it returns false then it also releases the mutex.
bool MassStorage::FindNext(FileInfo &file_info)
{
	if (dirMutex.GetHolder() != RTOSIface::GetCurrentTask())
	{
		return false;		// error, we don't hold the mutex
	}

	FILINFO entry;

	if (f_readdir(&findDir, &entry) != FR_OK || entry.fname[0] == 0)
	{
		f_closedir(&findDir);
		dirMutex.Release();
		return false;
	}

	file_info.isDirectory = (entry.fattrib & AM_DIR);
	file_info.size = entry.fsize;
	file_info.fileName.copy(entry.fname);
	file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);
	return true;
}

// Quit searching for files. Needed to avoid hanging on to the mutex. Safe to call even if the caller doesn't hold the mutex.
void MassStorage::AbandonFindNext()
{
	if (dirMutex.GetHolder() == RTOSIface::GetCurrentTask())
	{
		dirMutex.Release();
	}
}

// Month names. The first entry is used for invalid month numbers.
static const char *monthNames[13] = { "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

// Returns the name of the specified month or '???' if the specified value is invalid.
const char* MassStorage::GetMonthName(const uint8_t month)
{
	return (month <= 12) ? monthNames[month] : monthNames[0];
}

// Delete a file or directory
bool MassStorage::Delete(const char* filePath)
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
				if (fil.file.obj.fs == file.obj.fs && fil.file.dir_sect == file.dir_sect && fil.file.dir_ptr == file.dir_ptr )
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
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to delete file %s\n", filePath);
		}
		return false;
	}
	return true;
}

// Create a new directory
bool MassStorage::MakeDirectory(const char *parentDir, const char *dirName)
{
	String<MaxFilenameLength> location;
	if (!CombineName(location.GetRef(), parentDir, dirName))
	{
		return false;
	}
	if (f_mkdir(location.c_str()) != FR_OK)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create directory %s\n", location.c_str());
		return false;
	}
	return true;
}

bool MassStorage::MakeDirectory(const char *directory)
{
	if (f_mkdir(directory) != FR_OK)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create directory %s\n", directory);
		return false;
	}
	return true;
}

// Rename a file or directory
bool MassStorage::Rename(const char *oldFilename, const char *newFilename)
{
	if (newFilename[0] >= '0' && newFilename[0] <= '9' && newFilename[1] == ':')
	{
		// Workaround for DWC 1.13 which send a volume specification at the start of the new path.
		// f_rename can't handle this, so skip past the volume specification.
		// We are assuming that the user isn't really trying to rename across volumes. This is a safe assumption when the client is DWC.
		newFilename += 2;
	}
	if (f_rename(oldFilename, newFilename) != FR_OK)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to rename file or directory %s to %s\n", oldFilename, newFilename);
		return false;
	}
	return true;
}

// Check if the specified file exists
bool MassStorage::FileExists(const char *filePath) const
{
	FILINFO fil;
	return (f_stat(filePath, &fil) == FR_OK);
}

// Check if the specified directory exists
// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
bool MassStorage::DirectoryExists(const StringRef& path) const
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	const size_t len = path.strlen();
	if (len != 0 && (path[len - 1] == '/' || path[len - 1] == '\\'))
	{
		path.Truncate(len - 1);
	}

	DIR dir;
	const bool ok = (f_opendir(&dir, path.c_str()) == FR_OK);
	if (ok)
	{
		f_closedir(&dir);
	}
	return ok;
}

// Check if the specified directory exists
bool MassStorage::DirectoryExists(const char *path) const
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	String<MaxFilenameLength> loc;
	loc.copy(path);
	return DirectoryExists(loc.GetRef());
}

// Return the last modified time of a file, or zero if failure
time_t MassStorage::GetLastModifiedTime(const char *filePath) const
{
	FILINFO fil;
	if (f_stat(filePath, &fil) == FR_OK)
	{
		return ConvertTimeStamp(fil.fdate, fil.ftime);
	}
	return 0;
}

bool MassStorage::SetLastModifiedTime(const char *filePath, time_t time)
{
	const struct tm * const timeInfo = gmtime(&time);
	FILINFO fno;
    fno.fdate = (WORD)(((timeInfo->tm_year - 80) * 512U) | (timeInfo->tm_mon + 1) * 32U | timeInfo->tm_mday);
    fno.ftime = (WORD)(timeInfo->tm_hour * 2048U | timeInfo->tm_min * 32U | timeInfo->tm_sec / 2U);
    const bool ok = (f_utime(filePath, &fno) == FR_OK);
    if (!ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to set last modified time for file '%s'\n", filePath);
	}
    return ok;
}

// Mount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
// This may only be called to mount one card at a time.
GCodeResult MassStorage::Mount(size_t card, const StringRef& reply, bool reportSuccess)
{
	if (card >= NumSdCards)
	{
		reply.copy("SD card number out of range");
		return GCodeResult::error;
	}

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
	if (mounted != FR_OK)
	{
		reply.printf("Cannot mount SD card %u: code %d", card, mounted);
		return GCodeResult::error;
	}

	inf.isMounted = true;
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

	return GCodeResult::ok;
}

// Unmount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
GCodeResult MassStorage::Unmount(size_t card, const StringRef& reply)
{
	if (card >= NumSdCards)
	{
		reply.copy("SD card number out of range");
		return GCodeResult::error;
	}

	reply.printf("SD card %u may now be removed", card);
	const unsigned int numFilesClosed = InternalUnmount(card, true);
	if (numFilesClosed != 0)
	{
		reply.catf(" (%u file(s) were closed)", numFilesClosed);
	}
	return GCodeResult::ok;
}

// Check if the drive referenced in the specified path is mounted. Return true if it is.
// Ideally we would try to mount it if it is not, however mounting a drive can take a long time, and the functions that call this are expected to execute quickly.
bool MassStorage::CheckDriveMounted(const char* path)
{
	const size_t card = (strlen(path) >= 2 && path[1] == ':' && isDigit(path[0]))
						? path[0] - '0'
						: 0;
	return card < NumSdCards && info[card].isMounted;
}

// Return true if any files are open on the file system
bool MassStorage::AnyFileOpen(const FATFS *fs) const
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
unsigned int MassStorage::InvalidateFiles(const FATFS *fs, bool doClose)
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

/*static*/ time_t MassStorage::ConvertTimeStamp(uint16_t fdate, uint16_t ftime)
{
	struct tm timeInfo;
	memset(&timeInfo, 0, sizeof(timeInfo));
	timeInfo.tm_year = (fdate >> 9) + 80;
	const uint16_t month = (fdate >> 5) & 0x0F;
	timeInfo.tm_mon = (month == 0) ? month : month - 1;		// month is 1..12 in FAT but 0..11 in struct tm
	timeInfo.tm_mday = max<int>(fdate & 0x1F, 1);
	timeInfo.tm_hour = (ftime >> 11) & 0x1F;
	timeInfo.tm_min = (ftime >> 5) & 0x3F;
	timeInfo.tm_sec = ftime & 0x1F;
	timeInfo.tm_isdst = 0;
	return mktime(&timeInfo);
}

bool MassStorage::IsCardDetected(size_t card) const
{
	return info[card].cardState == CardDetectState::present;
}

// Unmount a file system returning the number of open files were invalidated
unsigned int MassStorage::InternalUnmount(size_t card, bool doClose)
{
	SdCardInfo& inf = info[card];
	MutexLocker lock1(fsMutex);
	MutexLocker lock2(inf.volMutex);
	const unsigned int invalidated = InvalidateFiles(&inf.fileSystem, doClose);
	const char path[3] = { (char)('0' + card), ':', 0 };
	f_mount(nullptr, path, 0);
	memset(&inf.fileSystem, 0, sizeof(inf.fileSystem));
	sd_mmc_unmount(card);
	inf.isMounted = false;
	return invalidated;
}

unsigned int MassStorage::GetNumFreeFiles() const
{
	unsigned int numFreeFiles = 0;
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (fil.usageMode == FileUseMode::free)
		{
			++numFreeFiles;
		}
	}
	return numFreeFiles;
}

void MassStorage::Spin()
{
	for (size_t card = 0; card < NumSdCards; ++card)
	{
		SdCardInfo& inf = info[card];
		if (inf.cdPin != NoPin)
		{
			if (digitalRead(inf.cdPin))
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

	// Check if any files are supposed to be closed
	{
		MutexLocker lock(fsMutex);
		for (FileStore & fil : files)
		{
			if (fil.closeRequested)
			{
				// We could not close this file in an ISR, so do it here
				fil.Close();
			}
		}
	}
}

// Append the simulated printing time to the end of the file
void MassStorage::RecordSimulationTime(const char *printingFilePath, uint32_t simSeconds)
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
MassStorage::InfoResult MassStorage::GetCardInfo(size_t slot, uint64_t& capacity, uint64_t& freeSpace, uint32_t& speed, uint32_t& clSize)
{
	if (slot >= NumSdCards)
	{
		return InfoResult::badSlot;
	}

	SdCardInfo& inf = info[slot];
	if (!inf.isMounted)
	{
		return InfoResult::noCard;
	}

	capacity = (uint64_t)sd_mmc_get_capacity(slot) * 1024;
	speed = sd_mmc_get_interface_speed(slot);
	String<ShortScratchStringLength> path;
	path.printf("%u:/", slot);
	uint32_t freeClusters;
	FATFS *fs;
	const FRESULT fr = f_getfree(path.c_str(), &freeClusters, &fs);
	if (fr == FR_OK)
	{
		clSize = fs->csize * 512;
		freeSpace = (fr == FR_OK) ? (uint64_t)freeClusters * clSize : 0;
	}
	else
	{
		clSize = 0;
		freeSpace = 0;
	}
	return InfoResult::ok;
}

#ifdef RTOS

// Functions called by FatFS to acquire/release mutual exclusion
extern "C"
{
	// Create a sync object. We already created it, we just need to copy the handle.
	int ff_cre_syncobj (BYTE vol, FF_SYNC_t* psy)
	{
		*psy = &reprap.GetPlatform().GetMassStorage()->GetVolumeMutex(vol);
		return 1;
	}

	// Lock sync object
	int ff_req_grant (FF_SYNC_t sy)
	{
		sy->Take();
		return 1;
	}

	// Unlock sync object
	void ff_rel_grant (FF_SYNC_t sy)
	{
		sy->Release();
	}

	// Delete a sync object
	int ff_del_syncobj (FF_SYNC_t sy)
	{
		return 1;		// nothing to do, we never delete the mutex
	}
}

#endif

// End
