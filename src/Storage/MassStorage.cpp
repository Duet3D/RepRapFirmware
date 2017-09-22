#include "MassStorage.h"
#include "Platform.h"
#include "RepRap.h"
#include "sd_mmc.h"

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
MassStorage::MassStorage(Platform* p) : platform(p)
{
	memset(&fileSystems, 0, sizeof(fileSystems));
}

void MassStorage::Init()
{
	freeWriteBuffers = nullptr;
	for (size_t i = 0; i < NumFileWriteBuffers; ++i)
	{
		freeWriteBuffers = new FileWriteBuffer(freeWriteBuffers);
	}

	for (size_t i = 0; i < NumSdCards; ++i)
	{
		isMounted[i] = false;
	}

	sd_mmc_init(SdCardDetectPins, SdWriteProtectPins, SdSpiCSPins);		// Initialize SD MMC stack

	// Try to mount the first SD card only
	char replyBuffer[100];
	StringRef reply(replyBuffer, ARRAY_SIZE(replyBuffer));
	do { } while (!Mount(0, reply, false));
	if (reply.strlen() != 0)
	{
		delay(3000);		// Wait a few seconds so users have a chance to see this
		platform->Message(UsbMessage, reply.Pointer());
	}
}

FileWriteBuffer *MassStorage::AllocateWriteBuffer()
{
	if (freeWriteBuffers == nullptr)
	{
		return nullptr;
	}

	FileWriteBuffer *buffer = freeWriteBuffers;
	freeWriteBuffers = buffer->Next();
	buffer->SetNext(nullptr);
	return buffer;
}

void MassStorage::ReleaseWriteBuffer(FileWriteBuffer *buffer)
{
	buffer->SetNext(freeWriteBuffers);
	freeWriteBuffers = buffer;
}

const char* MassStorage::CombineName(const char* directory, const char* fileName)
{
	size_t outIndex = 0;
	size_t inIndex = 0;

	// DC 2015-11-25 Only prepend the directory if the filename does not have an absolute path or volume specifier
	if (directory != nullptr && fileName[0] != '/' && (strlen(fileName) < 2 || !isdigit(fileName[0]) || fileName[1] != ':'))
	{
		while (directory[inIndex] != 0 && directory[inIndex] != '\n')
		{
			combinedName[outIndex] = directory[inIndex];
			inIndex++;
			outIndex++;
			if (outIndex >= ARRAY_SIZE(combinedName))
			{
				platform->MessageF(ErrorMessage, "CombineName() buffer overflow");
				outIndex = 0;
			}
		}

		if (inIndex > 0 && directory[inIndex - 1] != '/' && outIndex < ARRAY_UPB(combinedName))
		{
			combinedName[outIndex] = '/';
			outIndex++;
		}
		inIndex = 0;
	}

	while (fileName[inIndex] != 0 && fileName[inIndex] != '\n')
	{
		combinedName[outIndex] = fileName[inIndex];
		inIndex++;
		outIndex++;
		if (outIndex >= ARRAY_SIZE(combinedName))
		{
			platform->Message(ErrorMessage, "CombineName() buffer overflow");
			outIndex = 0;
		}
	}
	combinedName[outIndex] = 0;

	return combinedName;
}

// Open a directory to read a file list. Returns true if it contains any files, false otherwise.
bool MassStorage::FindFirst(const char *directory, FileInfo &file_info)
{
	TCHAR loc[FILENAME_LENGTH + 1];

	// Remove the trailing '/' from the directory name
	SafeStrncpy(loc, directory, ARRAY_SIZE(loc));
	const size_t len = strlen(loc);
	if (len != 0 && loc[len - 1] == '/')
	{
		loc[len - 1] = 0;
	}

	findDir.lfn = nullptr;
	FRESULT res = f_opendir(&findDir, loc);
	if (res == FR_OK)
	{
		FILINFO entry;
		entry.lfname = file_info.fileName;
		entry.lfsize = ARRAY_SIZE(file_info.fileName);

		for(;;)
		{
			res = f_readdir(&findDir, &entry);
			if (res != FR_OK || entry.fname[0] == 0) break;
			if (StringEquals(entry.fname, ".") || StringEquals(entry.fname, "..")) continue;

			file_info.isDirectory = (entry.fattrib & AM_DIR);

			if (file_info.fileName[0] == 0)
			{
				SafeStrncpy(file_info.fileName, entry.fname, ARRAY_SIZE(file_info.fileName));
			}

			file_info.size = entry.fsize;
			file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);

			return true;
		}
	}

	return false;
}

// Find the next file in a directory. Returns true if another file has been read.
bool MassStorage::FindNext(FileInfo &file_info)
{
	FILINFO entry;
	entry.lfname = file_info.fileName;
	entry.lfsize = ARRAY_SIZE(file_info.fileName);

	findDir.lfn = nullptr;
	if (f_readdir(&findDir, &entry) != FR_OK || entry.fname[0] == 0)
	{
		//f_closedir(findDir);
		return false;
	}

	file_info.isDirectory = (entry.fattrib & AM_DIR);
	file_info.size = entry.fsize;

	if (file_info.fileName[0] == 0)
	{
		SafeStrncpy(file_info.fileName, entry.fname, ARRAY_SIZE(file_info.fileName));
	}

	file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);
	return true;
}

// Month names. The first entry is used for invalid month numbers.
static const char *monthNames[13] = { "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

// Returns the name of the specified month or '???' if the specified value is invalid.
const char* MassStorage::GetMonthName(const uint8_t month)
{
	return (month <= 12) ? monthNames[month] : monthNames[0];
}

// Delete a file or directory
bool MassStorage::Delete(const char* directory, const char* fileName, bool silent)
{
	const char* location = (directory != nullptr)
							? platform->GetMassStorage()->CombineName(directory, fileName)
								: fileName;
	if (f_unlink(location) != FR_OK)
	{
		if (!silent)
		{
			platform->MessageF(ErrorMessage, "Failed to delete file %s\n", location);
		}
		return false;
	}
	return true;
}

// Create a new directory
bool MassStorage::MakeDirectory(const char *parentDir, const char *dirName)
{
	const char* location = platform->GetMassStorage()->CombineName(parentDir, dirName);
	if (f_mkdir(location) != FR_OK)
	{
		platform->MessageF(ErrorMessage, "Failed to create directory %s\n", location);
		return false;
	}
	return true;
}

bool MassStorage::MakeDirectory(const char *directory)
{
	if (f_mkdir(directory) != FR_OK)
	{
		platform->MessageF(ErrorMessage, "Failed to create directory %s\n", directory);
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
		platform->MessageF(ErrorMessage, "Failed to rename file or directory %s to %s\n", oldFilename, newFilename);
		return false;
	}
	return true;
}

// Check if the specified file exists
bool MassStorage::FileExists(const char *file) const
{
	FILINFO fil;
	fil.lfname = nullptr;
	return (f_stat(file, &fil) == FR_OK);
}

bool MassStorage::FileExists(const char *directory, const char *fileName) const
{
	const char *location = (directory != nullptr)
							? platform->GetMassStorage()->CombineName(directory, fileName)
							: fileName;
	return FileExists(location);
}

// Check if the specified directory exists
bool MassStorage::DirectoryExists(const char *path) const
{
	DIR dir;
	dir.lfn = nullptr;
	return (f_opendir(&dir, path) == FR_OK);
}

bool MassStorage::DirectoryExists(const char* directory, const char* subDirectory)
{
	return DirectoryExists(CombineName(directory, subDirectory));
}

// Return the last modified time of a file, or zero if failure
time_t MassStorage::GetLastModifiedTime(const char* directory, const char *fileName) const
{
	const char *location = (directory != nullptr)
							? platform->GetMassStorage()->CombineName(directory, fileName)
							: fileName;
	FILINFO fil;
	fil.lfname = nullptr;
	if (f_stat(location, &fil) == FR_OK)
	{
		return ConvertTimeStamp(fil.fdate, fil.ftime);
	}
	return 0;
}

bool MassStorage::SetLastModifiedTime(const char* directory, const char *fileName, time_t time)
{
	const char *location = (directory != nullptr)
							? platform->GetMassStorage()->CombineName(directory, fileName)
							: fileName;
	const struct tm * const timeInfo = gmtime(&time);
	FILINFO fno;
    fno.fdate = (WORD)(((timeInfo->tm_year - 80) * 512U) | (timeInfo->tm_mon + 1) * 32U | timeInfo->tm_mday);
    fno.ftime = (WORD)(timeInfo->tm_hour * 2048U | timeInfo->tm_min * 32U | timeInfo->tm_sec / 2U);
    const bool ok = (f_utime(location, &fno) == FR_OK);
    if (!ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to set last modified time for file '%s'\n", location);
	}
    return ok;
}

// Mount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
// This may only be called to mount one card at a time.
bool MassStorage::Mount(size_t card, StringRef& reply, bool reportSuccess)
{
	if (card >= NumSdCards)
	{
		reply.copy("SD card number out of range");
		return true;
	}

	if (isMounted[card] && platform->AnyFileOpen(&fileSystems[card]))
	{
		// Don't re-mount the card if any files are open on it
		reply.copy("SD card has open file(s)");
		return true;
	}

	static bool mounting = false;
	static size_t startTime;

	if (!mounting)
	{
		f_mount(card, nullptr);			// un-mount it from FATFS
		sd_mmc_unmount(card);			// this forces it to re-initialise the card
		isMounted[card] = false;
		startTime = millis();
		mounting = true;
		delay(2);
	}

	sd_mmc_err_t err = sd_mmc_check(card);
	if (err != SD_MMC_OK && millis() - startTime < 5000)
	{
		delay(2);
		return false;
	}

	mounting = false;
	if (err != SD_MMC_OK)
	{
		reply.printf("Cannot initialise SD card %u: %s", card, TranslateCardError(err));
	}
	else
	{
		// Mount the file systems
		memset(&fileSystems[card], 0, sizeof(FATFS));					// f_mount doesn't initialise the file structure, we must do it ourselves
		FRESULT mounted = f_mount(card, &fileSystems[card]);
		if (mounted != FR_OK)
		{
			reply.printf("Cannot mount SD card %u: code %d", card, mounted);
		}
		else
		{
			isMounted[card] = true;
			if (reportSuccess)
			{
				float capacity = sd_mmc_get_capacity(card)/1024;		// get capacity and convert to Mbytes
				const char* capUnits;
				if (capacity >= 1024.0)
				{
					capacity /= 1024.0;
					capUnits = "Gb";
				}
				else
				{
					capUnits = "Mb";
				}
				reply.printf("%s card mounted in slot %u, capacity %.2f%s", TranslateCardType(sd_mmc_get_type(card)), card, (double)capacity, capUnits);
			}
			else
			{
				reply.Clear();
			}
		}
	}
	return true;
}

// Unmount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
bool MassStorage::Unmount(size_t card, StringRef& reply)
{
	if (card >= NumSdCards)
	{
		reply.copy("SD card number out of range");
		return true;
	}

	platform->InvalidateFiles(&fileSystems[card]);
	f_mount(card, nullptr);
	sd_mmc_unmount(card);
	isMounted[card] = false;
	reply.Clear();
	return true;
}

// Check if the drive referenced in the specified path is mounted. Return true if it is.
// Ideally we would try to mount it if it is not, however mounting a drive can take a long time, and the functions that call this are expected to execute quickly.
bool MassStorage::CheckDriveMounted(const char* path)
{
	unsigned int card;
	if (strlen(path) >= 2 && isDigit(path[0]) && path[1] == ':')
	{
		card = path[0] - '0';
	}
	else
	{
		card = 0;
	}

	return card < NumSdCards && isMounted[card];
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

// End
