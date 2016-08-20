#include "RepRapFirmware.h"
#include "sd_mmc.h"

MassStorage::MassStorage(Platform* p) : platform(p)
{
	memset(&fileSystems, 0, sizeof(fileSystems));
}

void MassStorage::Init()
{
	sd_mmc_init(SdCardDetectPins, SdWriteProtectPins, SdSpiCSPins);		// Initialize SD MMC stack

	// Try to mount the first SD card only
	char replyBuffer[100];
	StringRef reply(replyBuffer, ARRAY_SIZE(replyBuffer));
	do { } while (!Mount(0, reply));
	if (reply.strlen() != 0)
	{
		delay(3000);		// Wait a few seconds so users have a chance to see this
		platform->Message(HOST_MESSAGE, reply.Pointer());
	}
}

const char* MassStorage::CombineName(const char* directory, const char* fileName)
{
	size_t outIndex = 0;
	size_t inIndex = 0;

	// DC 2015-11-25 Only prepend the directory if the filename does not have an absolute path
	if (directory != NULL && fileName[0] != '/' && (strlen(fileName) < 3 || !isdigit(fileName[0]) || fileName[1] != ':' || fileName[2] != '/'))
	{
		while (directory[inIndex] != 0 && directory[inIndex] != '\n')
		{
			combinedName[outIndex] = directory[inIndex];
			inIndex++;
			outIndex++;
			if (outIndex >= ARRAY_SIZE(combinedName))
			{
				platform->MessageF(GENERIC_MESSAGE, "CombineName() buffer overflow.");
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
			platform->Message(GENERIC_MESSAGE, "CombineName() buffer overflow.");
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
	size_t len = strnlen(directory, ARRAY_UPB(loc));
	if (len == 0)
	{
		loc[0] = 0;
	}
	else if (directory[len - 1] == '/')
	{
		strncpy(loc, directory, len - 1);
		loc[len - 1] = 0;
	}
	else
	{
		strncpy(loc, directory, len);
		loc[len] = 0;
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
			file_info.size = entry.fsize;
			uint16_t day = entry.fdate & 0x1F;
			if (day == 0)
			{
				// This can happen if a transfer hasn't been processed completely.
				day = 1;
			}
			file_info.day = day;
			file_info.month = (entry.fdate & 0x01E0) >> 5;
			file_info.year = (entry.fdate >> 9) + 1980;
			if (file_info.fileName[0] == 0)
			{
				strncpy(file_info.fileName, entry.fname, ARRAY_SIZE(file_info.fileName));
			}

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
	uint16_t day = entry.fdate & 0x1F;
	if (day == 0)
	{
		// This can happen if a transfer hasn't been processed completely.
		day = 1;
	}
	file_info.day = day;
	file_info.month = (entry.fdate & 0x01E0) >> 5;
	file_info.year = (entry.fdate >> 9) + 1980;
	if (file_info.fileName[0] == 0)
	{
		strncpy(file_info.fileName, entry.fname, ARRAY_SIZE(file_info.fileName));
	}

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
bool MassStorage::Delete(const char* directory, const char* fileName)
{
	const char* location = (directory != NULL)
							? platform->GetMassStorage()->CombineName(directory, fileName)
								: fileName;
	if (f_unlink(location) != FR_OK)
	{
		platform->MessageF(GENERIC_MESSAGE, "Can't delete file %s\n", location);
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
		platform->MessageF(GENERIC_MESSAGE, "Can't create directory %s\n", location);
		return false;
	}
	return true;
}

bool MassStorage::MakeDirectory(const char *directory)
{
	if (f_mkdir(directory) != FR_OK)
	{
		platform->MessageF(GENERIC_MESSAGE, "Can't create directory %s\n", directory);
		return false;
	}
	return true;
}

// Rename a file or directory
bool MassStorage::Rename(const char *oldFilename, const char *newFilename)
{
	if (f_rename(oldFilename, newFilename) != FR_OK)
	{
		platform->MessageF(GENERIC_MESSAGE, "Can't rename file or directory %s to %s\n", oldFilename, newFilename);
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

// Mount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
// This may only be called to mount one card at a time.
bool MassStorage::Mount(size_t card, StringRef& reply)
{
	if (card >= NumSdCards)
	{
		reply.copy("SD card number out of range");
		return true;
	}

	static bool mounting = false;
	static size_t startTime;

	if (!mounting)
	{
		sd_mmc_unmount(card);			// this forces it to re-initialise the card
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
		f_mount(card, NULL);

		reply.printf("Cannot initialise SD card %u: ", card);
		switch (err)
		{
			case SD_MMC_ERR_NO_CARD:
				reply.cat("Card not found");
				break;
			case SD_MMC_ERR_UNUSABLE:
				reply.cat("Card is unusable, try another one");
				break;
			case SD_MMC_ERR_SLOT:
				reply.cat("Slot unknown");
				break;
			case SD_MMC_ERR_COMM:
				reply.cat("General communication error");
				break;
			case SD_MMC_ERR_PARAM:
				reply.cat("Illegal input parameter");
				break;
			case SD_MMC_ERR_WP:
				reply.cat("Card write protected");
				break;
			default:
				reply.catf("Unknown (code %d)", err);
				break;
		}
	}
	else
	{
		if (reprap.Debug(moduleStorage))
		{
			// Print some card details
			platform->MessageF(DEBUG_MESSAGE, "SD card %u detected, capacity: %u\n", card, sd_mmc_get_capacity(card));
			platform->Message(DEBUG_MESSAGE, "Card type: ");
			switch (sd_mmc_get_type(card))
			{
				case CARD_TYPE_SD | CARD_TYPE_HC:
					platform->Message(DEBUG_MESSAGE, "SDHC\n");
					break;
				case CARD_TYPE_SD:
					platform->Message(DEBUG_MESSAGE, "SD\n");
					break;
				case CARD_TYPE_MMC | CARD_TYPE_HC:
					platform->Message(DEBUG_MESSAGE, "MMC High Density\n");
					break;
				case CARD_TYPE_MMC:
					platform->Message(DEBUG_MESSAGE, "MMC\n");
					break;
				case CARD_TYPE_SDIO:
					platform->Message(DEBUG_MESSAGE, "SDIO\n");
					break;
				case CARD_TYPE_SD_COMBO:
					platform->Message(DEBUG_MESSAGE, "SD COMBO\n");
					break;
				case CARD_TYPE_UNKNOWN:
				default:
					platform->Message(DEBUG_MESSAGE, "Unknown\n");
					break;
			}
		}

		// Mount the file systems
		FRESULT mounted = f_mount(card, &fileSystems[card]);
		if (mounted != FR_OK)
		{
			reply.printf("Can't mount SD card %u: code %d\n", card, mounted);
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
	f_mount(card, NULL);
	sd_mmc_unmount(card);
	return true;
}

// End
