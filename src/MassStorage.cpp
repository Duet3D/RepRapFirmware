#include "RepRapFirmware.h"

MassStorage::MassStorage(Platform* p) : platform(p)
{
	memset(&fileSystem, 0, sizeof(fileSystem));
}

void MassStorage::Init()
{
	// Initialize SD MMC stack
	sd_mmc_init();

	const size_t startTime = millis();
	sd_mmc_err_t err;
	do {
		err = sd_mmc_check(0);
		delay_ms(1);
	} while (err != SD_MMC_OK && millis() - startTime < 5000);

	if (err != SD_MMC_OK)
	{
		delay_ms(3000);		// Wait a few seconds so users have a chance to see this

		platform->Message(HOST_MESSAGE, "Cannot initialise the SD card: ");
		switch (err)
		{
			case SD_MMC_ERR_NO_CARD:
				platform->Message(HOST_MESSAGE, "Card not found\n");
				break;
			case SD_MMC_ERR_UNUSABLE:
				platform->Message(HOST_MESSAGE, "Card is unusable, try another one\n");
				break;
			case SD_MMC_ERR_SLOT:
				platform->Message(HOST_MESSAGE, "Slot unknown\n");
				break;
			case SD_MMC_ERR_COMM:
				platform->Message(HOST_MESSAGE, "General communication error\n");
				break;
			case SD_MMC_ERR_PARAM:
				platform->Message(HOST_MESSAGE, "Illegal input parameter\n");
				break;
			case SD_MMC_ERR_WP:
				platform->Message(HOST_MESSAGE, "Card write protected\n");
				break;
			default:
				platform->MessageF(HOST_MESSAGE, "Unknown (code %d)\n", err);
				break;
		}
		return;
	}

	// Print some card details (optional)

	/*platform->Message(HOST_MESSAGE, "SD card detected!\nCapacity: %d\n", sd_mmc_get_capacity(0));
	platform->AppendMessage(HOST_MESSAGE, "Bus clock: %d\n", sd_mmc_get_bus_clock(0));
	platform->AppendMessage(HOST_MESSAGE, "Bus width: %d\nCard type: ", sd_mmc_get_bus_width(0));
	switch (sd_mmc_get_type(0))
	{
		case CARD_TYPE_SD | CARD_TYPE_HC:
			platform->AppendMessage(HOST_MESSAGE, "SDHC\n");
			break;
		case CARD_TYPE_SD:
			platform->AppendMessage(HOST_MESSAGE, "SD\n");
			break;
		case CARD_TYPE_MMC | CARD_TYPE_HC:
			platform->AppendMessage(HOST_MESSAGE, "MMC High Density\n");
			break;
		case CARD_TYPE_MMC:
			platform->AppendMessage(HOST_MESSAGE, "MMC\n");
			break;
		case CARD_TYPE_SDIO:
			platform->AppendMessage(HOST_MESSAGE, "SDIO\n");
			return;
		case CARD_TYPE_SD_COMBO:
			platform->AppendMessage(HOST_MESSAGE, "SD COMBO\n");
			break;
		case CARD_TYPE_UNKNOWN:
		default:
			platform->AppendMessage(HOST_MESSAGE, "Unknown\n");
			return;
	}*/

	// Mount the file system

	FRESULT mounted = f_mount(0, &fileSystem);
	if (mounted != FR_OK)
	{
		platform->MessageF(HOST_MESSAGE, "Can't mount filesystem 0: code %d\n", mounted);
	}
}

const char* MassStorage::CombineName(const char* directory, const char* fileName)
{
	size_t out = 0;
	size_t in = 0;

	// DC 2015-11-25 Only prepend the directory if the filename does not have an absolute path
	if (directory != NULL && fileName[0] != '/' && (strlen(fileName) < 3 || !isdigit(fileName[0]) || fileName[1] != ':' || fileName[2] != '/'))
	{
		while (directory[in] != 0 && directory[in] != '\n')
		{
			combinedName[out] = directory[in];
			in++;
			out++;
			if (out >= ARRAY_SIZE(combinedName))
			{
				platform->MessageF(GENERIC_MESSAGE, "CombineName() buffer overflow.");
				out = 0;
			}
		}

		if (in > 0 && directory[in - 1] != '/' && out < ARRAY_UPB(combinedName))
		{
			combinedName[out] = '/';
			out++;
		}
		in = 0;
	}

	while (fileName[in] != 0 && fileName[in] != '\n')
	{
		combinedName[out] = fileName[in];
		in++;
		out++;
		if (out >= ARRAY_SIZE(combinedName))
		{
			platform->Message(GENERIC_MESSAGE, "CombineName() buffer overflow.");
			out = 0;
		}
	}
	combinedName[out] = 0;

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

// End
