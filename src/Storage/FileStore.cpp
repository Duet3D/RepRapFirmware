//------------------------------------------------------------------------------------------------

#include "RepRapFirmware.h"
#include "FileStore.h"
#include "MassStorage.h"
#include "Platform.h"
#include "RepRap.h"

uint32_t FileStore::longestWriteTime = 0;

FileStore::FileStore() : writeBuffer(nullptr)
{
	Init();
}

void FileStore::Init()
{
	usageMode = FileUseMode::free;
	openCount = 0;
	closeRequested = false;
}

// Invalidate the file if it uses the specified FATFS object
bool FileStore::Invalidate(const FATFS *fs, bool doClose)
{
	if (file.fs == fs)
	{
		if (doClose)
		{
			(void)ForceClose();
		}
		else
		{
			file.fs = nullptr;
			if (writeBuffer != nullptr)
			{
				reprap.GetPlatform().GetMassStorage()->ReleaseWriteBuffer(writeBuffer);
				writeBuffer = nullptr;
			}
		}
		usageMode = FileUseMode::invalidated;
		return true;
	}
	return false;
}

// Return true if the file is open on the specified file system
bool FileStore::IsOpenOn(const FATFS *fs) const
{
	return openCount != 0 && file.fs == fs;
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.
bool FileStore::Open(const char* directory, const char* fileName, OpenMode mode)
{
	String<MaxFilenameLength> location;
	MassStorage::CombineName(location.GetRef(), directory, fileName);
	const bool writing = (mode == OpenMode::write || mode == OpenMode::append);

	if (writing)
	{
		// Try to create the path of this file if we want to write to it
		String<MaxFilenameLength> filePath;
		filePath.copy(location.c_str());

		size_t i = (isdigit(filePath[0]) && filePath[1] == ':') ? 2 : 0;
		if (filePath[i] == '/')
		{
			++i;
		}

		while (i < filePath.strlen())
		{
			if (filePath[i] == '/')
			{
				filePath[i] = 0;
				if (!reprap.GetPlatform().GetMassStorage()->DirectoryExists(filePath.GetRef()) && !reprap.GetPlatform().GetMassStorage()->MakeDirectory(filePath.c_str()))
				{
					reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create folder %s while trying to open file %s\n", filePath.c_str(), location.c_str());
					return false;
				}
				filePath[i] = '/';
			}
			++i;
		}

		// Also try to allocate a write buffer so we can perform faster writes
		// We only do this if the mode is write, not append, because we don't want to use up a large buffer to append messages to the log file,
		// especially as we need to flush messages to SD card regularly.
		// Currently, append mode is used for the log file and for appending simulated print times to GCodes files (which required read access too).
		writeBuffer = (mode == OpenMode::write) ? reprap.GetPlatform().GetMassStorage()->AllocateWriteBuffer() : nullptr;
	}

	const FRESULT openReturn = f_open(&file, location.c_str(),
										(mode == OpenMode::write) ?  FA_CREATE_ALWAYS | FA_WRITE
											: (mode == OpenMode::append) ? FA_READ | FA_WRITE | FA_OPEN_ALWAYS
												: FA_OPEN_EXISTING | FA_READ);
	if (openReturn != FR_OK)
	{
		// We no longer report an error if opening a file in read mode fails unless debugging is enabled, because sometimes that is quite normal.
		// It is up to the caller to report an error if necessary.
		if (reprap.Debug(modulePlatform))
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Can't open %s to %s, error code %d\n", location.c_str(), (writing) ? "write" : "read", (int)openReturn);
		}
		return false;
	}
	crc.Reset();
	usageMode = (writing) ? FileUseMode::readWrite : FileUseMode::readOnly;
	openCount = 1;
	return true;
}

void FileStore::Duplicate()
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		break;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		{
			const irqflags_t flags = cpu_irq_save();
			++openCount;
			cpu_irq_restore(flags);
		}
		break;

	case FileUseMode::invalidated:
	default:
		break;
	}
}

// This may be called from an ISR, in which case we need to defer the close
bool FileStore::Close()
{
	switch (usageMode)
	{
	case FileUseMode::free:
		if (!inInterrupt())
		{
			INTERNAL_ERROR;
		}
		return false;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		{
			const irqflags_t flags = cpu_irq_save();
			if (openCount > 1)
			{
				--openCount;
				cpu_irq_restore(flags);
				return true;
			}
			else if (inInterrupt())
			{
				closeRequested = true;
				cpu_irq_restore(flags);
				return true;
			}
			else
			{
				cpu_irq_restore(flags);
				return ForceClose();
			}
		}

	case FileUseMode::invalidated:
	default:
		{
			const irqflags_t flags = cpu_irq_save();
			if (openCount > 1)
			{
				--openCount;
			}
			else
			{
				usageMode = FileUseMode::free;
			}
			cpu_irq_restore(flags);
			return true;
		}
	}
}

bool FileStore::ForceClose()
{
	bool ok = true;
	if (usageMode == FileUseMode::readWrite)
	{
		ok = Flush();
	}

	if (writeBuffer != nullptr)
	{
		reprap.GetPlatform().GetMassStorage()->ReleaseWriteBuffer(writeBuffer);
		writeBuffer = nullptr;
	}

	const FRESULT fr = f_close(&file);
	usageMode = FileUseMode::free;
	closeRequested = false;
	openCount = 0;
	return ok && fr == FR_OK;
}

bool FileStore::Seek(FilePosition pos)
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		return false;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		return f_lseek(&file, pos) == FR_OK;

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

FilePosition FileStore::Position() const
{
	return (usageMode == FileUseMode::readOnly || usageMode == FileUseMode::readWrite) ? file.fptr : 0;
}

uint32_t FileStore::ClusterSize() const
{
	return (usageMode == FileUseMode::readOnly || usageMode == FileUseMode::readWrite) ? file.fs->csize * 512u : 1;	// we divide by the cluster size so return 1 not 0 if there is an error
}

#if 0	// not currently used
bool FileStore::GoToEnd()
{
	return Seek(Length());
}
#endif

FilePosition FileStore::Length() const
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		return 0;

	case FileUseMode::readOnly:
		return file.fsize;

	case FileUseMode::readWrite:
		return (writeBuffer != nullptr) ? file.fsize + writeBuffer->BytesStored() : file.fsize;

	case FileUseMode::invalidated:
	default:
		return 0;
	}
}

// Single character read
bool FileStore::Read(char& b)
{
	return Read(&b, sizeof(char));
}

// Returns the number of bytes read or -1 if the read process failed
int FileStore::Read(char* extBuf, size_t nBytes)
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		return -1;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		{
			UINT bytes_read;
			FRESULT readStatus = f_read(&file, extBuf, nBytes, &bytes_read);
			if (readStatus != FR_OK)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Cannot read file, error code %d.\n", (int)readStatus);
				return -1;
			}
			return (int)bytes_read;
		}

	case FileUseMode::invalidated:
	default:
		return -1;
	}
}

// As Read but stop after '\n' or '\r\n' and null-terminate the string.
// If the next line is too long to fit in the buffer then the line will be split.
int FileStore::ReadLine(char* buf, size_t nBytes)
{
	const FilePosition lineStart = Position();
	const int r = Read(buf, nBytes);
	if (r < 0)
	{
		return r;
	}

	int i = 0;
	while (i < r && buf[i] != '\r' && buf[i] != '\n')
	{
		++i;
	}

	if (i + 1 < r && buf[i] == '\r' && buf[i + 1] == '\n')	// if stopped at CRLF (Windows-style line end)
	{
		Seek(lineStart + i + 2);							// seek to just after the CRLF
	}
	else if (i < r)											// if stopped at CR or LF
	{
		Seek(lineStart + i + 1);							// seek to just after the CR or LF
	}
	else if (i == (int)nBytes)
	{
		--i;												// make room for the null terminator
		Seek(lineStart + i);
	}
	buf[i] = 0;
	return i;
}

FRESULT FileStore::Store(const char *s, size_t len, size_t *bytesWritten)
{
	uint32_t time = Platform::GetInterruptClocks();
	crc.Update(s, len);
	const FRESULT writeStatus = f_write(&file, s, len, bytesWritten);
	time = Platform::GetInterruptClocks() - time;
	if (time > longestWriteTime)
	{
		longestWriteTime = time;
	}
	return writeStatus;
}

bool FileStore::Write(char b)
{
	return Write(&b, sizeof(char));
}

bool FileStore::Write(const char* b)
{
	return Write(b, strlen(b));
}

bool FileStore::Write(const char *s, size_t len)
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		return false;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		{
			size_t totalBytesWritten = 0;
			FRESULT writeStatus = FR_OK;
			if (writeBuffer == nullptr)
			{
				writeStatus = Store(s, len, &totalBytesWritten);
			}
			else
			{
				do
				{
					size_t bytesStored = writeBuffer->Store(s + totalBytesWritten, len - totalBytesWritten);
					if (writeBuffer->BytesLeft() == 0)
					{
						const size_t bytesToWrite = writeBuffer->BytesStored();
						size_t bytesWritten;
						writeStatus = Store(writeBuffer->Data(), bytesToWrite, &bytesWritten);
						writeBuffer->DataTaken();

						if (bytesToWrite != bytesWritten)
						{
							// Something went wrong
							break;
						}
					}
					totalBytesWritten += bytesStored;
				}
				while (writeStatus == FR_OK && totalBytesWritten != len);
			}

			if ((writeStatus != FR_OK) || (totalBytesWritten != len))
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Failed to write to file, error code %d. Card may be full.\n", (int)writeStatus);
				return false;
			}
			return true;
		}

	case FileUseMode::invalidated:
	default:
		return 0;
	}
}

bool FileStore::Flush()
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		return false;

	case FileUseMode::readOnly:
		return true;

	case FileUseMode::readWrite:
		if (writeBuffer != nullptr)
		{
			const size_t bytesToWrite = writeBuffer->BytesStored();
			if (bytesToWrite != 0)
			{
				size_t bytesWritten;
				const FRESULT writeStatus = Store(writeBuffer->Data(), bytesToWrite, &bytesWritten);
				writeBuffer->DataTaken();

				if ((writeStatus != FR_OK) || (bytesToWrite != bytesWritten))
				{
					reprap.GetPlatform().MessageF(ErrorMessage, "Failed to flush data to file, error code %d. Card may be full.\n", (int)writeStatus);
					return false;
				}
			}
		}
		return f_sync(&file) == FR_OK;

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

// Truncate file at current file pointer
bool FileStore::Truncate()
{
	switch (usageMode)
	{
	case FileUseMode::free:
	case FileUseMode::readOnly:
		INTERNAL_ERROR;
		return false;

	case FileUseMode::readWrite:
		if (!Flush())
		{
			return false;
		}
		return f_truncate(&file) == FR_OK;

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

// Return the file write time in milliseconds, and clear it
float FileStore::GetAndClearLongestWriteTime()
{
	const float ret = (float)longestWriteTime * StepClocksToMillis;
	longestWriteTime = 0;
	return ret;
}

#if 0	// not currently used

// Provide a cluster map for fast seeking. Needs _USE_FASTSEEK defined as 1 in conf_fatfs to make any difference.
// The first element of the table must be set to the total number of 32-bit entries in the table before calling this.
bool FileStore::SetClusterMap(uint32_t tbl[])
{
	switch (usageMode)
	{
	case FileUseMode::free:
		INTERNAL_ERROR;
		return false;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		{
			file.cltbl = tbl;
			const FRESULT ret = f_lseek(&file, CREATE_LINKMAP);
//			debugPrintf("ret %d need %u\n", (int)ret, tbl[0]);
			return ret == FR_OK;
		}

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

#endif

// End
