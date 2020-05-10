//------------------------------------------------------------------------------------------------

#include "RepRapFirmware.h"

#if HAS_MASS_STORAGE

#include "FileStore.h"
#include "MassStorage.h"
#include "Platform.h"
#include "RepRap.h"
#include "Libraries/Fatfs/diskio.h"
#include "Movement/StepTimer.h"

FileStore::FileStore() noexcept : writeBuffer(nullptr)
{
	Init();
}

void FileStore::Init() noexcept
{
	usageMode = FileUseMode::free;
	openCount = 0;
	closeRequested = false;
}

// Invalidate the file if it uses the specified FATFS object
bool FileStore::Invalidate(const FATFS *fs, bool doClose) noexcept
{
	if (file.obj.fs == fs)
	{
		if (doClose)
		{
			(void)ForceClose();
		}
		else
		{
			file.obj.fs = nullptr;
			if (writeBuffer != nullptr)
			{
				MassStorage::ReleaseWriteBuffer(writeBuffer);
				writeBuffer = nullptr;
			}
		}
		usageMode = FileUseMode::invalidated;
		return true;
	}
	return false;
}

// Return true if the file is open on the specified file system
bool FileStore::IsOpenOn(const FATFS *fs) const noexcept
{
	return openCount != 0 && file.obj.fs == fs;
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.
bool FileStore::Open(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	const bool writing = (mode == OpenMode::write || mode == OpenMode::writeWithCrc || mode == OpenMode::append);
	writeBuffer = nullptr;

	if (writing)
	{
		if (!MassStorage::EnsurePath(filePath, true))
		{
			return false;
		}

		// Also try to allocate a write buffer so we can perform faster writes
		// We only do this if the mode is write, not append, because we don't want to use up a large buffer to append messages to the log file,
		// especially as we need to flush messages to SD card regularly.
		// Currently, append mode is used for the log file and for appending simulated print times to GCodes files (which required read access too).
		if (mode == OpenMode::write || mode == OpenMode::writeWithCrc)
		{
			writeBuffer = MassStorage::AllocateWriteBuffer();
		}
	}

	const FRESULT openReturn = f_open(&file, filePath,
										(mode == OpenMode::write || mode == OpenMode::writeWithCrc) ?  FA_CREATE_ALWAYS | FA_WRITE
											: (mode == OpenMode::append) ? FA_READ | FA_WRITE | FA_OPEN_ALWAYS
												: FA_OPEN_EXISTING | FA_READ);
	if (openReturn != FR_OK)
	{
		if (writeBuffer != nullptr)
		{
			MassStorage::ReleaseWriteBuffer(writeBuffer);
			writeBuffer = nullptr;
		}

		// We no longer report an error if opening a file in read mode fails unless debugging is enabled, because sometimes that is quite normal.
		// It is up to the caller to report an error if necessary.
		if (reprap.Debug(modulePlatform))
		{
			reprap.GetPlatform().MessageF(WarningMessage, "Failed to open %s to %s, error code %d\n", filePath, (writing) ? "write" : "read", (int)openReturn);
		}
		return false;
	}

	crc.Reset();
	calcCrc = (mode == OpenMode::writeWithCrc);
	usageMode = (writing) ? FileUseMode::readWrite : FileUseMode::readOnly;
	openCount = 1;
#ifndef __LPC17xx__
	if (preAllocSize != 0 && (mode == OpenMode::write || mode == OpenMode::writeWithCrc))
	{
		const FRESULT expandReturn = f_expand(&file, preAllocSize, 1);		// try to pre-allocate contiguous space - it doesn't matter if it fails
		if (reprap.Debug(moduleStorage))
		{
			debugPrintf("Preallocating %" PRIu32 " bytes returned %d\n", preAllocSize, (int)expandReturn);
		}
	}
#endif
	reprap.VolumesUpdated();
	return true;
}

void FileStore::Duplicate() noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
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
bool FileStore::Close() noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		if (!inInterrupt())
		{
			REPORT_INTERNAL_ERROR;
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

bool FileStore::ForceClose() noexcept
{
	bool ok = true;
	if (usageMode == FileUseMode::readWrite)
	{
		ok = Flush();
	}

	if (writeBuffer != nullptr)
	{
		MassStorage::ReleaseWriteBuffer(writeBuffer);
		writeBuffer = nullptr;
	}

	const FRESULT fr = f_close(&file);
	usageMode = FileUseMode::free;
	closeRequested = false;
	openCount = 0;
	reprap.VolumesUpdated();
	return ok && fr == FR_OK;
}

bool FileStore::Seek(FilePosition pos) noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
		return false;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		return f_lseek(&file, pos) == FR_OK;

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

FilePosition FileStore::Position() const noexcept
{
	return (usageMode == FileUseMode::readOnly || usageMode == FileUseMode::readWrite) ? file.fptr : 0;
}

uint32_t FileStore::ClusterSize() const noexcept
{
	return (usageMode == FileUseMode::readOnly || usageMode == FileUseMode::readWrite) ? file.obj.fs->csize * 512u : 1;	// we divide by the cluster size so return 1 not 0 if there is an error
}

#if 0	// not currently used
bool FileStore::GoToEnd()
{
	return Seek(Length());
}
#endif

FilePosition FileStore::Length() const noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
		return 0;

	case FileUseMode::readOnly:
		return f_size(&file);

	case FileUseMode::readWrite:
		return (writeBuffer != nullptr) ? f_size(&file) + writeBuffer->BytesStored() : f_size(&file);

	case FileUseMode::invalidated:
	default:
		return 0;
	}
}

// Single character read
bool FileStore::Read(char& b) noexcept
{
	return Read(&b, sizeof(char));
}

// Returns the number of bytes read or -1 if the read process failed
int FileStore::Read(char* extBuf, size_t nBytes) noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
		return -1;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
		{
			UINT bytes_read;
			FRESULT readStatus = f_read(&file, extBuf, nBytes, &bytes_read);
			if (readStatus != FR_OK)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Cannot read file, error code %d\n", (int)readStatus);
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
// Return the number of characters in the line excluding the null terminator, or -1 if end of file or a read error occurs.
int FileStore::ReadLine(char* buf, size_t nBytes) noexcept
{
	const FilePosition lineStart = Position();
	const int r = Read(buf, nBytes);
	if (r <= 0)
	{
		return -1;
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

FRESULT FileStore::Store(const char *s, size_t len, size_t *bytesWritten) noexcept
{
	if (calcCrc)
	{
		crc.Update(s, len);
	}
	const FRESULT writeStatus = f_write(&file, s, len, bytesWritten);
	return writeStatus;
}

bool FileStore::Write(char b) noexcept
{
	return Write(&b, sizeof(char));
}

bool FileStore::Write(const char* b) noexcept
{
	return Write(b, strlen(b));
}

bool FileStore::Write(const char *s, size_t len) noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
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
		return false;
	}
}

bool FileStore::Flush() noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
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
bool FileStore::Truncate() noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
	case FileUseMode::readOnly:
		REPORT_INTERNAL_ERROR;
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

// Return true if the passed file is the same as ours
bool FileStore::IsSameFile(const FIL& otherFile) const noexcept
{
	return file.obj.fs == otherFile.obj.fs && file.dir_sect == otherFile.dir_sect && file.dir_ptr == otherFile.dir_ptr;
}

#if 0	// not currently used

// Provide a cluster map for fast seeking. Needs _USE_FASTSEEK defined as 1 in conf_fatfs to make any difference.
// The first element of the table must be set to the total number of 32-bit entries in the table before calling this.
bool FileStore::SetClusterMap(uint32_t tbl[]) noexcept
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

#endif

// End
