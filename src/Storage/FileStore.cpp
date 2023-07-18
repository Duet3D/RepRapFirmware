//------------------------------------------------------------------------------------------------

#include "RepRapFirmware.h"

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

#include <Platform/RepRap.h>
#include <Platform/Platform.h>

#include "FileStore.h"

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
# include "MassStorage.h"
#endif

#if HAS_MASS_STORAGE
# include <Libraries/Fatfs/diskio.h>
# include <Movement/StepTimer.h>
#endif

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

FileStore::FileStore() noexcept
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	: writeBuffer(nullptr)
#endif
{
	Init();
}

void FileStore::Init() noexcept
{
	usageMode = FileUseMode::free;
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	openCount = 0;
	closeRequested = false;
#endif
#if HAS_EMBEDDED_FILES
	fileIndex = -1;
#endif
#if HAS_SBC_INTERFACE
	handle = noFileHandle;
	length = 0;
#endif
#if HAS_EMBEDDED_FILES || HAS_SBC_INTERFACE
	offset = 0;
#endif
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.
bool FileStore::Open(const char *_ecv_array filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	const bool writing = (mode == OpenMode::write || mode == OpenMode::writeWithCrc || mode == OpenMode::append);
#if HAS_EMBEDDED_FILES
# if HAS_SBC_INTERFACE
	if (!reprap.UsingSbcInterface())
# endif
	{
		if (!writing)
		{
			fileIndex = EmbeddedFiles::OpenFile(filePath);
			if (fileIndex >= 0)
			{
				offset = 0;
				usageMode = FileUseMode::readOnly;
				openCount = 1;
				return true;
			}
		}

		// We no longer report an error if opening a file in read mode fails unless debugging is enabled, because sometimes that is quite normal.
		// It is up to the caller to report an error if necessary.
		if (reprap.Debug(Module::Storage))
		{
			reprap.GetPlatform().MessageF(WarningMessage, "Failed to open %s to %sn", filePath, (writing) ? "write" : "read");
		}
		return false;
	}
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	writeBuffer = nullptr;

	// Try to allocate a write buffer
	if (writing)
	{
# if HAS_MASS_STORAGE
		if (!MassStorage::EnsurePath(filePath, true))
		{
			return false;
		}
# endif

		// Also try to allocate a write buffer so we can perform faster writes
		// We only do this if the mode is write, not append, because we don't want to use up a large buffer to append messages to the log file,
		// especially as we need to flush messages to SD card regularly.
		// Currently, append mode is used for the log file and for appending simulated print times to GCodes files (which require read access too).
		if (mode == OpenMode::write || mode == OpenMode::writeWithCrc)
		{
			writeBuffer = MassStorage::AllocateWriteBuffer();
		}
	}

	// Attempt to open the file
	bool fileOpened = false;
# if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		handle = reprap.GetSbcInterface().OpenFile(filePath, mode, length, preAllocSize);
		if (handle != noFileHandle)
		{
			offset = (mode == OpenMode::append) ? length : 0;
			fileOpened = true;
		}
		else
		{
			length = offset = 0;
		}
	}
#  if HAS_MASS_STORAGE
	else
#  endif
# endif
# if HAS_MASS_STORAGE
	{
		const FRESULT openReturn = f_open(&file, filePath,
											(mode == OpenMode::write || mode == OpenMode::writeWithCrc) ? FA_CREATE_ALWAYS | FA_WRITE
												: (mode == OpenMode::append) ? FA_READ | FA_WRITE | FA_OPEN_ALWAYS | FA_OPEN_APPEND
													: FA_OPEN_EXISTING | FA_READ);
		if (openReturn == FR_OK)
		{
			fileOpened = true;
		}
		else
		{
			// We no longer report an error if opening a file in read mode fails unless debugging is enabled, because sometimes that is quite normal.
			// It is up to the caller to report an error if necessary.
			if (reprap.Debug(Module::Storage))
			{
				reprap.GetPlatform().MessageF(WarningMessage, "Failed to open %s to %s, error code %d\n", filePath, (writing) ? "write" : "read", (int)openReturn);
			}
		}
	}
# endif

	// Discard the write buffer if that failed
	if (!fileOpened)
	{
		if (writeBuffer != nullptr)
		{
			MassStorage::ReleaseWriteBuffer(writeBuffer);
			writeBuffer = nullptr;
		}

		return false;
	}

	// File open, carry on
	crc.Reset();
	calcCrc = (mode == OpenMode::writeWithCrc);
	usageMode = (writing) ? FileUseMode::readWrite : FileUseMode::readOnly;
	openCount = 1;
# if HAS_MASS_STORAGE
	if (preAllocSize != 0 && (mode == OpenMode::write || mode == OpenMode::writeWithCrc))
	{
		const FRESULT expandReturn = f_expand(&file, preAllocSize, 1);		// try to pre-allocate contiguous space - it doesn't matter if it fails
		if (reprap.Debug(Module::Storage))
		{
			debugPrintf("Preallocating %" PRIu32 " bytes returned %d\n", preAllocSize, (int)expandReturn);
		}
	}
# endif
	reprap.VolumesUpdated();
	return true;
#endif
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

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
			const irqflags_t flags = IrqSave();
			if (openCount > 1)
			{
				--openCount;
				IrqRestore(flags);
				return true;
			}
			else if (inInterrupt())
			{
				closeRequested = true;
				IrqRestore(flags);
				return true;
			}
			else
			{
				IrqRestore(flags);
				return ForceClose();
			}
		}

	case FileUseMode::invalidated:
	default:
		{
			const irqflags_t flags = IrqSave();
			if (openCount > 1)
			{
				--openCount;
				IrqRestore(flags);
			}
			else
			{
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
				FileWriteBuffer *wb = nullptr;
				std::swap(wb, writeBuffer);
				IrqRestore(flags);
				if (wb != nullptr)
				{
					MassStorage::ReleaseWriteBuffer(wb);
				}
#endif
				usageMode = FileUseMode::free;
			}
			return true;
		}
	}
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
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			if (reprap.GetSbcInterface().SeekFile(handle, pos))
			{
				offset = pos;
				return true;
			}
			return false;
		}
#endif
#if HAS_MASS_STORAGE
		return f_lseek(&file, pos) == FR_OK;
#elif HAS_EMBEDDED_FILES
		offset = min<FilePosition>(pos, EmbeddedFiles::Length(fileIndex));
		return true;
#else
		return false;
#endif

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

FilePosition FileStore::Position() const noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		return offset;
	}
#endif
#if HAS_MASS_STORAGE
	return (usageMode == FileUseMode::readOnly || usageMode == FileUseMode::readWrite) ? file.fptr : 0;
#elif HAS_EMBEDDED_FILES
	return offset;
#else
	return 0;
#endif
}

FilePosition FileStore::Length() const noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
		return 0;

	case FileUseMode::readOnly:
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			return length;
		}
#endif
#if HAS_MASS_STORAGE
		return f_size(&file);
#elif HAS_EMBEDDED_FILES
		return EmbeddedFiles::Length(fileIndex);
#else
		return 0;
#endif

	case FileUseMode::readWrite:
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			return (writeBuffer != nullptr) ? length + writeBuffer->BytesStored() : length;
		}
#endif
#if HAS_MASS_STORAGE
		return (writeBuffer != nullptr) ? f_size(&file) + writeBuffer->BytesStored() : f_size(&file);
#else
		return 0;
#endif

	case FileUseMode::invalidated:
	default:
		return 0;
	}
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

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
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			length = offset;
			return reprap.GetSbcInterface().TruncateFile(handle);
		}
#endif
#if HAS_MASS_STORAGE
		return f_truncate(&file) == FR_OK;
#else
		return false;
#endif

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

// Returns the number of bytes read or -1 if the read process failed
int FileStore::Read(char *_ecv_array extBuf, size_t nBytes) noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
		REPORT_INTERNAL_ERROR;
		return -1;

	case FileUseMode::readOnly:
	case FileUseMode::readWrite:
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			int bytesRead = reprap.GetSbcInterface().ReadFile(handle, extBuf, nBytes);
			if (bytesRead > 0)
			{
				offset += bytesRead;
			}
			return bytesRead;
		}
#endif
#if HAS_MASS_STORAGE
		{
			UINT bytes_read;
			const FRESULT readStatus = f_read(&file, extBuf, nBytes, &bytes_read);
			if (readStatus != FR_OK)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Cannot read file, error code %d\n", (int)readStatus);
				return -1;
			}
			return (int)bytes_read;
		}
#elif HAS_EMBEDDED_FILES
		{
			const int ret = EmbeddedFiles::Read(fileIndex, offset, extBuf, nBytes);
			if (ret > 0)
			{
				offset += ret;
			}
			return ret;
		}
#else
		return -1;
#endif

	case FileUseMode::invalidated:
	default:
		return -1;
	}
}

// As Read but stop after '\n' or '\r\n' and null-terminate the string.
// If the next line is too long to fit in the buffer then the line will be split.
// Return the number of characters in the line excluding the null terminator, or -1 if end of file or a read error occurs.
int FileStore::ReadLine(char *_ecv_array buf, size_t nBytes) noexcept
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

bool FileStore::ForceClose() noexcept
{
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
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
#endif

#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		reprap.GetSbcInterface().CloseFile(handle);
		handle = noFileHandle;
		offset = length = 0;
		usageMode = FileUseMode::free;
		return ok;
	}
#endif

#if HAS_MASS_STORAGE
	const FRESULT fr = f_close(&file);
	usageMode = FileUseMode::free;
	closeRequested = false;
	openCount = 0;
	reprap.VolumesUpdated();
	return ok && fr == FR_OK;
#endif

#if HAS_EMBEDDED_FILES
	usageMode = FileUseMode::free;
	closeRequested = false;
	openCount = 0;
	return true;
#endif

	return false;
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
			const irqflags_t flags = IrqSave();
			++openCount;
			IrqRestore(flags);
		}
		break;

	case FileUseMode::invalidated:
	default:
		break;
	}
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

bool FileStore::Store(const char *_ecv_array s, size_t len, size_t *bytesWritten) noexcept
{
	if (calcCrc)
	{
		crc.Update(s, len);
	}

#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		const bool ok = (len > 0) ? reprap.GetSbcInterface().WriteFile(handle, s, len) : true;
		*bytesWritten = ok ? len : 0;
		offset += len;
		if (offset > length)
		{
			length = offset;
		}
		return ok;
	}
#endif

#if HAS_MASS_STORAGE
	const FRESULT writeStatus = f_write(&file, s, len, bytesWritten);
	if (writeStatus != FR_OK)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to write to file, error code %d. Card may be full.\n", (int)writeStatus);
		return false;
	}
	return true;
#else
	*bytesWritten = 0;
	return false;
#endif
}

bool FileStore::Write(char b) noexcept
{
	return Write(&b, sizeof(char));
}

bool FileStore::Write(const char *_ecv_array b) noexcept
{
	return Write(b, strlen(b));
}

bool FileStore::Write(const char *_ecv_array s, size_t len) noexcept
{
	switch (usageMode)
	{
	case FileUseMode::free:
	case FileUseMode::readOnly:
		REPORT_INTERNAL_ERROR;
		return false;

	case FileUseMode::readWrite:
		{
			size_t totalBytesWritten = 0;
			bool writeOk = true;
			if (writeBuffer == nullptr)
			{
				writeOk = Store(s, len, &totalBytesWritten);
			}
			else
			{
				do
				{
					const size_t bytesStored = writeBuffer->Store(s + totalBytesWritten, len - totalBytesWritten);
					if (writeBuffer->BytesLeft() == 0)
					{
						const size_t bytesToWrite = writeBuffer->BytesStored();
						size_t bytesWritten;
						// In SBC mode, if we have a timeout or a connection reset then during the following Store() call this file may be invalidated
						writeOk = Store(writeBuffer->Data(), bytesToWrite, &bytesWritten);
						if (!writeOk)
						{
							break;
						}

						writeBuffer->DataTaken();

						if (bytesToWrite != bytesWritten)
						{
							// Something went wrong
							break;
						}
					}
					totalBytesWritten += bytesStored;
				}
				while (totalBytesWritten != len);
			}

			if (writeOk && totalBytesWritten == len)
			{
				return true;
			}
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to write to file. Card may be full.\n");
			return false;
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
				const bool writeOk = Store(writeBuffer->Data(), bytesToWrite, &bytesWritten);
				// Don't call DataTaken if Store() returned false because the SBC task may have invalidated the file
				if (writeOk)
				{
					writeBuffer->DataTaken();
					if (bytesWritten == bytesToWrite)
					{
						return true;
					}
				}

				reprap.GetPlatform().MessageF(ErrorMessage, "Failed to flush data to file. Card may be full.\n");
				return false;
			}
		}
#if HAS_SBC_INTERFACE
		if (reprap.UsingSbcInterface())
		{
			return true;
		}
#endif
#if HAS_MASS_STORAGE
		return f_sync(&file) == FR_OK;
#endif

	case FileUseMode::invalidated:
	default:
		return false;
	}
}

#if HAS_SBC_INTERFACE			// these functions are only supported in SBC mode

// Invalidate the file. Don't free the write buffer because another task may be using it.
void FileStore::Invalidate() noexcept
{
	usageMode = FileUseMode::invalidated;
	handle = noFileHandle;
}

#endif

#if HAS_MASS_STORAGE			// the remaining functions are only supported on local storage

// Invalidate the file if it uses the specified FATFS object
bool FileStore::Invalidate(const FATFS *fs) noexcept
{
	if (file.obj.fs == fs)
	{
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

// Return true if the passed file is the same as ours
bool FileStore::IsSameFile(const FIL& otherFile) const noexcept
{
	return file.obj.fs == otherFile.obj.fs && file.dir_sect == otherFile.dir_sect && file.dir_ptr == otherFile.dir_ptr;
}

uint32_t FileStore::ClusterSize() const noexcept
{
	return (usageMode == FileUseMode::readOnly || usageMode == FileUseMode::readWrite) ? file.obj.fs->csize * 512u : 1;	// we divide by the cluster size so return 1 not 0 if there is an error
}


#endif	// HAS_MASS_STORAGE

#if 0	// these are not currently used

bool FileStore::GoToEnd()
{
	return Seek(Length());
}

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

#endif	// HAS_MASS_STORAGE || HAS_SBC_INTERFACE

#if SUPPORT_ASYNC_MOVES && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)

// Copy an open file handle to make a duplicate with its own position. Intended for use on files opened in read-only mode.
// We assume that FatFs doesn't keep a count of open files, so it's OK for us to just make a copy of the FIL structure.
void FileStore::CopyFrom(const FileStore *f) noexcept
{
	usageMode = FileUseMode::readOnly;
#if HAS_EMBEDDED_FILES
	fileIndex = f->fileIndex;
#else
	file = f->file;
	writeBuffer = nullptr;
	crc.Reset();
	calcCrc = false;
#endif
	closeRequested = false;
	openCount = 1;
	reprap.VolumesUpdated();
}

#endif

#endif

// End
