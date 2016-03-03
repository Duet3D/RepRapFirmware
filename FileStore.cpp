//------------------------------------------------------------------------------------------------

#include "RepRapFirmware.h"
#include "FileStore.h"
#include "MassStorage.h"
#include "Platform.h"

uint32_t FileStore::longestWriteTime = 0;

FileStore::FileStore(Platform* p) : platform(p)
{
}

void FileStore::Init()
{
	bufferPointer = 0;
	inUse = false;
	writing = false;
	lastBufferEntry = 0;
	openCount = 0;
}

// Open a local file (for example on an SD card).
// This is protected - only Platform can access it.
bool FileStore::Open(const char* directory, const char* fileName, bool write)
{
	const char* location = (directory != NULL)
							? platform->GetMassStorage()->CombineName(directory, fileName)
								: fileName;
	writing = write;
	lastBufferEntry = FileBufLen;

	FRESULT openReturn = f_open(&file, location, (writing) ?  FA_CREATE_ALWAYS | FA_WRITE : FA_OPEN_EXISTING | FA_READ);
	if (openReturn != FR_OK)
	{
		// We no longer report an error if opening a file in read mode fails unless debugging is enabled, because sometimes that is quite normal.
		// It is up to the caller to report an error if necessary.
		if (reprap.Debug(modulePlatform))
		{
			platform->MessageF(GENERIC_MESSAGE, "Can't open %s to %s, error code %d\n", location, (writing) ? "write" : "read", openReturn);
		}
		return false;
	}

	bufferPointer = (writing) ? 0 : FileBufLen;
	inUse = true;
	openCount = 1;
	return true;
}

void FileStore::Duplicate()
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to dup a non-open file.\n");
		return;
	}
	++openCount;
}

bool FileStore::Close()
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to close a non-open file.\n");
		return false;
	}
	--openCount;
	if (openCount != 0)
	{
		return true;
	}
	bool ok = true;
	if (writing)
	{
		ok = Flush();
	}
	FRESULT fr = f_close(&file);
	inUse = false;
	writing = false;
	lastBufferEntry = 0;
	return ok && fr == FR_OK;
}

bool FileStore::Seek(FilePosition pos)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to seek on a non-open file.\n");
		return false;
	}
	if (writing)
	{
		WriteBuffer();
	}
	FRESULT fr = f_lseek(&file, pos);
	bufferPointer = (writing) ? 0 : FileBufLen;
	return fr == FR_OK;
}

FilePosition FileStore::Position() const
{
	FilePosition pos = file.fptr;
	if (writing)
	{
		pos += bufferPointer;
	}
	else if (bufferPointer < lastBufferEntry)
	{
		pos -= (lastBufferEntry - bufferPointer);
	}
	return pos;
}

#if 0	// not currently used
bool FileStore::GoToEnd()
{
	return Seek(Length());
}
#endif

FilePosition FileStore::Length() const
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to size non-open file.\n");
		return 0;
	}
	return file.fsize;
}

float FileStore::FractionRead() const
{
	FilePosition len = Length();
	if (len == 0)
	{
		return 0.0;
	}

	return (float)Position() / (float)len;
}

uint8_t FileStore::Status()
{
	if (!inUse)
		return (uint8_t)IOStatus::nothing;

	if (lastBufferEntry == FileBufLen)
		return (uint8_t)IOStatus::byteAvailable;

	if (bufferPointer < lastBufferEntry)
		return (uint8_t)IOStatus::byteAvailable;

	return (uint8_t)IOStatus::nothing;
}

bool FileStore::ReadBuffer()
{
	FRESULT readStatus = f_read(&file, GetBuffer(), FileBufLen, &lastBufferEntry);	// Read a chunk of file
	if (readStatus)
	{
		platform->Message(GENERIC_MESSAGE, "Error reading file.\n");
		return false;
	}
	bufferPointer = 0;
	return true;
}

// Single character read via the buffer
bool FileStore::Read(char& b)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to read from a non-open file.\n");
		return false;
	}

	if (bufferPointer >= FileBufLen)
	{
		bool ok = ReadBuffer();
		if (!ok)
		{
			return false;
		}
	}

	if (bufferPointer >= lastBufferEntry)
	{
		b = 0;  // Good idea?
		return false;
	}

	b = (char) GetBuffer()[bufferPointer];
	bufferPointer++;

	return true;
}

// Block read, doesn't use the buffer
int FileStore::Read(char* extBuf, unsigned int nBytes)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to read from a non-open file.\n");
		return -1;
	}

	bufferPointer = FileBufLen;			// invalidate the buffer
	UINT bytes_read;
	FRESULT readStatus = f_read(&file, extBuf, nBytes, &bytes_read);
	if (readStatus != FR_OK)
	{
		platform->Message(GENERIC_MESSAGE, "Error reading file.\n");
		return -1;
	}
	return (int)bytes_read;
}

bool FileStore::WriteBuffer()
{
	if (bufferPointer != 0)
	{
		bool ok = InternalWriteBlock((const char*)GetBuffer(), bufferPointer);
		if (!ok)
		{
			platform->Message(GENERIC_MESSAGE, "Cannot write to file. Disc may be full.\n");
			return false;
		}
		bufferPointer = 0;
	}
	return true;
}

bool FileStore::Write(char b)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to write byte to a non-open file.\n");
		return false;
	}
	GetBuffer()[bufferPointer] = b;
	bufferPointer++;
	if (bufferPointer >= FileBufLen)
	{
		return WriteBuffer();
	}
	return true;
}

bool FileStore::Write(const char* b)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to write string to a non-open file.\n");
		return false;
	}
	int i = 0;
	while (b[i])
	{
		if (!Write(b[i++]))
		{
			return false;
		}
	}
	return true;
}

// Direct block write that bypasses the buffer. Used when uploading files.
bool FileStore::Write(const char *s, unsigned int len)
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to write block to a non-open file.\n");
		return false;
	}

	if (!WriteBuffer())
	{
		return false;
	}
	return InternalWriteBlock(s, len);
}

// Debugging variables
//extern "C" uint32_t numRead, numWrite;
//uint32_t maxRead, maxWrite;

bool FileStore::InternalWriteBlock(const char *s, unsigned int len)
{
	unsigned int bytesWritten;
	uint32_t time = micros();
//	numRead = numWrite = 0;

	FRESULT writeStatus = f_write(&file, s, len, &bytesWritten);
	time = micros() - time;
	if (time > longestWriteTime)
	{
		longestWriteTime = time;
//		maxRead=numRead; maxWrite=numWrite;
	}
	if ((writeStatus != FR_OK) || (bytesWritten != len))
	{
		platform->Message(GENERIC_MESSAGE, "Cannot write to file. Disc may be full.\n");
		return false;
	}
	return true;
}

bool FileStore::Flush()
{
	if (!inUse)
	{
		platform->Message(GENERIC_MESSAGE, "Attempt to flush a non-open file.\n");
		return false;
	}
	if (!WriteBuffer())
	{
		return false;
	}
	return f_sync(&file) == FR_OK;
}

float FileStore::GetAndClearLongestWriteTime()
{
	float ret = (float)longestWriteTime/1000.0;
	longestWriteTime = 0;
	return ret;
}

// End
