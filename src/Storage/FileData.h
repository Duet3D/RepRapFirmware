/*
 * FileData.h
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#ifndef FILEDATA_H_
#define FILEDATA_H_

#include "FileStore.h"

class FileGCodeInput;

// Small class to hold an open file and data relating to it.
// This is designed so that files are never left open and we never duplicate a file reference.
class FileData
{
public:
	friend class FileGCodeInput;

	FileData() noexcept : f(nullptr) {}

	~FileData() { (void)Close(); }

	FileData(const FileData& other) noexcept
	{
		f = other.f;
		if (f != nullptr)
		{
			f->Duplicate();
		}
	}

	FileData(FileData&& other) noexcept
	{
		f = other.f;
		other.f = nullptr;
	}

	// Make sure we don't assign these objects
	FileData& operator=(const FileData&) = delete;

	// Set this to refer to a newly-opened file
	void Set(FileStore* pfile) noexcept
	{
		Close();	// close any existing file
		f = pfile;
	}

	bool IsLive() const noexcept { return f != nullptr; }

	bool Close() noexcept
	{
		if (f != nullptr)
		{
			bool ok = f->Close();
			f = nullptr;
			return ok;
		}
		return false;
	}

	bool Read(char& b) noexcept
	{
		return f->Read(b);
	}

	int Read(char *buf, size_t nBytes) noexcept
	{
		return f->Read(buf, nBytes);
	}

# if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	bool Write(char b) noexcept
	{
		return f->Write(b);
	}

	bool Write(const char *s) noexcept
	{
		return f->Write(s, strlen(s));
	}

	bool Write(const char *s, size_t len) noexcept
	{
		return f->Write(s, len);
	}

	bool Write(const uint8_t *s, size_t len) noexcept
	{
		return f->Write(s, len);
	}

	// This returns the CRC32 of data written to a newly-created file. It does not calculate the CRC of an existing file.
	uint32_t GetCrc32() const noexcept
	{
		return f->GetCRC32();
	}

	bool Flush() noexcept
	{
		return f->Flush();
	}
# endif

	FilePosition GetPosition() const noexcept
	{
		return f->Position();
	}

	bool Seek(FilePosition position) noexcept
	{
		return f->Seek(position);
	}

	FilePosition Length() const noexcept
	{
		return f->Length();
	}

	// Move operator
	void MoveFrom(FileData& other) noexcept
	{
		Close();
		f = other.f;
		other.Init();
	}

private:
	FileStore *f;

	void Init() noexcept
	{
		f = nullptr;
	}
};

#endif
