/*
 * FileData.h
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#ifndef FILEDATA_H_
#define FILEDATA_H_

#include "FileStore.h"

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

class FileGCodeInput;

// Small class to hold an open file and data relating to it.
// This is designed so that files are never left open and we never duplicate a file reference.
class FileData
{
public:
	FileData() noexcept : f(nullptr) {}

	~FileData() { (void)Close(); }

	FileData(const FileData& other) noexcept
	{
		f = other.f;
		if (f != nullptr)
		{
			not_null(f)->Duplicate();
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

	bool operator==(const FileData& other) const noexcept
	{
		return f == other.f;
	}

	bool operator!=(const FileData& other) const noexcept
	{
		return f != other.f;
	}

	bool Close() noexcept
	{
		if (f != nullptr)
		{
			bool ok = not_null(f)->Close();
			f = nullptr;
			return ok;
		}
		return false;
	}

	bool Read(char& b) noexcept
	pre(IsLive())
	{
		return not_null(f)->Read(b);
	}

	int Read(char *_ecv_array buf, size_t nBytes) noexcept
	pre(IsLive())
	{
		return not_null(f)->Read(buf, nBytes);
	}

# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool Write(char b) noexcept
	pre(IsLive())
	{
		return not_null(f)->Write(b);
	}

	bool Write(const char *_ecv_array s) noexcept
	pre(IsLive())
	{
		return not_null(f)->Write(s, strlen(s));
	}

	bool Write(const char *_ecv_array s, size_t len) noexcept
	pre(IsLive())
	{
		return not_null(f)->Write(s, len);
	}

	bool Write(const uint8_t *_ecv_array s, size_t len) noexcept
	pre(IsLive())
	{
		return not_null(f)->Write(s, len);
	}

	// This returns the CRC32 of data written to a newly-created file. It does not calculate the CRC of an existing file.
	uint32_t GetCrc32() const noexcept
	pre(IsLive())
	{
		return not_null(f)->GetCRC32();
	}

	bool Flush() noexcept
	pre(IsLive())
	{
		return not_null(f)->Flush();
	}
# endif

	FilePosition GetPosition() const noexcept
	pre(IsLive())
	{
		return not_null(f)->Position();
	}

	bool Seek(FilePosition position) noexcept
	pre(IsLive())
	{
		return not_null(f)->Seek(position);
	}

	FilePosition Length() const noexcept
	pre(IsLive())
	{
		return not_null(f)->Length();
	}

	// Move operator
	void MoveFrom(FileData& other) noexcept
	{
		Close();
		f = other.f;
		other.Init();
	}

	// Copy operator
	void CopyFrom(const FileData& other) noexcept
	{
		Close();
		f = other.f;
		if (f != nullptr)
		{
			not_null(f)->Duplicate();
		}
	}

private:
	FileStore * null f;

	void Init() noexcept
	{
		f = nullptr;
	}
};

#endif

#endif
