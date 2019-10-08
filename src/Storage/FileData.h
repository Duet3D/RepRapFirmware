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

	FileData() : f(nullptr) {}

	// Set this to refer to a newly-opened file
	void Set(FileStore* pfile)
	{
		Close();	// close any existing file
		f = pfile;
	}

	bool IsLive() const { return f != nullptr; }

	bool Close()
	{
		if (f != nullptr)
		{
			bool ok = f->Close();
			f = nullptr;
			return ok;
		}
		return false;
	}

	bool Read(char& b)
	{
		return f->Read(b);
	}

	int Read(char *buf, size_t nBytes)
	{
		return f->Read(buf, nBytes);
	}

	bool Write(char b)
	{
		return f->Write(b);
	}

	bool Write(const char *s)
	{
		return f->Write(s, strlen(s));
	}

	bool Write(const char *s, size_t len)
	{
		return f->Write(s, len);
	}

	bool Write(const uint8_t *s, size_t len)
	{
		return f->Write(s, len);
	}

	// This returns the CRC32 of data written to a newly-created file. It does not calculate the CRC of an existing file.
	uint32_t GetCrc32() const
	{
		return f->GetCRC32();
	}

	bool Flush()
	{
		return f->Flush();
	}

	FilePosition GetPosition() const
	{
		return f->Position();
	}

	bool Seek(FilePosition position)
	{
		return f->Seek(position);
	}

	FilePosition Length() const
	{
		return f->Length();
	}

	// Assignment operator
	void CopyFrom(const FileData& other)
	{
		Close();
		f = other.f;
		if (f != nullptr)
		{
			f->Duplicate();
		}
	}

	// Move operator
	void MoveFrom(FileData& other)
	{
		Close();
		f = other.f;
		other.Init();
	}

private:
	FileStore *f;

	void Init()
	{
		f = nullptr;
	}

	// Private assignment operator to prevent us assigning these objects
	FileData& operator=(const FileData&);

	// Private copy constructor to prevent us copying these objects
	FileData(const FileData&);
};

#endif
