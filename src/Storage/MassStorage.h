#ifndef MASSSTORAGE_H
#define MASSSTORAGE_H

#include "RepRapFirmware.h"
#include "Pins.h"
#include "FileWriteBuffer.h"
#include "Libraries/Fatfs/ff.h"
#include <ctime>

// Info returned by FindFirst/FindNext calls
struct FileInfo
{
	bool isDirectory;
	char fileName[FILENAME_LENGTH];
	unsigned long size;
	time_t lastModified;
};

class MassStorage
{
public:

	bool FindFirst(const char *directory, FileInfo &file_info);
	bool FindNext(FileInfo &file_info);
	const char* GetMonthName(const uint8_t month);
	const char* CombineName(const char* directory, const char* fileName);
	bool Delete(const char* directory, const char* fileName, bool silent = false);
	bool MakeDirectory(const char *parentDir, const char *dirName);
	bool MakeDirectory(const char *directory);
	bool Rename(const char *oldFilename, const char *newFilename);
	bool FileExists(const char *file) const;
	bool FileExists(const char* directory, const char *fileName) const;
	bool DirectoryExists(const char *path) const;
	bool DirectoryExists(const char* directory, const char* subDirectory);
	time_t GetLastModifiedTime(const char* directory, const char *fileName) const;
	bool SetLastModifiedTime(const char* directory, const char *file, time_t time);
	bool Mount(size_t card, StringRef& reply, bool reportSuccess);
	bool Unmount(size_t card, StringRef& reply);
	bool IsDriveMounted(size_t drive) const { return drive < NumSdCards && isMounted[drive]; }
	bool CheckDriveMounted(const char* path);

friend class Platform;
friend class FileStore;

protected:

	MassStorage(Platform* p);
	void Init();

	FileWriteBuffer *AllocateWriteBuffer();
	void ReleaseWriteBuffer(FileWriteBuffer *buffer);

private:
	static time_t ConvertTimeStamp(uint16_t fdate, uint16_t ftime);

	Platform* platform;
	FATFS fileSystems[NumSdCards];
	DIR findDir;
	bool isMounted[NumSdCards];
	char combinedName[FILENAME_LENGTH + 1];

	FileWriteBuffer *freeWriteBuffers;
};

#endif
