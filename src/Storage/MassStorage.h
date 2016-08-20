#ifndef MASSSTORAGE_H

#define MASSSTORAGE_H

class Platform;
class FileInfo;

class MassStorage
{
public:

	bool FindFirst(const char *directory, FileInfo &file_info);
	bool FindNext(FileInfo &file_info);
	const char* GetMonthName(const uint8_t month);
	const char* CombineName(const char* directory, const char* fileName);
	bool Delete(const char* directory, const char* fileName);
	bool MakeDirectory(const char *parentDir, const char *dirName);
	bool MakeDirectory(const char *directory);
	bool Rename(const char *oldFilename, const char *newFilename);
	bool FileExists(const char *file) const;
	bool FileExists(const char* directory, const char *fileName) const;
	bool DirectoryExists(const char *path) const;
	bool DirectoryExists(const char* directory, const char* subDirectory);
	bool Mount(size_t card, StringRef& reply);
	bool Unmount(size_t card, StringRef& reply);

friend class Platform;

protected:

	MassStorage(Platform* p);
	void Init();

private:

	Platform* platform;
	FATFS fileSystems[NumSdCards];
	DIR findDir;
	char combinedName[FILENAME_LENGTH + 1];
};

#endif
