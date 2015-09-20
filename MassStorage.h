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
  bool PathExists(const char *path) const;
  bool PathExists(const char* directory, const char* subDirectory);

friend class Platform;

protected:

  MassStorage(Platform* p);
  void Init();

private:

  Platform* platform;
  FATFS fileSystem;
  DIR findDir;
  char combinedName[MaxFilenameLength + 1];
};

#endif
