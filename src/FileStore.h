// This class handles input from, and output to, files.

#ifndef FILESTORE_H
#define FILESTORE_H

typedef uint32_t FilePosition;
const FilePosition noFilePosition = 0xFFFFFFFF;
const size_t FileBufLen = 256;						// 512 would be more efficient, but need to free up some RAM first

enum class IOStatus : uint8_t
{
	nothing = 0,
	byteAvailable = 1,
	atEoF = 2,
	clientLive = 4,
	clientConnected = 8
};

class FileStore
{
public:

	uint8_t Status();								// Returns OR of IOStatus
	bool Read(char& b);								// Read 1 byte
	int Read(char* buf, size_t nBytes);				// Read a block of nBytes length
	bool Write(char b);								// Write 1 byte
	bool Write(const char *s, size_t len);			// Write a block of len bytes
	bool Write(const char* s);						// Write a string
	bool Close();									// Shut the file and tidy up
	bool Seek(FilePosition pos);					// Jump to pos in the file
	FilePosition Position() const;					// Return the current position in the file, assuming we are reading the file
#if 0	// not currently used
	bool GoToEnd();									// Position the file at the end (so you can write on the end).
#endif
	FilePosition Length() const;					// File size in bytes
	float FractionRead() const;						// How far in we are
	void Duplicate();								// Create a second reference to this file
	bool Flush();									// Write remaining buffer data
	static float GetAndClearLongestWriteTime();		// Return the longest time it took to write a block to a file, in milliseconds

	friend class Platform;

protected:

	FileStore(Platform* p);
	void Init();
    bool Open(const char* directory, const char* fileName, bool write);

private:
	bool ReadBuffer();
	bool WriteBuffer();
	bool InternalWriteBlock(const char *s, size_t len);
	uint8_t *GetBuffer() { return reinterpret_cast<uint8_t*>(buf32); }

    uint32_t buf32[FileBufLen/4];
	Platform* platform;
    unsigned int bufferPointer;

	FIL file;
	unsigned int lastBufferEntry;
	volatile unsigned int openCount;
	volatile bool closeRequested;

	bool inUse;
	bool writing;

	static uint32_t longestWriteTime;
};

#endif
