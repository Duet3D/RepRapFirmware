// This class handles serial I/O - typically via USB

#ifndef LINE_H
#define LINE_H

#include "Arduino.h"

// Input and output - these are ORed into a uint8_t
// By the Status() functions of the IO classes.

enum class IOStatus
{
  nothing = 0,
  byteAvailable = 1,
  atEoF = 2,
  clientLive = 4,
  clientConnected = 8
};

const uint16_t lineInBufsize = 256;				// use a power of 2 for good performance
const uint16_t lineOutBufSize = 2048;			// ideally this should be large enough to hold the results of an M503 command,
												// but could be reduced if we ever need the memory
class Line
{
public:

	uint8_t Status() const;				// Returns OR of IOStatus
	int Read(char& b);
	void Write(char b, bool important = false);
	void Write(const char* s, bool important = false);
	void Flush();

friend class Platform;
friend class RepRap;

protected:

	Line(Stream& p_iface);
	void Init();
	void Spin();
	void InjectString(char* string);
	unsigned int GetOutputColumn() const { return outputColumn; }

private:
	void TryFlushOutput();

	// Although the sam3x usb interface code already has a 512-byte buffer, adding this extra 256-byte buffer
	// increases the speed of uploading to the SD card by 10%
	char inBuffer[lineInBufsize];
	char outBuffer[lineOutBufSize];
	uint16_t inputGetIndex;
	uint16_t inputNumChars;
	uint16_t outputGetIndex;
	uint16_t outputNumChars;
	uint32_t timeLastCharWritten;

	uint8_t inWrite;
	bool ignoringOutputLine;
	unsigned int outputColumn;
	Stream& iface;
};

#endif
