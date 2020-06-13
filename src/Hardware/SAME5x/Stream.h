/*
  Stream.h
*/

#ifndef HARDWARE_SAME5X_STREAM_H
#define HARDWARE_SAME5X_STREAM_H

#include <RepRapFirmware.h>

class Stream
{
protected:
#if 0
	unsigned long _timeout;      // number of milliseconds to wait for the next char before aborting timed read
	unsigned long _startMillis;  // used for timeout measurement
	int timedRead() noexcept;    // private method to read stream with timeout
	int timedPeek() noexcept;    // private method to peek stream with timeout
	int peekNextDigit() noexcept; // returns the next numeric digit in the stream or -1 if timeout
#endif
public:
	virtual int available() noexcept = 0;
	virtual int read() noexcept = 0;
	virtual int peek() noexcept = 0;
	virtual void flush() noexcept = 0;
	virtual size_t canWrite() const noexcept { return 1; }	// DC42 added for Duet

	Stream() noexcept /*: _timeout(1000), _startMillis(0)*/ {}

	virtual size_t readBytes( char *buffer, size_t length) noexcept; // read chars from stream into buffer
	size_t readBytes( uint8_t *buffer, size_t length) noexcept { return readBytes((char *)buffer, length); }
	// terminates if length characters have been read or timeout (see setTimeout)
	// returns the number of characters placed in the buffer (0 means no valid data found)

};

#endif
