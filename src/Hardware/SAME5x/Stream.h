/*
  Stream.h
*/

#ifndef HARDWARE_SAME5X_STREAM_H
#define HARDWARE_SAME5X_STREAM_H

#include "Print.h"

class Stream : public Print
{
public:
	Stream() noexcept {}

	virtual int available() noexcept = 0;
	virtual int read() noexcept = 0;
	virtual void flush() noexcept = 0;
	virtual size_t canWrite() const noexcept { return 1; }	// DC42 added for Duet
	virtual size_t readBytes( char *buffer, size_t length) noexcept; // read chars from stream into buffer

	size_t readBytes( uint8_t *buffer, size_t length) noexcept { return readBytes((char *)buffer, length); }

};

#endif
