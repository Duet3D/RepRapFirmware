/*
 * Logger.cpp
 *
 *  Created on: 17 Sep 2017
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "Logger.h"
#include "OutputMemory.h"
#include "RepRap.h"
#include "Platform.h"

// Simple lock class that sets a variable true when it is created and makes sure it gets set false when it falls out of scope
class Lock
{
public:
	Lock(bool& pb) : b(pb) { b = true; }
	~Lock() { b = false; }

private:
	bool& b;
};

Logger::Logger() : logFile(), dirty(false), inLogger(false)
{
}

void Logger::Start(time_t time, const StringRef& filename)
{
	if (!inLogger)
	{
		Lock loggerLock(inLogger);
		FileStore * const f = reprap.GetPlatform().GetFileStore(SYS_DIR, filename.Pointer(), OpenMode::append);
		if (f != nullptr)
		{
			logFile.Set(f);
			logFile.Seek(logFile.Length());
			InternalLogMessage(time, "Event logging started\n");
		}
	}
}

void Logger::Stop(time_t time)
{
	if (logFile.IsLive() && !inLogger)
	{
		Lock loggerLock(inLogger);
		InternalLogMessage(time, "Event logging stopped\n");
		logFile.Close();
	}
}

void Logger::LogMessage(time_t time, const char *message)
{
	if (logFile.IsLive() && !inLogger)
	{
		Lock loggerLock(inLogger);
		InternalLogMessage(time, message);
	}
}

void Logger::LogMessage(time_t time, OutputBuffer *buf)
{
	if (logFile.IsLive() && !inLogger)
	{
		Lock loggerLock(inLogger);
		bool ok = WriteDateTime(time);
		if (ok)
		{
			ok = buf->WriteToFile(logFile);
		}

		if (ok)
		{
			dirty = true;
		}
		else
		{
			logFile.Close();
		}
	}
}

// Version of LogMessage for when we already know we want to proceed and we have already set inLogger
void Logger::InternalLogMessage(time_t time, const char *message)
{
	bool ok = WriteDateTime(time);
	if (ok)
	{
		const size_t len = strlen(message);
		if (len != 0)
		{
			ok = logFile.Write(message, len);
		}
		if (ok && (len == 0 || message[len - 1] != '\n'))
		{
			ok = logFile.Write('\n');
		}
	}

	if (ok)
	{
		dirty = true;
	}
	else
	{
		logFile.Close();
	}
}

void Logger::Flush()
{
	if (logFile.IsLive() && dirty && !inLogger)
	{
		Lock loggerLock(inLogger);
		logFile.Flush();
		dirty = false;
	}
}

// Write the data and time to the file followed by a space.
// Caller must already have checked and set inLogger.
bool Logger::WriteDateTime(time_t time)
{
	char bufferSpace[30];
	StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));
	if (time == 0)
	{
		const uint32_t timeSincePowerUp = (uint32_t)(millis64()/1000u);
		buf.printf("power up + %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32 " ", timeSincePowerUp/3600u, (timeSincePowerUp % 3600u)/60u, timeSincePowerUp % 60u);
	}
	else
	{
		const struct tm * const timeInfo = gmtime(&time);
		buf.printf("%04u-%02u-%02u %02u:%02u:%02u ",
						timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
	}
	return logFile.Write(buf.Pointer());
}

// End
