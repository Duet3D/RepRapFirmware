/*
 * Logger.cpp
 *
 *  Created on: 17 Sep 2017
 *      Author: David
 */

#include "Logger.h"

#if HAS_MASS_STORAGE

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

Logger::Logger() noexcept : logFile(), lastFlushTime(0), lastFlushFileSize(0), dirty(false), inLogger(false)
{
}

void Logger::Start(time_t time, const StringRef& filename) noexcept
{
	if (!inLogger)
	{
		Lock loggerLock(inLogger);
		FileStore * const f = reprap.GetPlatform().OpenSysFile(filename.c_str(), OpenMode::append);
		if (f != nullptr)
		{
			logFile.Set(f);
			lastFlushFileSize = logFile.Length();
			logFile.Seek(lastFlushFileSize);
			logFileName.copy(filename.c_str());
			InternalLogMessage(time, "Event logging started\n");
			reprap.StateUpdated();
		}
	}
}

void Logger::Stop(time_t time) noexcept
{
	if (logFile.IsLive() && !inLogger)
	{
		Lock loggerLock(inLogger);
		InternalLogMessage(time, "Event logging stopped\n");
		logFile.Close();
		reprap.StateUpdated();
	}
}

void Logger::LogMessage(time_t time, const char *message) noexcept
{
	if (logFile.IsLive() && !inLogger)
	{
		Lock loggerLock(inLogger);
		InternalLogMessage(time, message);
	}
}

void Logger::LogMessage(time_t time, OutputBuffer *buf) noexcept
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
			reprap.StateUpdated();
		}
	}
}

// Version of LogMessage for when we already know we want to proceed and we have already set inLogger
void Logger::InternalLogMessage(time_t time, const char *message) noexcept
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
		reprap.StateUpdated();
	}
}

// This is called regularly by Platform to give the logger an opportunity to flush the file buffer
void Logger::Flush(bool forced) noexcept
{
	if (logFile.IsLive() && dirty && !inLogger)
	{
		// Log file is dirty and can be flushed.
		// To avoid excessive disk write operations, flush it only if one of the following is true:
		// 1. We have possibly allocated a new cluster since the last flush. To avoid lost clusters if we power down before flushing,
		//    we should flush early in this case. Rather than determine the cluster size, we flush if we have started a new 512-byte sector.
		// 2. If it hasn't been flushed for LogFlushInterval milliseconds.
		const FilePosition currentPos = logFile.GetPosition();
		const uint32_t now = millis();
		if (forced || now - lastFlushTime >= LogFlushInterval || currentPos/512 != lastFlushFileSize/512)
		{
			Lock loggerLock(inLogger);
			logFile.Flush();
			lastFlushTime = millis();
			lastFlushFileSize = currentPos;
			dirty = false;
		}
	}
}

// Write the data and time to the file followed by a space.
// Caller must already have checked and set inLogger.
bool Logger::WriteDateTime(time_t time) noexcept
{
	String<30> bufferSpace;
	const StringRef buf = bufferSpace.GetRef();
	if (time == 0)
	{
		const uint32_t timeSincePowerUp = (uint32_t)(millis64()/1000u);
		buf.printf("power up + %02" PRIu32 ":%02" PRIu32 ":%02" PRIu32 " ", timeSincePowerUp/3600u, (timeSincePowerUp % 3600u)/60u, timeSincePowerUp % 60u);
	}
	else
	{
		tm timeInfo;
		gmtime_r(&time, &timeInfo);
		buf.printf("%04u-%02u-%02u %02u:%02u:%02u ",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}
	return logFile.Write(buf.c_str());
}

#endif

// End
