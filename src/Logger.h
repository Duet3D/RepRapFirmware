/*
 * Logger.h
 *
 *  Created on: 17 Sep 2017
 *      Author: David
 */

#ifndef SRC_LOGGER_H_
#define SRC_LOGGER_H_

#include "RepRapFirmware.h"

#if HAS_MASS_STORAGE

#include <ctime>
#include "Storage/FileData.h"

class OutputBuffer;

class Logger
{
public:
	Logger() noexcept;

	void Start(time_t time, const StringRef& file) noexcept;
	void Stop(time_t time) noexcept;
	void LogMessage(time_t time, const char *message) noexcept;
	void LogMessage(time_t time, OutputBuffer *buf) noexcept;
	void Flush(bool forced) noexcept;
	bool IsActive() const noexcept { return logFile.IsLive(); }
	const char *GetFileName() const noexcept { return (IsActive()) ? logFileName.c_str() : nullptr; }

private:
	bool WriteDateTime(time_t time) noexcept;
	void InternalLogMessage(time_t time, const char *message) noexcept;

	String<MaxFilenameLength> logFileName;
	FileData logFile;
	uint32_t lastFlushTime;
	FilePosition lastFlushFileSize;
	bool dirty;
	bool inLogger;
};

#endif

#endif /* SRC_LOGGER_H_ */
