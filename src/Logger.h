/*
 * Logger.h
 *
 *  Created on: 17 Sep 2017
 *      Author: David
 */

#ifndef SRC_LOGGER_H_
#define SRC_LOGGER_H_

#include <ctime>
#include "Storage/FileData.h"

class OutputBuffer;

class Logger
{
public:
	Logger();

	void Start(time_t time, const StringRef& file);
	void Stop(time_t time);
	void LogMessage(time_t time, const char *message);
	void LogMessage(time_t time, OutputBuffer *buf);
	void Flush(bool forced);
	bool IsActive() const { return logFile.IsLive(); }

private:
	bool WriteDateTime(time_t time);
	void InternalLogMessage(time_t time, const char *message);

	FileData logFile;
	uint32_t lastFlushTime;
	FilePosition lastFlushFileSize;
	bool dirty;
	bool inLogger;
};

#endif /* SRC_LOGGER_H_ */
