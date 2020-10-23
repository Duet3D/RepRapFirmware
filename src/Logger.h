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
#include "MessageType.h"
#include <General/NamedEnum.h>

class OutputBuffer;

NamedEnum(LogLevel, uint8_t, OFF, WARN, INFO, DEBUG);

class Logger
{
public:
	Logger(LogLevel logLvl) noexcept;

	void Start(time_t time, const StringRef& file) noexcept;
	void Stop(time_t time) noexcept;
	void LogMessage(time_t time, const char *message, MessageType type) noexcept;
	void LogMessage(time_t time, OutputBuffer *buf, MessageType type) noexcept;
	void Flush(bool forced) noexcept;
	bool IsActive() const noexcept { return logFile.IsLive(); }
	const char *GetFileName() const noexcept { return (IsActive()) ? logFileName.c_str() : nullptr; }
	LogLevel GetLogLevel() const noexcept { return logLevel; }
	void SetLogLevel(LogLevel newLogLevel) noexcept;
//	bool IsLoggingEnabledFor(const MessageType mt) const noexcept;
//	bool IsWarnEnabled() const noexcept { return logLevel >= LogLevel::WARN; }
//	bool IsInfoEnabled() const noexcept { return logLevel >= LogLevel::INFO; }
//	bool IsDebugEnabled() const noexcept { return logLevel >= LogLevel::DEBUG; }

private:
	NamedEnum(MessageLogLevel, uint8_t, DEBUG, INFO, WARN, OFF);
	MessageLogLevel GetMessageLogLevel(MessageType mt) const noexcept { return (MessageLogLevel) ((mt & MessageType::LogOff)>>30); }

	static const uint8_t LogEnabledThreshold = 3;

	bool WriteDateTime(time_t time) noexcept;
	bool WriteLogLevelPrefix(MessageLogLevel messageLogLevel) noexcept;
	void InternalLogMessage(time_t time, const char *message, const MessageLogLevel messageLogLevel) noexcept;
	bool IsLoggingEnabledFor(const MessageLogLevel mll) const noexcept { return (mll < MessageLogLevel::OFF) && (mll.ToBaseType() + logLevel.ToBaseType() >= LogEnabledThreshold); }
	void LogFirmwareInfo(time_t time) noexcept;
	bool IsEmptyMessage(const char * message) const noexcept { return message[0] == '\0' || (message[0] == '\n' && message[1] == '\0'); }

	String<MaxFilenameLength> logFileName;
	FileData logFile;
	uint32_t lastFlushTime;
	FilePosition lastFlushFileSize;
	bool dirty;
	bool inLogger;
	LogLevel logLevel;
};

#endif

#endif /* SRC_LOGGER_H_ */
