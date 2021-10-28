/*
 * MessageType.h
 *
 *  Created on: 21 May 2016
 *      Authors: David and Christian
 */

#ifndef MESSAGETYPE_H_
#define MESSAGETYPE_H_

#include <cstdint>

// Supported message destinations. This is now a bitmap. Note that this type is used by the SBC service as well
enum MessageType : uint32_t
{
	// Destinations (bytes 1-2)
	// Keep the following in sync with the order of GCodeBuffers in the GCodes class
	HttpMessage =				  0x01,	// A message that is to be sent to the web (HTTP)
	TelnetMessage =				  0x02,	// A message that is to be sent to a Telnet client
	FileMessage =				  0x04,	// A message that is to be sent to a file processor
	UsbMessage =				  0x08,	// A message that is to be sent in non-blocking mode to the host via USB
	AuxMessage =				  0x10,	// A message that is to be sent to an auxiliary device (PanelDue)
	TriggerMessage =			  0x20,	// A message that is to be sent to a trigger processor
	CodeQueueMessage =			  0x40,	// A message that is to be sent to the code queue channel
	LcdMessage =				  0x80,	// A message that is to be sent to the panel
	SbcMessage =				 0x100,	// A message that is to be sent to the SBC
	DaemonMessage =				 0x200,	// A message that is sent to the daemon processor
	Aux2Message =				 0x400,	// A message that is to be sent to the second aux device
	AutoPauseMessage =			 0x800,	// A message that is to be sent to an auto-pause processor

	// Special destinations (byte 3)
	BlockingUsbMessage =	   0x10000,	// A message that is to be sent to USB in blocking mode
	ImmediateAuxMessage =	   0x20000,	// A message that is to be sent to LCD in immediate mode

	DestinationsMask =		   0x308FF, // Mask for all the destinations

	// Special indicators (byte 4)
	// The first two are not processed when calling the version of Platform::Message that takes an OutputBuffer.
	ErrorMessageFlag =		 0x1000000,	// This is an error message
	WarningMessageFlag =	 0x2000000,	// This is a warning message
	RawMessageFlag =		 0x8000000,	// Do not encapsulate this message
	BinaryCodeReplyFlag =	0x10000000,	// This message comes from a binary G-Code buffer
	PushFlag =				0x20000000,	// There is more to come; the message has been truncated
	LogMessageLowBit =		0x40000000,	// Log level consists of two bits this is the low bit
	LogMessageHighBit =		0x80000000,	// Log level consists of two bits this is the high bit

	// Common combinations
	NoDestinationMessage = 0,												// A message that is going nowhere
	GenericMessage = UsbMessage | AuxMessage | HttpMessage | TelnetMessage,	// A message that is to be sent to the web, Telnet, USB and panel
	LogOff = LogMessageLowBit | LogMessageHighBit,							// Log level "off (3): do not log this message
	LogWarn = LogMessageHighBit,											// Log level "warn" (2): all messages of type Error and Warning are logged
	LogInfo = LogMessageLowBit,												// Log level "info" (1): all messages of level "warn" plus info messages
	LoggedGenericMessage = GenericMessage | LogWarn,						// A GenericMessage that is also logged
	DirectAuxMessage = AuxMessage | RawMessageFlag,							// Direct message to PanelDue
	ErrorMessage = GenericMessage | LogWarn | ErrorMessageFlag,				// An error message
	WarningMessage = GenericMessage | LogWarn | WarningMessageFlag,			// A warning message
	FirmwareUpdateMessage = UsbMessage | ImmediateAuxMessage,				// A message that conveys progress of a firmware update
	FirmwareUpdateErrorMessage = FirmwareUpdateMessage | ErrorMessageFlag,	// A message that reports an error during a firmware update
	NetworkInfoMessage = UsbMessage | AuxMessage | LogWarn				 	// A message that conveys information about the state of the network interface
};

inline constexpr MessageType AddLogDebug(MessageType mt) noexcept
{
	// Debug level has no flags set such that any non-flagged message automatically
	// is part of this log level - force it by removing the existing flags
	return (MessageType)(mt & ~(LogMessageLowBit | LogMessageHighBit));
}

inline constexpr MessageType AddLogWarn(MessageType mt) noexcept
{
	// Since increasing log levels have lower numbers we need to delete
	// any existing log flags first - otherwise this could lead to MessageLogLevel
	// rising to 3 which is equivalent to OFF
	return (MessageType)(AddLogDebug(mt) | LogWarn);
}

inline constexpr MessageType AddLogInfo(MessageType mt) noexcept
{
	// Since increasing log levels have lower numbers we need to delete
	// any existing log flags first - otherwise this could lead to MessageLogLevel
	// rising to 3 which is equivalent to OFF
	return (MessageType)(AddLogDebug(mt) | LogInfo);
}

inline constexpr MessageType RemoveLogging(MessageType mt) noexcept
{
	return (MessageType)(mt | LogOff);
}

inline constexpr MessageType AddError(MessageType mt) noexcept
{
	return AddLogWarn((MessageType)(mt | ErrorMessageFlag));
}

inline constexpr MessageType AddWarning(MessageType mt) noexcept
{
	return AddLogWarn((MessageType)(mt | WarningMessageFlag));
}

#endif /* MESSAGETYPE_H_ */
