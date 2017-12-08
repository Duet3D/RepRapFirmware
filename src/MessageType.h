/*
 * MessageType.h
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#ifndef MESSAGETYPE_H_
#define MESSAGETYPE_H_

#include <cstdint>

// Supported message destinations. This is now a bitmap.
enum MessageType : uint16_t
{
	// Destinations
	UsbMessage = 0x01,					// A message that is to be sent in non-blocking mode to the host via USB
	BlockingUsbMessage = 0x02,			// A message to be sent ot USB in blocking mode
	LcdMessage = 0x04,					// A message that is to be sent to the panel
	ImmediateLcdMessage = 0x08,			// A message to be sent to LCD in immediate mode
	HttpMessage = 0x10,					// A message that is to be sent to the web (HTTP)
	TelnetMessage = 0x20,				// A message that is to be sent to a Telnet client
	AuxMessage = 0x40,					// A message that is to be sent to the second auxiliary device
	LogMessage = 0x80,					// A message to be written to the log file

	// Special indicators. These are not processed when calling the version of Platform::Message that takes an OutputBuffer.
	ErrorMessageFlag = 0x100,			// This is an error message
	WarningMessageFlag = 0x200,			// This is a warning message

	// Common combinations
	DebugMessage = BlockingUsbMessage,										// A debug message to send in blocking mode to USB
	GenericMessage = UsbMessage | LcdMessage | HttpMessage | TelnetMessage,	// A message that is to be sent to the web, Telnet, USB and panel
	LoggedGenericMessage = GenericMessage | LogMessage,						// A GenericMessage that is also logged
	ErrorMessage = GenericMessage | LogMessage | ErrorMessageFlag,			// An error message
	WarningMessage = GenericMessage | LogMessage | WarningMessageFlag,		// A warning message
	FirmwareUpdateMessage = UsbMessage | ImmediateLcdMessage,				// A message that conveys progress of a firmware update
	FirmwareUpdateErrorMessage = FirmwareUpdateMessage | ErrorMessageFlag,	// A message that reports an error during a firmware update
	NetworkInfoMessage = UsbMessage | LcdMessage | LogMessage,			 	// A message that conveys information about the state of the network interface

	// Masks
	MessageDestinationMask = 0x00FF,
	MessageFlagsMask = 0x00F0
};

inline MessageType AddError(MessageType mt)
{
	return (MessageType)(mt | ErrorMessageFlag);
}

#endif /* MESSAGETYPE_H_ */
