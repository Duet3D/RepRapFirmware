/*
 * MessageType.h
 *
 *  Created on: 21 May 2016
 *      Author: David
 */

#ifndef MESSAGETYPE_H_
#define MESSAGETYPE_H_

// Supported message destinations
enum MessageType
{
	AUX_MESSAGE,						// A message that is to be sent to the panel
	AUX2_MESSAGE,						// A message that is to be sent to the second auxiliary device
	HOST_MESSAGE,						// A message that is to be sent in non-blocking mode to the host via USB
	DEBUG_MESSAGE,						// A debug message to send in blocking mode to USB
	HTTP_MESSAGE,						// A message that is to be sent to the web (HTTP)
	TELNET_MESSAGE,						// A message that is to be sent to a Telnet client
	GENERIC_MESSAGE,					// A message that is to be sent to the web, USB and panel
	FIRMWARE_UPDATE_MESSAGE				// A message that conveys progress of a firmware update
};

#endif /* MESSAGETYPE_H_ */
