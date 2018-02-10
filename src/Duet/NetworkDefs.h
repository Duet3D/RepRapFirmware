/*
 * NetworkDefs.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_NETWORKDEFS_H_
#define SRC_DUETNG_DUETETHERNET_NETWORKDEFS_H_

#include <cstdint>
#include <cstddef>

class NetworkTransaction;
class ConnectionState;
class NetworkBuffer;

// Definition of how a Connection is represented, for Webserver module
typedef ConnectionState *Connection;
const Connection NoConnection = nullptr;

typedef uint8_t SocketNumber;
const SocketNumber NoSocket = 255;

typedef uint16_t Port;

const uint8_t DefaultMacAddress[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };	// Need some sort of default...
const uint8_t DefaultIpAddress[4] = { 0, 0, 0, 0 };				// Need some sort of default...
const uint8_t DefaultNetMask[4] = { 255, 255, 255, 0 };
const uint8_t DefaultGateway[4] = { 0, 0, 0, 0 };

const Port DefaultHttpPort = 80;
const Port DefaultFtpPort = 21;
const Port DefaultTelnetPort = 23;

const size_t SsidBufferLength = 32;		// maximum size of a WiFi SSID

// MSS is defined in lwip
#include "Lwip/lwipopts.h"

#endif /* SRC_DUETNG_DUETETHERNET_NETWORKDEFS_H_ */
