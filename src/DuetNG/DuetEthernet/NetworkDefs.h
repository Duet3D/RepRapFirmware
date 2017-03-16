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
class Socket;
class NetworkBuffer;

// Definition of how a Connection is represented, for Webserver module
typedef Socket *Connection;
const Connection NoConnection = nullptr;

typedef uint8_t SocketNumber;
const SocketNumber NoSocket = 255;

typedef uint16_t Port;

const uint8_t DefaultMacAddress[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };
const uint8_t DefaultIpAddress[4] = { 0, 0, 0, 0 };
const uint8_t DefaultNetMask[4] = { 255, 255, 255, 0 };
const uint8_t DefaultGateway[4] = { 0, 0, 0, 0 };

const Port DefaultHttpPort = 80;
const Port DefaultFtpPort = 21;
const Port DefaultTelnetPort = 23;

const unsigned int TCP_MSS = 1460;

const size_t NetworkTransactionCount = 8;				// number of NetworkTransactions to be used for network IO
const size_t NetworkBufferCount = 16;					// number of 2K or 3K network buffers

// Define the following to use 3K buffers on the W5500 for the HTTP sockets and smaller buffers for everything else
// It doesn't seem to work, the chip keeps telling us that 1 byte is available.
//#define USE_3K_BUFFERS	1

#endif /* SRC_DUETNG_DUETETHERNET_NETWORKDEFS_H_ */
