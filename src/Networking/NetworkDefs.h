/*
 * NetworkDefs.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_NETWORKING_NETWORKDEFS_H_
#define SRC_NETWORKING_NETWORKDEFS_H_

#include <cstdint>
#include <cstddef>

class NetworkBuffer;

typedef uint8_t SocketNumber;

typedef uint16_t Port;
typedef uint8_t NetworkProtocol;

const uint8_t DefaultMacAddress[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };

const uint8_t DefaultIpAddress[4] = { 0, 0, 0, 0 };
const uint8_t DefaultNetMask[4] = { 255, 255, 255, 0 };
const uint8_t DefaultGateway[4] = { 0, 0, 0, 0 };

const size_t NumProtocols = 3;					// number of network protocols we support, not counting FtpDataProtocol or AnyProtocol
const NetworkProtocol HttpProtocol = 0, FtpProtocol = 1, TelnetProtocol = 2, FtpDataProtocol = 3, AnyProtocol = 255;

const size_t NumTcpPorts = NumProtocols + 1;
const Port DefaultHttpPort = 80;
const Port DefaultFtpPort = 21;
const Port DefaultTelnetPort = 23;

const Port DefaultPortNumbers[NumProtocols] = { DefaultHttpPort, DefaultFtpPort, DefaultTelnetPort };
const char * const ProtocolNames[NumProtocols] = { "HTTP", "FTP", "TELNET" };

const size_t NetworkBufferCount = 6;			// number of 2K network buffers
const size_t SsidBufferLength = 32;				// maximum characters in an SSID

#endif /* SRC_NETWORKING_NETWORKDEFS_H_ */
