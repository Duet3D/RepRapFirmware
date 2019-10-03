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
#include <General/IPAddress.h>

class NetworkBuffer;

typedef uint8_t SocketNumber;

typedef uint16_t Port;
typedef uint8_t NetworkProtocol;

constexpr uint8_t DefaultMacAddress[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };

constexpr IPAddress DefaultIpAddress;		// will be initialised to 0 by constructor
const IPAddress DefaultNetMask((const uint8_t[]){ 255, 255, 255, 0 });
constexpr IPAddress DefaultGateway;			// will be initialised to 0 by constructor

constexpr size_t NumProtocols = 3;					// number of network protocols we support, not counting FtpDataProtocol, MdnsProtocol or AnyProtocol
constexpr NetworkProtocol HttpProtocol = 0, FtpProtocol = 1, TelnetProtocol = 2, FtpDataProtocol = 3, MdnsProtocol = 4, AnyProtocol = 255;

constexpr size_t NumTcpPorts = NumProtocols + 1;
constexpr Port DefaultHttpPort = 80;
constexpr Port DefaultFtpPort = 21;
constexpr Port DefaultTelnetPort = 23;

constexpr Port DefaultPortNumbers[NumProtocols] = { DefaultHttpPort, DefaultFtpPort, DefaultTelnetPort };
constexpr const char * ProtocolNames[NumProtocols] = { "HTTP", "FTP", "TELNET" };

constexpr uint8_t MdnsMacAddress[6] = { 0x01, 0x00, 0x5E, 0x00, 0x00, 0xFB };
constexpr uint8_t MdnsIPAddress[4] = { 224, 0, 0, 251 };
constexpr Port MdnsPort = 5353;

#if defined(__LPC17xx__)
const size_t NetworkBufferCount = 2;				// number of MSS sized buffers
#else
constexpr size_t NetworkBufferCount = 6;			// number of 2K network buffers
#endif

constexpr size_t SsidBufferLength = 32;				// maximum characters in an SSID

#endif /* SRC_NETWORKING_NETWORKDEFS_H_ */
