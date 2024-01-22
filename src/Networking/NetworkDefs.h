/*
 * NetworkDefs.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_NETWORKING_NETWORKDEFS_H_
#define SRC_NETWORKING_NETWORKDEFS_H_

#include <General/IPAddress.h>

class NetworkBuffer;

typedef uint8_t SocketNumber;
typedef uint16_t TcpPort;

// Network protocols
typedef uint8_t NetworkProtocol;

constexpr size_t NumSelectableProtocols = 5;	// number of network protocols that the user can select (excludes FtpDataProtocol, MdnsProtocol, AnyProtocol)
constexpr size_t NumTcpProtocols = 6;			// the selectable protocols plus any additional TCP protocols

constexpr NetworkProtocol
	// User-selectable protocols
	HttpProtocol = 0, FtpProtocol = 1, TelnetProtocol = 2, MulticastDiscoveryProtocol = 3, MqttProtocol = 4,
	// Other TCP protocols that need listeners when we are using Lwip
	FtpDataProtocol = 5,
	// Remaining protocols
	MdnsProtocol = 6, AnyProtocol = 255;

constexpr TcpPort DefaultHttpPort = 80;
constexpr TcpPort DefaultFtpPort = 21;
constexpr TcpPort DefaultTelnetPort = 23;
#if SUPPORT_MULTICAST_DISCOVERY
constexpr TcpPort DefaultMulticastDiscoveryPort = 10002;	// this is actually a UDP port
#endif
#if SUPPORT_MQTT
constexpr TcpPort DefaultMqttPort = 1883;
#endif

constexpr TcpPort DefaultPortNumbers[NumSelectableProtocols] =
{
	DefaultHttpPort, DefaultFtpPort, DefaultTelnetPort,
#if SUPPORT_MULTICAST_DISCOVERY
	DefaultMulticastDiscoveryPort,
#else
	0,
#endif
#if SUPPORT_MQTT
	DefaultMqttPort,
#else
	0,
#endif
};
constexpr const char *_ecv_array ProtocolNames[NumSelectableProtocols] =
{
	"HTTP", "FTP", "TELNET",
#if SUPPORT_MULTICAST_DISCOVERY
	"Multicast Discovery",
#else
	"",
#endif
#if SUPPORT_MQTT
	"MQTT",
#else
	"",
#endif
};

struct MacAddress
{
public:
	uint8_t bytes[6];

	uint32_t LowWord() const noexcept;
	uint16_t HighWord() const noexcept;

	void SetFromBytes(const uint8_t mb[6]) noexcept;
	void SetDefault() noexcept { SetFromBytes(defaultBytes); }

private:
	static constexpr uint8_t defaultBytes[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };
};

constexpr IPAddress DefaultIpAddress;				// will be initialised to 0 by constructor
constexpr IPAddress DefaultNetMask(0x00FFFFFF);		// equivalent to 255.255.255.0. Use constexpr constructor to avoid it being allocated in RAM.
constexpr IPAddress DefaultGateway;					// will be initialised to 0 by constructor
constexpr uint32_t AcceptAnyIp = 0;

constexpr uint8_t MdnsMacAddress[6] = { 0x01, 0x00, 0x5E, 0x00, 0x00, 0xFB };
constexpr uint8_t MdnsIPAddress[4] = { 224, 0, 0, 251 };
constexpr TcpPort MdnsPort = 5353;

#if SAME70 || SAME5x
constexpr size_t NetworkBufferCount = 10;			// number of 2K network buffers
#else
constexpr size_t NetworkBufferCount = 6;			// number of 2K network buffers
#endif

constexpr size_t SsidBufferLength = 32;				// maximum characters in an SSID

#endif /* SRC_NETWORKING_NETWORKDEFS_H_ */
