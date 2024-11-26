/*
 * Network.h
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_NETWORK_NETWORK_H_
#define SRC_NETWORK_NETWORK_H_

#include <RepRapFirmware.h>
#include "NetworkDefs.h"
#include <RTOSIface/RTOSIface.h>
#include <ObjectModel/ObjectModel.h>
#include <General/NamedEnum.h>

#if defined(DUET3_MB6HC) && HAS_WIFI_NETWORKING
const size_t MaxNetworkInterfaces = 2;
#elif defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(DUET_NG) || defined(DUET_M) || defined(PCCB) || defined(DUET3MINI)
const size_t MaxNetworkInterfaces = 1;
#else
# error Wrong Network.h file included
#endif

#if SAME70
const size_t NumHttpResponders = 6;		// the number of concurrent HTTP requests we can process
const size_t NumTelnetResponders = 2;	// the number of concurrent Telnet sessions we support
#else
// Limit the number of HTTP responders to 4 because they take around 2.5K of memory each
# if defined(DUET_NG) || defined(DUET_M)
// We now support only 3 HTTP sockets on Duet 2 Ethernet, so it's probably not worth supporting 4 responders and we would rather have the extra RAM
const size_t NumHttpResponders = 3;		// the number of concurrent HTTP requests we can process
# else
const size_t NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process
# endif
const size_t NumTelnetResponders = 1;	// the number of concurrent Telnet sessions we support
#endif // not SAME70

const size_t NumFtpResponders = 1;		// the number of concurrent FTP sessions we support

// Forward declarations
class NetworkResponder;
class NetworkClient;
class NetworkInterface;
class Socket;
class WiFiInterface;
class WifiFirmwareUploader;

NamedEnum(NetworkState, uint8_t,
	disabled,					// Network disabled
	enabled,					// Network enabled but not started yet
	starting1,					// starting up (used by WiFi networking)
	starting2,					// starting up (used by WiFi networking)
	changingMode,				// running and in the process of switching between modes (used by WiFi networking)
	idle,						// wifi module is running but in idle mode
	establishingLink,			// starting up, waiting for link
	obtainingIP,				// link established, waiting for DHCP
	connected,					// just established a connection
	active						// network running
);

// The main network class that drives the network.
class Network INHERIT_OBJECT_MODEL
{
public:
	explicit Network(Platform& p) noexcept;
	Network(const Network&) = delete;

	void Init() noexcept;
	void Activate() noexcept;
	void Exit() noexcept;
#if HAS_NETWORKING
	[[noreturn]] void Spin() noexcept;
#endif
	void Diagnostics(MessageType mtype) noexcept;
	unsigned int GetNumNetworkInterfaces() const noexcept;
	bool IsWiFiInterface(unsigned int interface) const noexcept;

#if HAS_NETWORKING
	const NetworkInterface *_ecv_null GetInterface(unsigned int interface) { return interfaces[interface]; }
#endif

#if defined(DUET3_MB6HC)
	void CreateAdditionalInterface() noexcept;
#endif

	GCodeResult EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply) noexcept;
	GCodeResult EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept;
	GCodeResult DisableProtocol(unsigned int interface, NetworkProtocol protocol, const StringRef& reply) noexcept;
	GCodeResult ReportProtocols(unsigned int interface, const StringRef& reply) const noexcept;

	// WiFi interfaces
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & longReply);
	WifiFirmwareUploader *_ecv_null GetWifiUploader() const noexcept;
	void ResetWiFiForUpload(bool external) noexcept;
	const char *_ecv_array GetWiFiServerVersion() const noexcept;

#if HAS_NETWORKING
	// Network Protocol
	GCodeResult ConfigureNetworkProtocol(GCodeBuffer& gb, const StringRef& reply);
#endif

	// Global settings
	GCodeResult GetNetworkState(unsigned int interface, const StringRef& reply) noexcept;
	int EnableState(unsigned int interface) const noexcept;

	IPAddress GetIPAddress(unsigned int interface) const noexcept;

#if HAS_SBC_INTERFACE
	void SetReportedIPAddress(IPAddress p_ipAddress) noexcept;
#endif

#if HAS_NETWORKING
	void SetEthernetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept;
	IPAddress GetNetmask(unsigned int interface) const noexcept;
	IPAddress GetGateway(unsigned int interface) const noexcept;
	bool UsingDhcp(unsigned int interface) const noexcept;
	GCodeResult SetMacAddress(unsigned int interface, const MacAddress& mac, const StringRef& reply) noexcept;
	const MacAddress& GetMacAddress(unsigned int interface) const noexcept;
	const char *_ecv_array GetHostname() const noexcept { return hostname; }
	void SetHostname(const char *_ecv_array name) noexcept;

	void TerminateResponders(const NetworkInterface *iface, NetworkProtocol protocol) noexcept;
#endif

#if SUPPORT_HTTP
	const char *_ecv_array _ecv_null GetCorsSite() const noexcept { return corsSite.IsEmpty() ? nullptr : corsSite.c_str(); }
	void SetCorsSite(const char *_ecv_array site) noexcept { corsSite.copy(site); }
#endif

	bool FindResponder(Socket *skt, NetworkProtocol protocol) noexcept;

#if HAS_CLIENTS
	bool StartClient(NetworkInterface *interface, NetworkProtocol protocol) noexcept;
	void StopClient(NetworkInterface *interface, NetworkProtocol protocol) noexcept;
#endif

	void HandleHttpGCodeReply(const char *_ecv_array msg) noexcept;
	void HandleTelnetGCodeReply(const char *_ecv_array msg) noexcept;
	void HandleHttpGCodeReply(OutputBuffer *buf) noexcept;
	void HandleTelnetGCodeReply(OutputBuffer *buf) noexcept;

#if SUPPORT_MQTT
	void MqttPublish(const char *_ecv_array msg, const char *_ecv_array topic, int qos, bool retain, bool dup) noexcept;
#endif

	uint32_t GetHttpReplySeq() noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	WiFiInterface *_ecv_null FindWiFiInterface() const noexcept;

	Platform& platform;

#if HAS_NETWORKING
	NetworkInterface *_ecv_null interfaces[MaxNetworkInterfaces];
#endif

#if HAS_RESPONDERS
	NetworkResponder *_ecv_from _ecv_null responders;
	NetworkResponder *_ecv_from _ecv_null nextResponderToPoll;
#endif

#if HAS_CLIENTS
	NetworkClient *_ecv_null clients;
#endif

#if SUPPORT_HTTP
	Mutex httpMutex;
#endif
#if SUPPORT_TELNET
	Mutex telnetMutex;
#endif

	uint32_t fastLoop, slowLoop;

#if SUPPORT_HTTP
	String<StringLength20> corsSite;
#endif

#ifdef DUET3_MB6HC
	unsigned int numActualNetworkInterfaces = 1;	// don't add a second interface until we know whether the board supports it
#endif

	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
#if HAS_SBC_INTERFACE
	IPAddress reportedIPAddress;					// IP address to use for 12864 displays in SBC mode
#endif
};

inline unsigned int Network::GetNumNetworkInterfaces() const noexcept
{
#if defined(DUET3_MB6HC)
	return numActualNetworkInterfaces;
#else
	return MaxNetworkInterfaces;
#endif
}

#endif /* SRC_NETWORK_NETWORK_H_ */
