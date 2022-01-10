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

#if defined(DUET3_V03)
const size_t NumNetworkInterfaces = 2;
#elif defined(DUET3_MB6HC) || defined(DUET3_MB6XD) || defined(DUET_NG) || defined(DUET_M) || defined(__LPC17xx__) || defined(PCCB) || defined(DUET3MINI)
const size_t NumNetworkInterfaces = 1;
#else
# error Wrong Network.h file included
#endif

#if defined(__LPC17xx__)
// Only 2 http responders as we are tight on memory.
const size_t NumHttpResponders = 2;		// the number of concurrent HTTP requests we can process
const size_t NumFtpResponders = 0;		// the number of concurrent FTP sessions we support
const size_t NumTelnetResponders = 0;	// the number of concurrent Telnet sessions we support
#else

# if SAME70
const size_t NumHttpResponders = 6;		// the number of concurrent HTTP requests we can process
const size_t NumTelnetResponders = 2;	// the number of concurrent Telnet sessions we support
# else
// Limit the number of HTTP responders to 4 because they take around 2K of memory each
const size_t NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process
const size_t NumTelnetResponders = 1;	// the number of concurrent Telnet sessions we support
# endif // not SAME70

const size_t NumFtpResponders = 1;		// the number of concurrent FTP sessions we support
#endif // not __LPC17xx__

#define HAS_RESPONDERS	(SUPPORT_HTTP || SUPPORT_FTP || SUPPORT_TELNET)

// Forward declarations
class NetworkResponder;
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
	establishingLink,			// starting up, waiting for link
	obtainingIP,				// link established, waiting for DHCP
	connected,					// just established a connection
	active						// network running
);

// The main network class that drives the network.
class Network INHERIT_OBJECT_MODEL
{
public:
	Network(Platform& p) noexcept;
	Network(const Network&) = delete;

	void Init() noexcept;
	void Activate() noexcept;
	void Exit() noexcept;
#if HAS_NETWORKING
	[[noreturn]] void Spin() noexcept;
#endif
	void Diagnostics(MessageType mtype) noexcept;
	bool IsWiFiInterface(unsigned int interface) const noexcept;

	GCodeResult EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply) noexcept;
	GCodeResult EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept;
	GCodeResult DisableProtocol(unsigned int interface, NetworkProtocol protocol, const StringRef& reply) noexcept;
	GCodeResult ReportProtocols(unsigned int interface, const StringRef& reply) const noexcept;

	// WiFi interfaces
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& longReply);
	WifiFirmwareUploader *GetWifiUploader() const noexcept;
	void ResetWiFiForUpload(bool external) noexcept;
	const char* GetWiFiServerVersion() const noexcept;

	// Global settings
	GCodeResult GetNetworkState(unsigned int interface, const StringRef& reply) noexcept;
	int EnableState(unsigned int interface) const noexcept;

	void SetEthernetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept;
	IPAddress GetIPAddress(unsigned int interface) const noexcept;
	const char *GetHostname() const noexcept { return hostname; }
	void SetHostname(const char *name) noexcept;
	GCodeResult SetMacAddress(unsigned int interface, const MacAddress& mac, const StringRef& reply) noexcept;
	const MacAddress& GetMacAddress(unsigned int interface) const noexcept;

#if SUPPORT_HTTP
	const char *GetCorsSite() const noexcept { return corsSite.IsEmpty() ? nullptr : corsSite.c_str(); }
	void SetCorsSite(const char *site) noexcept { corsSite.copy(site); }
#endif

	bool FindResponder(Socket *skt, NetworkProtocol protocol) noexcept;

	void HandleHttpGCodeReply(const char *msg) noexcept;
	void HandleTelnetGCodeReply(const char *msg) noexcept;
	void HandleHttpGCodeReply(OutputBuffer *buf) noexcept;
	void HandleTelnetGCodeReply(OutputBuffer *buf) noexcept;
	uint32_t GetHttpReplySeq() noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(interfaces)

private:
	WiFiInterface *FindWiFiInterface() const noexcept;

	Platform& platform;

#if HAS_NETWORKING
	NetworkInterface *interfaces[NumNetworkInterfaces];
#endif

#if HAS_RESPONDERS
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;
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
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
};

#endif /* SRC_NETWORK_NETWORK_H_ */
