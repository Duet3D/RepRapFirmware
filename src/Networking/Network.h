/*
 * Network.h
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_NETWORK_NETWORK_H_
#define SRC_NETWORK_NETWORK_H_

#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "RTOSIface/RTOSIface.h"
#include "ObjectModel/ObjectModel.h"

#if defined(DUET3_V03)
const size_t NumNetworkInterfaces = 2;
#elif defined(SAME70XPLD) || defined(DUET3_V05) || defined(DUET3_V06) || defined(DUET_NG) || defined(DUET_M) || defined(__LPC17xx__)
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
# endif

const size_t NumFtpResponders = 1;		// the number of concurrent FTP sessions we support
#endif

// Forward declarations
class NetworkResponder;
class NetworkInterface;
class Socket;
class WiFiInterface;
class WifiFirmwareUploader;

// The main network class that drives the network.
class Network INHERIT_OBJECT_MODEL
{
public:
	Network(Platform& p) noexcept;

	void Init() noexcept;
	void Activate() noexcept;
	void Exit() noexcept;
	void Spin() noexcept;
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
	void SetMacAddress(unsigned int interface, const uint8_t mac[]) noexcept;
	const uint8_t *GetMacAddress(unsigned int interface) const noexcept;

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

	NetworkInterface *interfaces[NumNetworkInterfaces];
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;

	Mutex httpMutex;
#if SUPPORT_TELNET
	Mutex telnetMutex;
#endif

	uint32_t fastLoop, slowLoop;

	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
};

#endif /* SRC_NETWORK_NETWORK_H_ */
