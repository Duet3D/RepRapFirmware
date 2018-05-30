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
#include "RTOSIface.h"

#if defined(SAME70_TEST_BOARD)
const size_t NumNetworkInterfaces = 2;
#elif defined(DUET_NG) || defined(DUET_M)
const size_t NumNetworkInterfaces = 1;
#else
# error Wrong Network.h file included
#endif

const size_t NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process
const size_t NumFtpResponders = 1;		// the number of concurrent FTP sessions we support
const size_t NumTelnetResponders = 2;	// the number of concurrent Telnet sessions we support

// Forward declarations
class NetworkResponder;
class NetworkInterface;
class Socket;
class WiFiInterface;
class WifiFirmwareUploader;

// The main network class that drives the network.
class Network
{
public:
	Network(Platform& p);

	void Init();
	void Activate();
	void Exit();
	void Spin(bool full);
	void Interrupt();
	void Diagnostics(MessageType mtype);
	bool InNetworkStack() const;
	bool IsWiFiInterface(unsigned int interface) const;

	GCodeResult EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply);
	GCodeResult EnableProtocol(unsigned int interface, NetworkProtocol protocol, int port, int secure, const StringRef& reply);
	GCodeResult DisableProtocol(unsigned int interface, NetworkProtocol protocol, const StringRef& reply);
	GCodeResult ReportProtocols(unsigned int interface, const StringRef& reply) const;

	// WiFi interfaces
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& longReply);
	WifiFirmwareUploader *GetWifiUploader() const;
	void ResetWiFiForUpload(bool external);
	const char* GetWiFiServerVersion() const;

	// Global settings
	GCodeResult GetNetworkState(unsigned int interface, const StringRef& reply);
	int EnableState(unsigned int interface) const;

	void SetEthernetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);
	const char *GetHostname() const { return hostname; }
	void SetHostname(const char *name);
	void SetMacAddress(unsigned int interface, const uint8_t mac[]);
	const uint8_t *GetMacAddress(unsigned int interface) const;

	bool FindResponder(Socket *skt, NetworkProtocol protocol);

	void HandleHttpGCodeReply(const char *msg);
	void HandleTelnetGCodeReply(const char *msg);
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq();

private:
	WiFiInterface *FindWiFiInterface() const;

	Platform& platform;

	NetworkInterface *interfaces[NumNetworkInterfaces];
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;

	Mutex httpMutex, telnetMutex;

	uint32_t fastLoop, slowLoop;

	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
};

#endif /* SRC_NETWORK_NETWORK_H_ */
