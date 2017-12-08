/*
 * Network.h
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_SAME70_NETWORK_H_
#define SRC_SAME70_NETWORK_H_

#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageFormats.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"

const int EthernetInterfaceIndex = 0;
const int WiFiInterfaceIndex = 1;
const size_t NumNetworkInterfaces = 2;

const size_t NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process
const size_t NumFtpResponders = 1;		// the number of concurrent FTP sessions we support
const size_t NumTelnetResponders = 2;	// the number of concurrent Telnet sessions we support


// Forward declarations
class Platform;
class NetworkResponder;
class NetworkInterface;
class Socket;
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
	bool InLwip() const;

	void Start(int interface);
	void Stop(int interface);
	bool IsWiFiInterface(int interface) const;

	void EnableProtocol(int interface, int protocol, int port, int secure, StringRef& reply);
	void DisableProtocol(int interface, int protocol, StringRef& reply);
	void ReportProtocols(int interface, StringRef& reply) const;

	// WiFi interfaces
	void EnableWiFi(int mode, const StringRef& ssid, StringRef& reply);
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer& gb, StringRef& reply, OutputBuffer*& longReply);
	WifiFirmwareUploader& GetWifiUploader();
	void ResetWiFiForUpload(bool external);

	// Global settings
	void EnableEthernet(int mode, StringRef& reply);
	bool GetNetworkState(int interface, StringRef& reply);
	int EnableState(int interface) const;

	const uint8_t *GetIPAddress(int interface) const;
	void SetIPAddress(int interface, const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);

	const char *GetHostname() const { return hostname; }
	void SetHostname(const char *name);

	bool FindResponder(Socket *skt, Protocol protocol);

	void HandleHttpGCodeReply(const char *msg);
	void HandleTelnetGCodeReply(const char *msg);
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq();

private:
	Platform& platform;
	uint32_t longWait;

	NetworkInterface *interfaces[NumNetworkInterfaces];
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;

	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
};

#endif
