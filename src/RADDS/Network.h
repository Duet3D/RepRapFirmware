#ifndef NETWORK_H
#define NETWORK_H

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"

const uint8_t DefaultIpAddress[4] = { 0, 0, 0, 0 };
const uint8_t DefaultNetMask[4] = { 0, 0, 0, 0 };
const uint8_t DefaultGateway[4] = { 0, 0, 0, 0 };
const uint8_t macAddress[6] = { 0, 0, 0, 0, 0, 0 };

const size_t SsidBufferLength = 32;				// maximum characters in an SSID

// The main network class that drives the network.
class Network
{
public:
	Network(Platform& p) { }
	void Init() const { }
	void Activate() const { }
	void Exit() const { }
	void Spin(bool full) const { }
	void Interrupt() const { }
	void Diagnostics(MessageType mtype) const { }

	GCodeResult EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply);
	GCodeResult EnableProtocol(unsigned int interface, int protocol, int port, bool secure, const StringRef& reply);
	GCodeResult DisableProtocol(unsigned int interface, int protocol, const StringRef& reply);
	GCodeResult ReportProtocols(unsigned int interface, const StringRef& reply) const;

	GCodeResult GetNetworkState(unsigned int interface, const StringRef& reply);

	void SetEthernetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]) { }
	void SetMacAddress(unsigned int interface, const uint8_t mac[]) { }
	const uint8_t *GetMacAddress(unsigned int interface) const { return macAddress; }

	void SetHostname(const char *name) const { }
	bool IsWiFiInterface(unsigned int interface) const { return false; }

	void HandleHttpGCodeReply(const char *msg) { }
	void HandleTelnetGCodeReply(const char *msg) { }
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq() { return 0; }
};

#endif
