#ifndef NETWORK_H
#define NETWORK_H

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "General/IPAddress.h"

const IPAddress DefaultIpAddress;
const IPAddress DefaultNetMask;
const IPAddress DefaultGateway;
const uint8_t macAddress[6] = { 0, 0, 0, 0, 0, 0 };

const uint8_t DefaultMacAddress[6] = { 0, 0, 0, 0, 0, 0 };
const size_t SsidBufferLength = 32;				// maximum characters in an SSID

// The main network class that drives the network.
class Network
{
public:
	Network(Platform& p) noexcept { }
	void Init() const noexcept { }
	void Activate() const noexcept { }
	void Exit() const noexcept { }
	void Diagnostics(MessageType mtype) const noexcept { }

	GCodeResult EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply) noexcept;
	GCodeResult EnableProtocol(unsigned int interface, int protocol, int port, bool secure, const StringRef& reply) noexcept;
	GCodeResult DisableProtocol(unsigned int interface, int protocol, const StringRef& reply) noexcept;
	GCodeResult ReportProtocols(unsigned int interface, const StringRef& reply) const noexcept;

	GCodeResult GetNetworkState(unsigned int interface, const StringRef& reply) noexcept;

	void SetEthernetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept { }
	void SetMacAddress(unsigned int interface, const uint8_t mac[]) noexcept { }
	const uint8_t *GetMacAddress(unsigned int interface) const noexcept { return macAddress; }

	void SetHostname(const char *name) const noexcept { }
	bool IsWiFiInterface(unsigned int interface) const noexcept { return false; }

	void HandleHttpGCodeReply(const char *msg) noexcept { }
	void HandleTelnetGCodeReply(const char *msg) noexcept { }
	void HandleHttpGCodeReply(OutputBuffer *buf) noexcept;
	void HandleTelnetGCodeReply(OutputBuffer *buf) noexcept;
	uint32_t GetHttpReplySeq() noexcept { return 0; }
};

#endif
