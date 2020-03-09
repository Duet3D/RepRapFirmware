#ifndef NETWORK_H
#define NETWORK_H

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "General/IPAddress.h"
#include <Networking/NetworkDefs.h>

// The main network class that drives the network.
class Network
{
public:
	Network(Platform& p) noexcept { macAddress.SetDefault(); }
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
	GCodeResult SetMacAddress(unsigned int interface, const MacAddress& mac, const StringRef& reply) noexcept;
	const MacAddress& GetMacAddress(unsigned int interface) const noexcept { return macAddress; }

	void SetHostname(const char *name) const noexcept { }
	bool IsWiFiInterface(unsigned int interface) const noexcept { return false; }

	void HandleHttpGCodeReply(const char *msg) noexcept { }
	void HandleTelnetGCodeReply(const char *msg) noexcept { }
	void HandleHttpGCodeReply(OutputBuffer *buf) noexcept;
	void HandleTelnetGCodeReply(OutputBuffer *buf) noexcept;
	uint32_t GetHttpReplySeq() noexcept { return 0; }

private:
	MacAddress macAddress;
};

#endif
