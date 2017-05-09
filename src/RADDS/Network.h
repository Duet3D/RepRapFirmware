#ifndef NETWORK_H
#define NETWORK_H

#include "RepRapFirmware.h"
#include "MessageType.h"

const uint8_t DefaultMacAddress[6] = { 0, 0, 0, 0, 0, 0 };
const uint8_t DefaultIpAddress[4] = { 0, 0, 0, 0 };
const uint8_t DefaultNetMask[4] = { 0, 0, 0, 0 };
const uint8_t DefaultGateway[4] = { 0, 0, 0, 0 };

// The main network class that drives the network.
class Network
{
public:
	Network(Platform& p) { }
	void Init() const { }
	void Activate() const { }
	void Enable(int mode, StringRef& reply);
	bool GetNetworkState(StringRef& reply);
	void Exit() const { }
	void Spin(bool full) const { }
	void Interrupt() const { }
	void Diagnostics(MessageType mtype) const { }

	void EnableProtocol(int protocol, int port, bool secure, StringRef& reply) { }
	void DisableProtocol(int protocol, StringRef& reply) { }
	void ReportProtocols(StringRef& reply) const;

	bool IsEnabled() const { return false; }
	bool InLwip() const { return false; }
	void SetHostname(const char *name) const { }
	void SetHttpPort(uint16_t port) const { }
	const uint8_t *GetIPAddress() const;

	void HandleHttpGCodeReply(const char *msg) { }
	void HandleTelnetGCodeReply(const char *msg) { }
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq() { return 0; }
};

#endif
