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
	Network(Platform* p) { };
	void Init() const { };
	void Activate() const { };
	void Disable() const { };
	void Enable() const { };
	void Exit() const { }
	void Spin(bool full) const { };
	void Interrupt() const { };
	void Diagnostics(MessageType mtype) const { };

	bool IsEnabled() const { return false; }
	bool InLwip() const { return false; }
	void SetHostname(const char *name) const { };
	void SetHttpPort(uint16_t port) const { };
	uint16_t GetHttpPort() const { return (uint16_t)0; }
	const uint8_t *GetIPAddress() const;
};

#endif
