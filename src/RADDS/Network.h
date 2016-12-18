#ifndef NETWORK_H
#define NETWORK_H

#include <inttypes.h>
#include "Platform.h"

const uint8_t MAC_ADDRESS[6] = { 0, 0, 0, 0, 0, 0 };
const uint8_t IP_ADDRESS[4] = { 0, 0, 0, 0 };
const uint8_t NET_MASK[4] = { 0, 0, 0, 0 };
const uint8_t GATE_WAY[4] = { 0, 0, 0, 0 };

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
	void Spin() const { };
	void Interrupt() const { };
	void Diagnostics(MessageType mtype) const { };

	boolean IsEnabled() const { return false; }
	boolean InLwip() const { return false; }
	void SetHostname(const char *name) const { };
	void SetHttpPort(uint16_t port) const { };
	uint16_t GetHttpPort() const { return (uint16_t)0; }
	const uint8_t *GetIPAddress() const;
};

#endif
