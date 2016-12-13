/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <cstdint>
#include <cctype>
#include <cstring>
#include <cstdlib>

#include "MessageType.h"

// Return code definitions
const uint32_t rcNumber = 0x0000FFFF;
const uint32_t rcJson = 0x00010000;
const uint32_t rcKeepOpen = 0x00020000;

static const uint8_t IP_ADDRESS[4] = { 192, 168, 1, 10 };				// Need some sort of default...
static const uint8_t NET_MASK[4] = { 255, 255, 255, 0 };
static const uint8_t GATE_WAY[4] = { 192, 168, 1, 1 };
static const uint16_t DEFAULT_HTTP_PORT = 80;

class TransactionBuffer;
class WifiFirmwareUploader;
class Platform;

// The main network class that drives the network.
class Network
{
public:
	const uint8_t *IPAddress() const;
	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);

	Network(Platform* p);
	void Init();
	void Activate();
	void Exit();
	void Spin();
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	bool InLwip() const { return false; }

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(uint16_t port);
	uint16_t GetHttpPort() const;

	void SetHostname(const char *name);

private:
	uint16_t httpPort;
	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
};

#endif
