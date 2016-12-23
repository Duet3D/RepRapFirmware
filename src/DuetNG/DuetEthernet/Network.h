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
#include "NetworkTransaction.h"

const uint8_t DefaultMacAddress[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };	// Need some sort of default...
const uint8_t DefaultIpAddress[4] = { 0, 0, 0, 0 };				// Need some sort of default...
const uint8_t DefaultNetMask[4] = { 255, 255, 255, 0 };
const uint8_t DefaultGateway[4] = { 0, 0, 0, 0 };

const uint16_t DefaultHttpPort = 80;
const uint16_t FTP_PORT = 21;
const uint16_t TELNET_PORT = 23;

// We have 8 sockets available on the W5500. We reserve one for DHCP, leaving 7 for TCP/IP transactions i.e. HTTP, FTP and Telnet.
const size_t NumTcpSockets = 7;
const uint8_t DhcpSocketNumber = 7;

class Platform;

// The main network class that drives the network.
class Network
{
public:
	const uint8_t *GetIPAddress() const;
	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);

	Network(Platform* p);
	void Init();
	void Activate();
	void Exit();
	void Spin();
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	bool Lock();
	void Unlock();
	bool InLwip() const;

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(uint16_t port);
	uint16_t GetHttpPort() const;

	void SetHostname(const char *name);

	// Interfaces for the Webserver

	NetworkTransaction *GetTransaction(const ConnectionState *cs = nullptr);

	void OpenDataPort(uint16_t port);
	uint16_t GetDataPort() const;
	void CloseDataPort();

	void SaveDataConnection();
	void SaveFTPConnection();
	void SaveTelnetConnection();

	bool AcquireFTPTransaction();
	bool AcquireDataTransaction();
	bool AcquireTelnetTransaction();

private:
	enum class NetworkState
	{
		disabled,					// WiFi not active
		enabled,					// WiFi enabled but not started yet
		establishingLink,			// starting up (waiting for initialisation)
		obtainingIP,
		active
	};

	Platform *platform;
    float longWait;
    uint32_t lastTickMillis;

	void AppendTransaction(NetworkTransaction* * list, NetworkTransaction *r);
	void PrependTransaction(NetworkTransaction* * list, NetworkTransaction *r);
	bool AcquireTransaction(ConnectionState *cs);

	NetworkTransaction * freeTransactions;
	NetworkTransaction * readyTransactions;
	NetworkTransaction * writingTransactions;

    uint16_t httpPort;
	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0

    NetworkState state;
    bool activated;
    bool usingDhcp;
};

#endif
