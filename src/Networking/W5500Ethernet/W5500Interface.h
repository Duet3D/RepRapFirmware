/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by chrishamm

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include "NetworkInterface.h"
#include "NetworkDefs.h"
#include "MessageType.h"

class NetworkResponder;
class HttpResponder;
class FtpResponder;
class TelnetResponder;
class W5500Socket;

// We have 8 sockets available on the W5500.
const size_t NumHttpSockets = 4;				// sockets 0-3 are for HTTP
const SocketNumber FtpSocketNumber = 4;
const SocketNumber FtpDataSocketNumber = 5;		// TODO can we allocate this dynamically when required, to allow more http sockets most of the time?
const SocketNumber TelnetSocketNumber = 6;
const size_t NumW5500TcpSockets = 7;
const SocketNumber DhcpSocketNumber = 7;		// TODO can we allocate this dynamically when required, to allow more http sockets most of the time?

class Platform;

// The main network class that drives the network.
class W5500Interface : public NetworkInterface
{
public:
	W5500Interface(Platform& p);

	void Init() override;
	void Activate() override;
	void Exit() override;
	void Spin(bool full) override;
	void Diagnostics(MessageType mtype) override;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) override;			// enable or disable the network
	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) override;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) override;
	GCodeResult ReportProtocols(const StringRef& reply) const override;

	GCodeResult GetNetworkState(const StringRef& reply) override;
	int EnableState() const override;
	bool InNetworkStack() const override { return false; }
	bool IsWiFiInterface() const override { return false; }

	void UpdateHostname(const char *name) override { }
	const uint8_t *GetIPAddress() const override { return ipAddress; }
	void SetMacAddress(const uint8_t mac[]) override;
	const uint8_t *GetMacAddress() const override { return macAddress; }

	void OpenDataPort(Port port) override;
	void TerminateDataPort() override;

private:
	enum class NetworkState
	{
		disabled,					// Network disabled
		enabled,					// Network enabled but not started yet
		establishingLink,			// starting up, waiting for link
		obtainingIP,				// link established, waiting for DHCP
		connected,					// just established a connection
		active						// network running
	};

	void Start();
	void Stop();
	void InitSockets();
	void TerminateSockets();

	void StartProtocol(NetworkProtocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(NetworkProtocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const
	pre(protocol < NumProtocols);

	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);
	Platform& platform;
	uint32_t lastTickMillis;

	W5500Socket *sockets[NumW5500TcpSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

	NetworkState state;
	bool activated;
	bool usingDhcp;

	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	uint8_t macAddress[6];
};

#endif
