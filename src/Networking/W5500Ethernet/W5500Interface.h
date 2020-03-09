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
class MdnsResponder;
class W5500Socket;

// We have 8 sockets available on the W5500.
const size_t NumW5500TcpSockets = 6;

const SocketNumber MdnsSocketNumber = 6;
const SocketNumber DhcpSocketNumber = 7;

class Platform;

// The main network class that drives the network.
class W5500Interface : public NetworkInterface
{
public:
	W5500Interface(Platform& p) noexcept;

	void Init() noexcept override;
	void Activate() noexcept override;
	void Exit() noexcept override;
	void Spin() noexcept override;
	void Diagnostics(MessageType mtype) noexcept override;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept override;			// enable or disable the network
	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept override;
	bool IsProtocolEnabled(NetworkProtocol protocol) noexcept;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) noexcept override;
	GCodeResult ReportProtocols(const StringRef& reply) const noexcept override;

	GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
	int EnableState() const noexcept override;
	bool IsWiFiInterface() const noexcept override { return false; }

	void UpdateHostname(const char *name) noexcept override;
	IPAddress GetIPAddress() const noexcept override { return ipAddress; }
	void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept override;
	GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept override;
	const MacAddress& GetMacAddress() const noexcept override { return macAddress; }

	void OpenDataPort(Port port) noexcept override;
	void TerminateDataPort() noexcept override;

protected:
	DECLARE_OBJECT_MODEL

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

	void Start() noexcept;
	void Stop() noexcept;
	void InitSockets() noexcept;
	void ResetSockets() noexcept;
	void TerminateSockets() noexcept;

	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
	pre(protocol < NumProtocols);

	Platform& platform;
	uint32_t lastTickMillis;

	W5500Socket *sockets[NumW5500TcpSockets];
	size_t ftpDataSocket;							// number of the port for FTP DATA connections
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	W5500Socket *mdnsSocket;
	MdnsResponder *mdnsResponder;

	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

	NetworkState state;
	bool activated;
	bool usingDhcp;

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	MacAddress macAddress;
};

#endif
