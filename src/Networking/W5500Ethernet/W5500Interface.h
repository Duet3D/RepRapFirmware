/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by chrishamm

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <Networking/NetworkInterface.h>
#include <Networking/NetworkDefs.h>

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

	GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
	int EnableState() const noexcept override;
	bool IsWiFiInterface() const noexcept override { return false; }

	void UpdateHostname(const char *name) noexcept override;

	IPAddress GetIPAddress() const noexcept override { return ipAddress; }
	IPAddress GetNetmask() const noexcept override { return netmask; }
	IPAddress GetGateway() const noexcept override { return gateway; }
	bool UsingDhcp() const noexcept override { return usingDhcp; }
	void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept override;
	GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept override;
	const MacAddress& GetMacAddress() const noexcept override { return macAddress; }

	void OpenDataPort(TcpPort port) noexcept override;
	void TerminateDataPort() noexcept override;

protected:
	DECLARE_OBJECT_MODEL

	// Disable a network protocol that is enabled. If 'permanent' is true we will leave this protocol disables, otherwise we are about to re-enable it with different parameters.
	void IfaceShutdownProtocol(NetworkProtocol protocol, bool permanent) noexcept override;

	// Enable a network protocol that is currently disabled
	void IfaceStartProtocol(NetworkProtocol protocol) noexcept override;

private:
	void Start() noexcept;
	void Stop() noexcept;
	void InitSockets() noexcept;
	void TerminateSockets() noexcept;

	Platform& platform;
	uint32_t lastTickMillis;

	W5500Socket *sockets[NumW5500TcpSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	W5500Socket *mdnsSocket;
	MdnsResponder *mdnsResponder;

	bool activated;
	bool usingDhcp = true;

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	MacAddress macAddress;
};

#endif
