/*
 * EthernetInterface.h
 *
 *  Created on: 24 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_NETWORKING_LWIPETHERNET_LWIPETHERNETINTERFACE_H_
#define SRC_NETWORKING_LWIPETHERNET_LWIPETHERNETINTERFACE_H_

#include <RepRapFirmware.h>

#if HAS_LWIP_NETWORKING

#include <Networking/NetworkInterface.h>
#include <Networking/NetworkDefs.h>

// We have 8 sockets available for Ethernet
const size_t NumHttpSockets = 5;				// sockets 0-4 are for HTTP
const SocketNumber FtpSocketNumber = 5;
const SocketNumber FtpDataSocketNumber = 6;
const SocketNumber TelnetSocketNumber = 7;
const size_t NumEthernetSockets = 8;

// Forward declarations
class LwipSocket;
struct tcp_pcb;

// The main network class that drives Ethernet network.
class LwipEthernetInterface : public NetworkInterface
{
public:
	LwipEthernetInterface(Platform& p) noexcept;

	void Init() noexcept override;
	void Activate() noexcept override;
	void Exit() noexcept override;
	void Spin() noexcept override;
	void Diagnostics(MessageType mtype) noexcept override;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept override;			// enable or disable the network
	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept override;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) noexcept override;
	GCodeResult ReportProtocols(const StringRef& reply) const noexcept override;

	GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
	int EnableState() const noexcept override;
	bool IsWiFiInterface() const noexcept override { return false; }

	void UpdateHostname(const char *hostname) noexcept override;
	IPAddress GetIPAddress() const noexcept override;
	void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept override;
	GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept override;
	const MacAddress& GetMacAddress() const noexcept override { return macAddress; }

	// LwIP interfaces
	bool ConnectionEstablished(tcp_pcb *pcb) noexcept;

	void OpenDataPort(TcpPort port) noexcept override;
	void TerminateDataPort() noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	void Start() noexcept;
	void Stop() noexcept;
	void InitSockets() noexcept;
	void TerminateSockets() noexcept;

	void RebuildMdnsServices() noexcept;

	void StartProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumProtocols);

	void ShutdownProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumProtocols);

	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
	pre(protocol < NumProtocols);

	Platform& platform;

	LwipSocket *sockets[NumEthernetSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	TcpPort portNumbers[NumProtocols];				// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled
	bool closeDataPort;
	tcp_pcb *listeningPcbs[NumTcpPorts];

	bool activated;
	bool initialised;
	bool usingDhcp;

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	MacAddress macAddress;
};

#endif	// HAS_LWIP_NETWORKING

#endif	// SRC_NETWORKING_LWIPETHERNET_LWIPETHERNETINTERFACE_H_
