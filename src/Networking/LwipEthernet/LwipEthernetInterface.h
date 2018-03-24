/*
 * EthernetInterface.h
 *
 *  Created on: 24 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_SAME70_LWIPETHERNETINTERFACE_H_
#define SRC_SAME70_LWIPETHERNETINTERFACE_H_

#include "Networking/NetworkInterface.h"
#include "Networking/NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"

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
	LwipEthernetInterface(Platform& p);

	void Init() override;
	void Activate() override;
	void Exit() override;
	void Spin(bool full) override;
	void Interrupt() override;
	void Diagnostics(MessageType mtype) override;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) override;			// enable or disable the network
	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) override;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) override;
	GCodeResult ReportProtocols(const StringRef& reply) const override;

	GCodeResult GetNetworkState(const StringRef& reply) override;
	int EnableState() const override;
	bool InNetworkStack() const override;
	bool IsWiFiInterface() const override { return false; }

	void UpdateHostname(const char *hostname) override;
	const uint8_t *GetIPAddress() const override;
	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]) override;
	void SetMacAddress(const uint8_t mac[]) override;
	const uint8_t *GetMacAddress() const override { return macAddress; }

	// LwIP interfaces
	bool ConnectionEstablished(tcp_pcb *pcb);

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

	void RebuildMdnsServices();

	void StartProtocol(NetworkProtocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(NetworkProtocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const
	pre(protocol < NumProtocols);

	Platform& platform;

	LwipSocket *sockets[NumEthernetSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled
	bool closeDataPort;
	tcp_pcb *listeningPcbs[NumTcpPorts];

	NetworkState state;
	bool activated;
	bool initialised;
	bool usingDhcp;

	uint8_t macAddress[6];
};

#endif
