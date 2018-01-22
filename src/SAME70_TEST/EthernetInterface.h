/*
 * EthernetInterface.h
 *
 *  Created on: 24 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_SAME70_ETHERNETINTERFACE_H_
#define SRC_SAME70_ETHERNETINTERFACE_H_

#include "NetworkInterface.h"
#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"

// We have 8 sockets available for Ethernet
const size_t NumHttpSockets = 5;				// sockets 0-4 are for HTTP
const SocketNumber FtpSocketNumber = 5;
const SocketNumber FtpDataSocketNumber = 6;
const SocketNumber TelnetSocketNumber = 7;
const size_t NumEthernetSockets = 8;


class Platform;
class LwipSocket;
struct tcp_pcb;

// The main network class that drives Ethernet network.
class EthernetInterface : public NetworkInterface
{
public:
	EthernetInterface(Platform& p);

	void Init() override;
	void Activate() override;
	void Exit() override;
	void Spin(bool full) override;
	void Interrupt() override;
	void Diagnostics(MessageType mtype) override;
	void Start() override;
	void Stop() override;

	void EnableProtocol(int protocol, int port, int secure, StringRef& reply) override;
	void DisableProtocol(int protocol, StringRef& reply) override;
	void ReportProtocols(StringRef& reply) const override;

	// LwIP interfaces
	void ResetCallback();
	bool Lock();
	void Unlock();
	static bool InLwip();

	bool ConnectionEstablished(tcp_pcb *pcb);

	// Global settings
	void Enable(int mode, StringRef& reply) override;
	bool GetNetworkState(StringRef& reply) override;
	int EnableState() const override;

	const uint8_t *GetIPAddress() const override;
	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]) override;

	void SetHostname(const char *hostname) override;

	void OpenDataPort(Port port) override;
	void TerminateDataPort() override;
	void DataPortClosing() override;

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

	void InitSockets();
	void TerminateSockets();

	void RebuildMdnsServices();

	void StartProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(Protocol protocol, StringRef& reply) const
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
};

#endif
