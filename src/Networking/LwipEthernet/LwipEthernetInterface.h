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

extern "C" {
#include "lwip/opt.h"
}

// We have 8 sockets available for Ethernet
const size_t NumHttpSockets = 5;					// sockets 0-4 are for HTTP
#if SAME70
const size_t NumTlsHttpSockets = 4;					// save some RAM by reducing the number of active HTTPS sessions
#else
const size_t NumTlsHttpSockets = 3;
#endif

const SocketNumber FtpSocketNumber = 5;
const SocketNumber FtpDataSocketNumber = 6;
const SocketNumber TelnetSocketNumber = 7;

#if SUPPORT_MQTT
const SocketNumber MqttSocketNumber = 8;
const size_t NumEthernetSockets = 9;
#else
const size_t NumEthernetSockets = 8;
#endif


// Forward declarations
class LwipSocket;
struct altcp_pcb;
struct altcp_tls_config;

// The main network class that drives Ethernet network.
class LwipEthernetInterface : public NetworkInterface
{
public:
	LwipEthernetInterface(Platform& p) noexcept;

	void Init() noexcept override;
	void Activate() noexcept override;
	void Exit() noexcept override;
	void Spin() noexcept override;
	void Diagnostics(const StringRef& reply) noexcept override;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept override;			// enable or disable the network

	GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
	int EnableState() const noexcept override;
	bool IsWiFiInterface() const noexcept override { return false; }

	void UpdateHostname(const char *hostname) noexcept override;

	IPAddress GetIPAddress() const noexcept override { return ipAddress; }
	IPAddress GetNetmask() const noexcept override { return netmask; }
	IPAddress GetGateway() const noexcept override { return gateway; }
	bool UsingDhcp() const noexcept override { return usingDhcp; }
	void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept override;

	GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept override;
	const MacAddress& GetMacAddress() const noexcept override { return macAddress; }

	// LwIP interfaces
	bool ConnectionEstablished(altcp_pcb *pcb) noexcept;

	bool OpenDataPort(TcpPort port, bool useTls = false) noexcept override;
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

	void RebuildMdnsServices() noexcept;

	void StartProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumSelectableProtocols);

#if HAS_CLIENTS
	void ConnectProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumSelectableProtocols);
#endif

	void ShutdownProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumSelectableProtocols);

#if LWIP_ALTCP_TLS
	bool SupportsTls() const noexcept override { return true; }
	uint8_t *ReadPemFile(const char *filename, size_t& len) noexcept;
	bool LoadTlsCertificates() noexcept override;
	void FreeTlsConfig() noexcept;
#endif

	Platform& platform;

	LwipSocket *sockets[NumEthernetSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	bool closeDataPort;
	altcp_pcb *listeningPcbs[NumTcpProtocols];

#if LWIP_ALTCP_TLS
	altcp_pcb *tlsListeningPcbs[NumSelectableProtocols];
	altcp_tls_config *tlsConfig = nullptr;
#endif

	bool activated;
	bool initialised;
	bool usingDhcp = true;

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	MacAddress macAddress;
};

#endif	// HAS_LWIP_NETWORKING

#endif	// SRC_NETWORKING_LWIPETHERNET_LWIPETHERNETINTERFACE_H_
