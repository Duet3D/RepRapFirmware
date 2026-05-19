/*
 * NetworkInterface.h
 *
 *  Created on: 24 Nov 2017
 *      Author: Christian
 */

#ifndef SRC_NETWORKING_NETWORKINTERFACE_H_
#define SRC_NETWORKING_NETWORKINTERFACE_H_

#include "Network.h"

// Abstract base class for network modules
class NetworkInterface INHERIT_OBJECT_MODEL
{
public:
	NetworkInterface() noexcept;
	NetworkInterface(const NetworkInterface &_ecv_from) = delete;

	virtual void Init() noexcept = 0;
	virtual void Activate() noexcept = 0;
	virtual void Exit() noexcept = 0;
	virtual void Spin() noexcept = 0;
	virtual void Diagnostics(const StringRef& reply) noexcept = 0;

	// tlsParam:  1 = enable TLS support (probe + auto-import on WiFi; load /sys/server.{crt,key} on LwIP)
	//            0 = plain mode, no TLS
	//           -1 = clear any stored TLS material first, then come up plain (M552 T-1 S1)
	virtual GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply, int tlsParam = 1) noexcept = 0;
	virtual GCodeResult GetNetworkState(const StringRef& reply) noexcept = 0;
	virtual int EnableState() const noexcept = 0;
	virtual bool IsWiFiInterface() const noexcept = 0;

	virtual IPAddress GetIPAddress() const noexcept = 0;
	virtual IPAddress GetNetmask() const noexcept = 0;
	virtual IPAddress GetGateway() const noexcept = 0;
	virtual bool UsingDhcp() const noexcept = 0;
	virtual void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept = 0;
	virtual GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept = 0;
	virtual const MacAddress& GetMacAddress() const noexcept = 0;

	virtual void UpdateHostname(const char *_ecv_array hostname) noexcept = 0;

	virtual bool OpenDataPort(TcpPort port, bool useTls = false) noexcept = 0;
	virtual void TerminateDataPort() noexcept = 0;

	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply, bool shutdown = true) noexcept;
	GCodeResult ReportProtocols(const StringRef& reply) const noexcept;

	// Apply any M586 T1 calls that were deferred because SupportsTls() was false at the time.
	// Called by the WiFi interface once an asynchronous TryEnableTls completes, so that a typical
	// config.g sequence "M552 T1 / M586 T1" still binds the TLS listener even though the TLS probe
	// runs after M552 returns. No-op when SupportsTls() is still false
	void ApplyPendingTlsProtocols() noexcept;

	TcpPort GetTlsPortNumber(NetworkProtocol p) const noexcept pre(p < NumSelectableProtocols) { return tlsPortNumbers[p]; }

	Mutex interfaceMutex;										// mutex to protect against multiple tasks using the same interface concurrently. Public so that sockets can lock it.

protected:
	virtual bool SupportsTls() const noexcept { return false; }
	virtual bool LoadTlsCertificates(const StringRef& reply) noexcept { return false; }

	// Disable a network protocol that is enabled. If 'permanent' is true we will leave this protocol disables, otherwise we are about to re-enable it with different parameters.
	virtual void IfaceShutdownProtocol(NetworkProtocol protocol, bool permanent) noexcept
		pre(protocol < NumSelectableProtocols; GetState() == NetworkState::active)
		 = 0;

	// Enable a network protocol that is currently disabled
	virtual void IfaceStartProtocol(NetworkProtocol protocol) noexcept
		pre(protocol < NumSelectableProtocols; GetState() == NetworkState::active)
		 = 0;

	NetworkState::RawType GetState() const noexcept { return state.RawValue(); }
	void SetState(NetworkState::RawType newState) noexcept;
	const char *_ecv_array GetStateName() const noexcept { return state.ToString(); }
	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
		pre(protocol < NumSelectableProtocols);

	uint32_t ipAddresses[NumSelectableProtocols];				// IP address of the corresponding server, used by client protocols only
	TcpPort portNumbers[NumSelectableProtocols];				// port number used for each protocol
	TcpPort tlsPortNumbers[NumSelectableProtocols];				// TLS port number for each protocol
	bool protocolEnabled[NumSelectableProtocols];				// whether the plain protocol is enabled
	bool tlsProtocolEnabled[NumSelectableProtocols];			// whether the TLS variant of the protocol is enabled
	bool tlsProtocolPending[NumSelectableProtocols];			// M586 T1 deferred because SupportsTls() was false; applied later by ApplyPendingTlsProtocols()

private:
	NetworkState state;
};

#endif /* SRC_NETWORKING_NETWORKINTERFACE_H_ */
