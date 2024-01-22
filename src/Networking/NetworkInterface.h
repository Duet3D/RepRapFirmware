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
	NetworkInterface(const NetworkInterface&) = delete;

	virtual void Init() noexcept = 0;
	virtual void Activate() noexcept = 0;
	virtual void Exit() noexcept = 0;
	virtual void Spin() noexcept = 0;
	virtual void Diagnostics(MessageType mtype) noexcept = 0;

	virtual GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept = 0;
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

	virtual void UpdateHostname(const char *hostname) noexcept = 0;

	virtual void OpenDataPort(TcpPort port) noexcept = 0;
	virtual void TerminateDataPort() noexcept = 0;

	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, uint32_t ip, int secure, const StringRef& reply) noexcept;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply, bool shutdown = true) noexcept;
	GCodeResult ReportProtocols(const StringRef& reply) const noexcept;

	Mutex interfaceMutex;										// mutex to protect against multiple tasks using the same interface concurrently. Public so that sockets can lock it.

protected:
	// Disable a network protocol that is enabled. If 'permanent' is true we will leave this protocol disables, otherwise we are about to re-enable it with different parameters.
	virtual void IfaceShutdownProtocol(NetworkProtocol protocol, bool permanent) noexcept = 0
		pre(protocol < NumSelectableProtocols; GetState() == NetworkState::active);

	// Enable a network protocol that is currently disabled
	virtual void IfaceStartProtocol(NetworkProtocol protocol) noexcept = 0
		pre(protocol < NumSelectableProtocols; GetState() == NetworkState::active);

	NetworkState::RawType GetState() const noexcept { return state.RawValue(); }
	void SetState(NetworkState::RawType newState) noexcept;
	const char *GetStateName() const noexcept { return state.ToString(); }
	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
		pre(protocol < NumSelectableProtocols);

	uint32_t ipAddresses[NumSelectableProtocols];				// IP address of the corresponding server, used by client protocols only
	TcpPort portNumbers[NumSelectableProtocols];				// port number used for each protocol
	bool protocolEnabled[NumSelectableProtocols];				// whether each protocol is enabled

private:
	NetworkState state;
};

#endif /* SRC_NETWORKING_NETWORKINTERFACE_H_ */
