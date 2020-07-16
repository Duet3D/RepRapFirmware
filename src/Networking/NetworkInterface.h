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
	NetworkInterface() : state(NetworkState::disabled) { }
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

	virtual GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept = 0;
	virtual bool IsProtocolEnabled(NetworkProtocol protocol) noexcept { return protocolEnabled[protocol]; }
	virtual TcpPort GetProtocolPort(NetworkProtocol protocol) noexcept { return portNumbers[protocol]; }
	virtual GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) noexcept = 0;
	virtual GCodeResult ReportProtocols(const StringRef& reply) const noexcept = 0;

	virtual IPAddress GetIPAddress() const noexcept = 0;
	virtual void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept = 0;
	virtual GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept = 0;
	virtual const MacAddress& GetMacAddress() const noexcept = 0;

	virtual void UpdateHostname(const char *hostname) noexcept = 0;

	virtual void OpenDataPort(TcpPort port) noexcept = 0;
	virtual void TerminateDataPort() noexcept = 0;

	Mutex interfaceMutex;							// mutex to protect against multiple tasks using the same interface concurrently. Public so that sockets can lock it.

protected:
	NetworkState::RawType GetState() const noexcept { return state.RawValue(); }
	void SetState(NetworkState::RawType newState) noexcept;
	const char *GetStateName() const noexcept { return state.ToString(); }

	TcpPort portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

private:
	NetworkState state;
};

#endif /* SRC_NETWORKING_NETWORKINTERFACE_H_ */
