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
	virtual void Init() = 0;
	virtual void Activate() = 0;
	virtual void Exit() = 0;
	virtual void Spin(bool full) = 0;
	virtual void Interrupt() { };
	virtual void Diagnostics(MessageType mtype) = 0;

	virtual GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) = 0;
	virtual GCodeResult GetNetworkState(const StringRef& reply) = 0;
	virtual int EnableState() const = 0;
	virtual bool InNetworkStack() const = 0;
	virtual bool IsWiFiInterface() const = 0;

	virtual GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) = 0;
	virtual bool IsProtocolEnabled(NetworkProtocol protocol) { return protocolEnabled[protocol]; }
	virtual Port GetProtocolPort(NetworkProtocol protocol) { return portNumbers[protocol]; }
	virtual GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) = 0;
	virtual GCodeResult ReportProtocols(const StringRef& reply) const = 0;

	virtual IPAddress GetIPAddress() const = 0;
	virtual void SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) = 0;
	virtual void SetMacAddress(const uint8_t mac[]) = 0;
	virtual const uint8_t *GetMacAddress() const = 0;

	virtual void UpdateHostname(const char *hostname) = 0;

	virtual void OpenDataPort(Port port) = 0;
	virtual void TerminateDataPort() = 0;

	Mutex interfaceMutex;							// mutex to protect against multiple tasks using the same interface concurrently. Public so that sockets can lock it.

protected:
	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled
};

#endif /* SRC_NETWORKING_NETWORKINTERFACE_H_ */
