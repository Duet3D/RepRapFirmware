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
class NetworkInterface
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
	virtual GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) = 0;
	virtual GCodeResult ReportProtocols(const StringRef& reply) const = 0;

	virtual const uint8_t *GetIPAddress() const = 0;
	virtual void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]) = 0;
	virtual void SetMacAddress(const uint8_t mac[]) = 0;
	virtual const uint8_t *GetMacAddress() const = 0;

	virtual void UpdateHostname(const char *hostname) = 0;

	virtual void OpenDataPort(Port port) = 0;
	virtual void TerminateDataPort() = 0;

protected:
	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled
};

#endif /* SRC_NETWORKING_NETWORKINTERFACE_H_ */
