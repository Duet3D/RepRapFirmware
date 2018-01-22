/*
 * NetworkInterface.h
 *
 *  Created on: 24 Nov 2017
 *      Author: Christian
 */

#ifndef SRC_SAME70_NETWORKINTERFACE_H_
#define SRC_SAME70_NETWORKINTERFACE_H_

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
	virtual void Start() = 0;
	virtual void Stop() = 0;

	virtual void Enable(int mode, StringRef& reply) { };
	virtual void Enable(int mode, const StringRef& ssid, StringRef& reply) { };
	virtual bool GetNetworkState(StringRef& reply) = 0;
	virtual int EnableState() const = 0;

	virtual void EnableProtocol(int protocol, int port, int secure, StringRef& reply) = 0;
	virtual void DisableProtocol(int protocol, StringRef& reply) = 0;
	virtual void ReportProtocols(StringRef& reply) const = 0;

	virtual const uint8_t *GetIPAddress() const = 0;
	virtual void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]) = 0;

	virtual void SetHostname(const char *hostname) = 0;

	virtual void OpenDataPort(Port port) = 0;
	virtual void TerminateDataPort() = 0;
	virtual void DataPortClosing() = 0;

protected:
	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

};



#endif /* SRC_SAME70_NETWORKINTERFACE_H_ */
