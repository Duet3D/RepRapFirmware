/*
 * Socket.h
 *
 *  Created on: 21 Nov 2017
 *      Author: Christian
 */

#ifndef SRC_NETWORKING_SOCKET_H_
#define SRC_NETWORKING_SOCKET_H_

#include "NetworkDefs.h"
#include "General/IPAddress.h"

const uint32_t FindResponderTimeout = 2000;		// how long we wait for a responder to become available
const uint32_t ConnectTimeout = 2000;			// how long we wait for an outgoing connection attempt
const uint32_t MaxAckTime = 4000;				// how long we wait for a connection to acknowledge the remaining data before it is closed
const uint32_t MaxWriteTime = 2000;				// how long we wait for a write operation to complete before it is cancelled


class NetworkInterface;

// Abstract socket structure that we use to track TCP connections
class Socket
{
public:
	explicit Socket(NetworkInterface *_ecv_from iface) noexcept : interface(iface), localPort(0), remotePort(0), remoteIPAddress() { }
	Socket(const Socket &_ecv_from) = delete;

	NetworkInterface *_ecv_from GetInterface() const noexcept { return interface; }

	TcpPort GetLocalPort() const noexcept { return localPort; }
	IPAddress GetRemoteIP() const noexcept { return remoteIPAddress; }
	TcpPort GetRemotePort() const noexcept { return remotePort; }
	NetworkProtocol GetProtocol() const noexcept { return protocol; }

	virtual void Poll() noexcept = 0;
	virtual void Close() noexcept = 0;
	virtual void Terminate() noexcept = 0;
	virtual void TerminateAndDisable() noexcept = 0;
	virtual bool ReadChar(char& c) noexcept = 0;
	virtual bool ReadBuffer(const uint8_t *_ecv_array &buffer, size_t &len) noexcept = 0;
	virtual void Taken(size_t len) noexcept = 0;
	virtual bool CanRead() const noexcept = 0;
	virtual bool CanSend() const noexcept = 0;
	virtual size_t Send(const uint8_t *_ecv_array data, size_t length) noexcept = 0;
	virtual void Send() noexcept = 0;

protected:
	NetworkInterface *_ecv_from const interface;
	TcpPort localPort, remotePort;						// The local and remote ports
	NetworkProtocol protocol;							// What protocol this socket is for
	IPAddress remoteIPAddress;							// The remote IP address
};

#endif /* SRC_NETWORKING_SOCKET_H_ */
