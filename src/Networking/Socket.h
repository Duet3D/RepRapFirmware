/*
 * Socket.h
 *
 *  Created on: 21 Nov 2017
 *      Author: Christian
 */

#ifndef SRC_NETWORKING_SOCKET_H_
#define SRC_NETWORKING_SOCKET_H_

#include "NetworkDefs.h"

const uint32_t FindResponderTimeout = 2000;		// how long we wait for a responder to become available
const uint32_t MaxAckTime = 4000;				// how long we wait for a connection to acknowledge the remaining data before it is closed
const uint32_t MaxWriteTime = 2000;				// how long we wait for a write operation to complete before it is cancelled


class NetworkInterface;

// Abstract socket structure that we use to track TCP connections
class Socket
{
public:
	Socket(NetworkInterface *iface) : interface(iface), localPort(0), remotePort(0), remoteIPAddress(0), state(SocketState::disabled) { }
	NetworkInterface *GetInterface() const { return interface; }

	Port GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	Port GetRemotePort() const { return remotePort; }
	NetworkProtocol GetProtocol() const { return protocol; }

	virtual void Poll(bool full) = 0;
	virtual void Close() = 0;
	virtual void Terminate() = 0;
	virtual void TerminateAndDisable() = 0;
	virtual bool ReadChar(char& c) = 0;
	virtual bool ReadBuffer(const uint8_t *&buffer, size_t &len) = 0;
	virtual void Taken(size_t len) = 0;
	virtual bool CanRead() const = 0;
	virtual bool CanSend() const = 0;
	virtual size_t Send(const uint8_t *data, size_t length) = 0;
	virtual void Send() = 0;

protected:
	enum class SocketState : uint8_t
	{
		disabled,
		inactive,
		listening,
		connected,
		clientDisconnecting,
		closing,
		aborted
	};

	NetworkInterface * const interface;
	Port localPort, remotePort;							// The local and remote ports
	NetworkProtocol protocol;							// What protocol this socket is for
	uint32_t remoteIPAddress;							// The remote IP address
	SocketState state;
};

#endif /* SRC_NETWORKING_SOCKET_H_ */
