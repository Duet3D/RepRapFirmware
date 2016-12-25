/*
 * Socket.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_SOCKET_H_
#define SRC_DUETNG_DUETETHERNET_SOCKET_H_

#include <NetworkDefs.h>
#include <cstdint>
#include <cstddef>

// Socket structure that we use to track TCP connections
class Socket
{
public:
	Socket();
	void Init(SocketNumber s, Port serverPort);
	void Poll();
	Port GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	Port GetRemotePort() const { return remotePort; }
	bool IsConnected() const;
	bool IsTerminated() const { return isTerminated; }
	void Terminate();
	SocketNumber GetNumber() const { return socketNum; }
	bool ReadChar(char& c);

private:
	enum class SocketState : uint8_t
	{
		inactive,
		idle,
		requestInProgress,
		requestDone,
		responseInProgress,
		responseDone
	};

	void ReInit();

	Port localPort, remotePort;							// The local and remote ports
	uint32_t remoteIPAddress;							// The remote IP address
	NetworkBuffer *receivedData;						// Chain of buffers holding received data
	NetworkTransaction * /*volatile*/ sendingTransaction;	// NetworkTransaction that is currently sending via this connection
	bool persistConnection;								// Do we expect this connection to stay alive?
	bool isTerminated;									// Will be true if the connection has gone down unexpectedly (TCP RST)
	SocketNumber socketNum;								// The W5500 socket number we are using
	SocketState state;
};

#endif /* SRC_DUETNG_DUETETHERNET_SOCKET_H_ */
