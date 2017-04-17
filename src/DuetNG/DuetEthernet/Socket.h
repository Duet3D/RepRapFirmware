/*
 * Socket.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_SOCKET_H_
#define SRC_DUETNG_DUETETHERNET_SOCKET_H_

#include "RepRapFirmware.h"
#include "NetworkDefs.h"

// Socket structure that we use to track TCP connections
class Socket
{
public:
	Socket();
	void Init(SocketNumber s, Port serverPort, Protocol p);
	void TerminateAndDisable();
	void Poll(bool full);
	Port GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	Port GetRemotePort() const { return remotePort; }
	bool IsConnected() const;
	bool IsTerminated() const { return isTerminated; }
	void Close();
	void Terminate();
	SocketNumber GetNumber() const { return socketNum; }
	bool ReadChar(char& c);
	bool ReadBuffer(const char *&buffer, size_t &len);
	bool CanRead() const;
	bool IsPersistentConnection() const { return persistConnection; }
	bool CanSend() const;
	void DiscardReceivedData();
	size_t Send(const uint8_t *data, size_t length);
	void Send();

private:
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

	void ReInit();

	Port localPort, remotePort;							// The local and remote ports
	Protocol protocol;									// What protocol this socket is for
	uint32_t remoteIPAddress;							// The remote IP address
	NetworkBuffer *receivedData;						// List of buffers holding received data
	uint32_t whenConnected;
	bool persistConnection;								// Do we expect this connection to stay alive?
	bool isTerminated;									// Will be true if the connection has gone down unexpectedly (TCP RST)
	SocketNumber socketNum;								// The W5500 socket number we are using
	SocketState state;
	bool sendOutstanding;								// True if we have written data to the socket but not flushed it
	bool isSending;										// True if we have written data to the W5500 to send and have not yet seen success or timeout
	uint16_t wizTxBufferPtr;							// Current offset into the Wizchip send buffer, if sendOutstanding is true
	uint16_t wizTxBufferLeft;							// Transmit buffer space left, if sendOutstanding is true
};

#endif /* SRC_DUETNG_DUETETHERNET_SOCKET_H_ */
