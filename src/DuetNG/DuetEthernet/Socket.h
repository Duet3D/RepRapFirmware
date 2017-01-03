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
	void Init(SocketNumber s, Port serverPort);
	void Poll(bool full);
	Port GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	Port GetRemotePort() const { return remotePort; }
	bool IsConnected() const;
	bool IsTerminated() const { return isTerminated; }
	void Close();
	void Terminate();
	SocketNumber GetNumber() const { return socketNum; }
	NetworkTransaction *GetTransaction() const { return currentTransaction; }
	bool ReadChar(char& c);
	bool ReadBuffer(const char *&buffer, size_t &len);
	bool HasMoreDataToRead() const;
	void ReleaseTransaction();
	bool IsPersistentConnection() const { return persistConnection; }
	bool CanWrite() const;
	void DiscardReceivedData();
	bool AcquireTransaction();

private:
	enum class SocketState : uint8_t
	{
		inactive,
		listening,
		connected,
		clientDisconnecting,
		closing
	};

	void ReInit();
	bool IsSending() const;								// Return true if we are in the sending phase

	bool TrySendData()									// Try to send data, returning true if all data has been sent and we ought to close the socket
	pre(IsSending());

	Port localPort, remotePort;							// The local and remote ports
	uint32_t remoteIPAddress;							// The remote IP address
	NetworkTransaction *currentTransaction;				// The transaction currently being processed on this socket
	NetworkBuffer *receivedData;						// List of buffers holding received data
	bool persistConnection;								// Do we expect this connection to stay alive?
	bool isTerminated;									// Will be true if the connection has gone down unexpectedly (TCP RST)
	SocketNumber socketNum;								// The W5500 socket number we are using
	SocketState state;
	bool isSending;										// True if we have written data to the W5500 to send and have not yet seen success or timeout
	bool needTransaction;								// True if the web server has asked for a transaction
};

#endif /* SRC_DUETNG_DUETETHERNET_SOCKET_H_ */
