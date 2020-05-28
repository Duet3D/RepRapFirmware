/*
 * Socket.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_NETWORKING_W5500SOCKET_H_
#define SRC_NETWORKING_W5500SOCKET_H_

#include "RepRapFirmware.h"
#include "NetworkDefs.h"
#include "Socket.h"

// Socket structure that we use to track TCP connections
class W5500Socket : public Socket
{
public:
	W5500Socket(NetworkInterface *iface) noexcept;
	void Init(SocketNumber s, TcpPort serverPort, NetworkProtocol p) noexcept;

	void Poll() noexcept override;
	void Close() noexcept override;
	void Terminate() noexcept override;
	void TerminateAndDisable() noexcept override;
	bool ReadChar(char& c) noexcept override;
	bool ReadBuffer(const uint8_t *&buffer, size_t &len) noexcept override;
	void Taken(size_t len) noexcept override;
	bool CanRead() const noexcept override;
	bool CanSend() const noexcept override;
	size_t Send(const uint8_t *data, size_t length) noexcept override;
	void Send() noexcept override;

private:
	void ReInit() noexcept;
	void ReceiveData() noexcept;
	void DiscardReceivedData() noexcept;

	NetworkBuffer *receivedData;						// List of buffers holding received data
	uint32_t whenConnected;
	bool persistConnection;								// Do we expect this connection to stay alive?
	bool isTerminated;									// Will be true if the connection has gone down unexpectedly (TCP RST)
	SocketNumber socketNum;								// The W5500 socket number we are using
	bool sendOutstanding;								// True if we have written data to the socket but not flushed it
	bool isSending;										// True if we have written data to the W5500 to send and have not yet seen success or timeout
	uint16_t wizTxBufferPtr;							// Current offset into the Wizchip send buffer, if sendOutstanding is true
	uint16_t wizTxBufferLeft;							// Transmit buffer space left, if sendOutstanding is true
};

#endif /* SRC_NETWORKING_W5500SOCKET_H_ */
