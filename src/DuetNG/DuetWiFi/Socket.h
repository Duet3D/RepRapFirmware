/*
 * WiFiSocket.h
 *
 *  Created on: 22 Apr 2017
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETWIFI_SOCKET_H_
#define SRC_DUETNG_DUETWIFI_SOCKET_H_

#include "RepRapFirmware.h"
#include "NetworkDefs.h"

class Socket
{
public:
	Socket();
	void Init(SocketNumber n);
	void Poll(bool full);
	Port GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	Port GetRemotePort() const { return remotePort; }
	void Close();
	void Terminate();
	bool ReadChar(char& c);
	bool ReadBuffer(const uint8_t *&buffer, size_t &len);
	void Taken(size_t len);
	bool CanRead() const;
	bool CanSend() const;
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
	void DiscardReceivedData();

	Port localPort, remotePort;							// The local and remote ports
	Protocol protocol;									// What protocol this socket is for
	uint32_t remoteIPAddress;							// The remote IP address
	NetworkBuffer *receivedData;						// List of buffers holding received data
	uint32_t whenConnected;
	bool persistConnection;								// Do we expect this connection to stay alive?
	SocketNumber socketNum;								// The WiFi socket number we are using
	SocketState state;
	bool sendOutstanding;								// True if we have written data to the socket but not flushed it
	bool isSending;										// True if we have written data to send and have not yet seen success or timeout
};

#endif /* SRC_DUETNG_DUETWIFI_SOCKET_H_ */
