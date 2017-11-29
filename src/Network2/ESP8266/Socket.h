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
	int State() const { return (int)state; }				// used only for reporting debug info, hence the 'int' return
	void Poll(bool full);
	Port GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIp; }
	Port GetRemotePort() const { return remotePort; }
	void Close();
	void Terminate();
	void TerminatePolitely();
	bool ReadChar(char& c);
	bool ReadBuffer(const uint8_t *&buffer, size_t &len);
	void Taken(size_t len);
	bool CanRead() const;
	bool CanSend() const;
	size_t Send(const uint8_t *data, size_t length);
	void Send();
	void SetNeedsPolling() { needsPolling = true; }
	bool NeedsPolling() const;

private:
	enum class SocketState : uint8_t
	{
		inactive,
		waitingForResponder,
		connected,
		clientDisconnecting,
		closing,
		broken
	};

	void ReInit();
	void ReceiveData(uint16_t bytesAvailable);
	void DiscardReceivedData();

	Port localPort, remotePort;							// The local and remote ports
	Protocol protocol;									// What protocol this socket is for
	uint32_t remoteIp;									// The remote IP address
	NetworkBuffer *receivedData;						// List of buffers holding received data
	uint32_t whenConnected;
	uint16_t txBufferSpace;								// How much free transmit buffer space the WiFi mofule reported
	SocketNumber socketNum;								// The WiFi socket number we are using
	SocketState state;
	bool needsPolling;
};

#endif /* SRC_DUETNG_DUETWIFI_SOCKET_H_ */
