/*
 * WiFiSocket.h
 *
 *  Created on: 22 Apr 2017
 *      Author: David
 */

#ifndef SRC_NETWORKING_WIFISOCKET_H_
#define SRC_NETWORKING_WIFISOCKET_H_

#include "RepRapFirmware.h"
#include "Networking/NetworkDefs.h"
#include "Networking/Socket.h"


class WiFiInterface;

class WiFiSocket : public Socket
{
public:
	WiFiSocket(NetworkInterface *iface);
	void Init(SocketNumber n);
	int State() const { return (int)state; }				// used only for reporting debug info, hence the 'int' return
	void Poll(bool full);
	void Close();
	bool IsClosing() const { return (state == SocketState::closing); }
	void Terminate();
	void TerminateAndDisable() { Terminate(); }
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

	WiFiInterface *GetInterface() const;
	void ReceiveData(uint16_t bytesAvailable);
	void DiscardReceivedData();

	NetworkBuffer *receivedData;						// List of buffers holding received data
	uint32_t whenConnected;
	uint16_t txBufferSpace;								// How much free transmit buffer space the WiFi mofule reported
	SocketNumber socketNum;								// The WiFi socket number we are using
	SocketState state;
	bool needsPolling;
};

#endif /* SRC_NETWORKING_WIFISOCKET_H_ */
