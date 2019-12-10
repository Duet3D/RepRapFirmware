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
	WiFiSocket(NetworkInterface *iface) noexcept;
	void Init(SocketNumber n) noexcept;
	int State() const noexcept { return (int)state; }				// used only for reporting debug info, hence the 'int' return
	void Poll() noexcept;
	void Close() noexcept;
	bool IsClosing() const noexcept { return (state == SocketState::closing); }
	void Terminate() noexcept;
	void TerminateAndDisable() noexcept { Terminate(); }
	bool ReadChar(char& c) noexcept;
	bool ReadBuffer(const uint8_t *&buffer, size_t &len) noexcept;
	void Taken(size_t len) noexcept;
	bool CanRead() const noexcept;
	bool CanSend() const noexcept;
	size_t Send(const uint8_t *data, size_t length) noexcept;
	void Send() noexcept;
	void SetNeedsPolling() noexcept { needsPolling = true; }
	bool NeedsPolling() const noexcept;

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

	WiFiInterface *GetInterface() const noexcept;
	void ReceiveData(uint16_t bytesAvailable) noexcept;
	void DiscardReceivedData() noexcept;

	NetworkBuffer *receivedData;						// List of buffers holding received data
	uint32_t whenConnected;
	uint16_t txBufferSpace;								// How much free transmit buffer space the WiFi mofule reported
	SocketNumber socketNum;								// The WiFi socket number we are using
	SocketState state;
	bool needsPolling;
};

#endif /* SRC_NETWORKING_WIFISOCKET_H_ */
