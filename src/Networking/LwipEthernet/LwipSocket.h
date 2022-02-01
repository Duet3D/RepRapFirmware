/*
 * LwipSocket.h
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_NETWORKING_LWIPETHERNET_LWIPSOCKET_H_
#define SRC_NETWORKING_LWIPETHERNET_LWIPSOCKET_H_

#include <RepRapFirmware.h>

#if HAS_LWIP_NETWORKING

#include "LwipEthernetInterface.h"
#include "Networking/NetworkDefs.h"
#include "Networking/Socket.h"

typedef int8_t err_t;
struct tcp_pcb;
struct pbuf;

// Socket structure for LwIP that we use to track TCP connections
class LwipSocket : public Socket
{
public:
	LwipSocket(NetworkInterface *iface) noexcept;
	int GetState() const noexcept { return (int)state; }		// for debugging

	// LwIP interfaces
	bool AcceptConnection(tcp_pcb *pcb) noexcept;
	void DataReceived(pbuf *data) noexcept;
	void DataSent(size_t numBytes) noexcept;
	void ConnectionClosedGracefully() noexcept;
	void ConnectionError(err_t err) noexcept;

	// Inherited members of the Socket class
	void Init(SocketNumber s, TcpPort serverPort, NetworkProtocol p) noexcept;
	void TerminateAndDisable() noexcept override;
	void Poll() noexcept override;
	void Close() noexcept override;
	bool IsClosing() const noexcept { return (state == SocketState::closing); }
	void Terminate() noexcept override;
	bool ReadChar(char& c) noexcept override;
	bool ReadBuffer(const uint8_t *&buffer, size_t &len) noexcept override;
	void Taken(size_t len) noexcept override;
	bool CanRead() const noexcept override;
	bool CanSend() const noexcept override;
	size_t Send(const uint8_t *data, size_t length) noexcept override;
	void Send() noexcept override { }

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

	void ReInit() noexcept;
	void DiscardReceivedData() noexcept;

	uint32_t whenConnected;
	uint32_t whenWritten;
	uint32_t whenClosed;
	bool responderFound;

	tcp_pcb *connectionPcb;
	pbuf *receivedData;
	size_t readIndex;

	SocketState state;
	size_t unAcked;
};

#endif	// HAS_LWIP_NETWORKING

#endif /* SRC_NETWORKING_LWIPETHERNET_LWIPSOCKET_H_ */
