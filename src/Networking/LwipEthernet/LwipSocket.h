/*
 * LwipSocket.h
 *
 *  Created on: 20 Nov 2017
 *      Authors: David and Christian
 */

#ifndef SRC_SAME70_LWIPSOCKET_H_
#define SRC_SAME70_LWIPSOCKET_H_

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
	LwipSocket(NetworkInterface *iface);
	int GetState() const { return (int)state; }		// for debugging

	// LwIP interfaces
	bool AcceptConnection(tcp_pcb *pcb);
	void DataReceived(pbuf *data);
	void DataSent(size_t numBytes);
	void ConnectionClosedGracefully();
	void ConnectionError(err_t err);

	// Inherited members of the Socket class
	void Init(SocketNumber s, Port serverPort, NetworkProtocol p);
	void TerminateAndDisable() override;
	void Poll(bool full) override;
	void Close() override;
	bool IsClosing() const { return (state == SocketState::closing); }
	void Terminate() override;
	bool ReadChar(char& c) override;
	bool ReadBuffer(const uint8_t *&buffer, size_t &len) override;
	void Taken(size_t len) override;
	bool CanRead() const override;
	bool CanSend() const override;
	size_t Send(const uint8_t *data, size_t length) override;
	void Send() override { }

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

#endif /* SRC_SAME70_LWIPSOCKET_H_ */
