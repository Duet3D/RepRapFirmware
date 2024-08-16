/*
 * LwipSocket.cpp
 *
 *  Created on: 20 Nov 2017
 *      Author: Christian
 */

// Define this to keep the ASF status codes from being included. Without it ERR_TIMEOUT is defined twice
#define NO_STATUS_CODES

#include "LwipSocket.h"

#if HAS_LWIP_NETWORKING

#include <Networking/NetworkBuffer.h>
#include <Platform/RepRap.h>

extern Mutex lwipMutex;

// ERR_IS_FATAL was defined like this in lwip 2.0.3 file err.h but isn't in 2.1.2
#define ERR_IS_FATAL(e) ((e) <= ERR_ABRT)

#ifndef UNUSED
# define UNUSED(v)		(void)(v)
#endif

//***************************************************************************************************

extern "C" {
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

static void conn_err(void *arg, err_t err)
{
	LwipSocket *socket = (LwipSocket *)arg;
	if (socket != nullptr)
	{
		socket->ConnectionError(err);
	}
}

static err_t conn_recv(void *arg, tcp_pcb *pcb, pbuf *p, err_t err)
{
	UNUSED(err);

	LwipSocket *socket = (LwipSocket *)arg;
	if (socket != nullptr)
	{
		if (p != nullptr)
		{
			socket->DataReceived(p);
		}
		else
		{
			socket->ConnectionClosedGracefully();
		}
		return ERR_OK;
	}

	tcp_abort(pcb);
	return ERR_ABRT;
}

static err_t conn_sent(void *arg, tcp_pcb *pcb, u16_t len)
{
	UNUSED(pcb);

	LwipSocket *socket = (LwipSocket *)arg;
	if (socket != nullptr)
	{
		socket->DataSent(len);
	}

	return ERR_OK;
}

}	// end extern "C"

//***************************************************************************************************

// LwipSocket class

LwipSocket::LwipSocket(NetworkInterface *iface) noexcept : Socket(iface), connectionPcb(nullptr),
		receivedData(nullptr), state(SocketState::disabled)
{
	ReInit();
}

bool LwipSocket::AcceptConnection(tcp_pcb *pcb) noexcept
{
	if ((state == SocketState::listening && pcb->local_port == localPort) ||
		 (state == SocketState::connecting && pcb->remote_port == remotePort))
	{
		ReInit();
		state = SocketState::connected;
		whenConnected = millis();

		connectionPcb = pcb;
		remoteIPAddress.SetV4LittleEndian(pcb->remote_ip.addr);

		if (outgoing)
		{
			localPort = pcb->local_port;
		}
		else
		{
			remotePort = pcb->remote_port;
		}

		tcp_arg(pcb, this);
		tcp_err(pcb, conn_err);
		tcp_recv(pcb, conn_recv);
		tcp_sent(pcb, conn_sent);
		return true;
	}
	return false;
}

void LwipSocket::DataReceived(pbuf *data) noexcept
{
	if (state != SocketState::closing)
	{
		// Store it for the NetworkResponder
		pbuf *const rdata = receivedData;
		if (rdata == nullptr)
		{
			receivedData = data;
		}
		else
		{
			pbuf_cat(rdata, data);
		}
	}
	else
	{
		// Don't process any more data if the connection is going down
		pbuf_free(data);
	}
}

void LwipSocket::DataSent(size_t numBytes) noexcept
{
	if (numBytes <= unAcked)
	{
		unAcked -= numBytes;
	}
	else
	{
		// Should never happen
		unAcked = 0;
	}

	if (unAcked == 0)
	{
		// Reset the write timer when all data has been ACKed
		whenWritten = 0;
	}
}

void LwipSocket::ConnectionClosedGracefully() noexcept
{
	if (connectionPcb != nullptr)
	{
		tcp_err(connectionPcb, nullptr);
		tcp_recv(connectionPcb, nullptr);
		tcp_sent(connectionPcb, nullptr);
		tcp_close(connectionPcb);
		connectionPcb = nullptr;
	}

	if (state == SocketState::closing && !outgoing)
	{
		state = SocketState::listening;
	}
	else
	{
		state = SocketState::peerDisconnecting;
		whenClosed = millis();
	}
}

void LwipSocket::ConnectionError(err_t err) noexcept
{
	DiscardReceivedData();
	connectionPcb = nullptr;

	state = (localPort == 0 || outgoing)
				? SocketState::disabled
				: (responderFound && state != SocketState::closing)
				  	? SocketState::aborted
				  	: SocketState::listening;
}

// Initialise a TCP socket
void LwipSocket::Init(SocketNumber skt, TcpPort serverPort, NetworkProtocol p, bool outgoing) noexcept
{
	UNUSED(skt);
	this->outgoing = outgoing;

	if (this->outgoing)
	{
		remotePort = serverPort;
		whenConnecting = millis();
		state = SocketState::connecting;
	}
	else
	{
		localPort = serverPort;
		state = SocketState::listening;
	}
	protocol = p;
	ReInit();
}

void LwipSocket::TerminateAndDisable() noexcept
{
	Terminate();
	state = SocketState::disabled;
}

void LwipSocket::ReInit() noexcept
{
	DiscardReceivedData();
	whenConnected = whenWritten = whenClosed = 0;
	responderFound = false;
	readIndex = unAcked = 0;
}

// Close a connection when the last packet has been sent
void LwipSocket::Close() noexcept
{
	if (state != SocketState::disabled && state != SocketState::listening)
	{
		MutexLocker lock(lwipMutex);
		DiscardReceivedData();
		state = SocketState::closing;
		whenClosed = millis();

		if (protocol == FtpDataProtocol)
		{
			localPort = 0;					// don't re-listen automatically
		}
	}
}

// Terminate a connection immediately
void LwipSocket::Terminate() noexcept
{
	if (state != SocketState::disabled)
	{
		MutexLocker lock(lwipMutex);
		if (connectionPcb != nullptr)
		{
			tcp_err(connectionPcb, nullptr);
			tcp_recv(connectionPcb, nullptr);
			tcp_sent(connectionPcb, nullptr);
			tcp_abort(connectionPcb);
			connectionPcb = nullptr;
		}

		DiscardReceivedData();
		whenClosed = millis();
		state = (localPort == 0 || outgoing) ? SocketState::disabled : SocketState::listening;
	}
}

// Return true if there is or may soon be more data to read
bool LwipSocket::CanRead() const noexcept
{
	return (state == SocketState::connected)
		|| (state == SocketState::peerDisconnecting && receivedData != nullptr);
}

bool LwipSocket::CanSend() const noexcept
{
	return (state == SocketState::connected);
}

// Get the next received pbuf, skipping any empty ones (we can get empty ones from lwip)
pbuf *LwipSocket::GetNextReceivedPbuf() noexcept
{
	pbuf *rdata;
	while ((rdata = receivedData) != nullptr && rdata->len == 0)
	{
		debugPrintf("Discarding empty pbuf\n");
		MutexLocker lock(lwipMutex);
		receivedData = rdata->next;
		rdata->next = nullptr;
		pbuf_free(rdata);
		readIndex = 0;
	}
	return rdata;
}

// Read 1 character from the receive buffers, returning true if successful
bool LwipSocket::ReadChar(char& c) noexcept
{
	pbuf *const rdata = GetNextReceivedPbuf();
	if (rdata != nullptr)
	{
		const char * const data = (const char *)rdata->payload;
		c = data[readIndex++];

		const uint16_t rlen = rdata->len;
		if (readIndex >= rlen)
		{
			// Free the buffer. Grab the mutex first to prevent lwip appending more data to it.
			MutexLocker lock(lwipMutex);

			receivedData = rdata->next;
			rdata->next = nullptr;
			pbuf_free(rdata);
			readIndex = 0;

			// Tell lwip we have taken this data
			if (connectionPcb != nullptr)
			{
				tcp_recved(connectionPcb, rlen);
			}
		}

		return true;
	}

	c = 0;
	return false;
}

// Return a pointer to data in a buffer and a length available
bool LwipSocket::ReadBuffer(const uint8_t *&buffer, size_t &len) noexcept
{
	pbuf *const rdata = GetNextReceivedPbuf();
	if (rdata != nullptr)
	{
		const uint8_t * const data = (const uint8_t *)rdata->payload;
		buffer = &data[readIndex];
		len = rdata->len - readIndex;
		return true;
	}

	return false;
}

// Flag some data as taken from the receive buffers. We never take data from more than one buffer at a time.
void LwipSocket::Taken(size_t len) noexcept
{
	pbuf *const rdata = receivedData;
	if (rdata != nullptr)			// should always be true
	{
		readIndex += len;
		const uint16_t rlen = rdata->len;
		if (readIndex >= rlen)
		{
			// Free the buffer. Grab the mutex first to prevent lwip appending more data to it.
			MutexLocker lock(lwipMutex);

			// Free the first item of the pbuf chain if the number of taken bytes exceeds its size
			receivedData = rdata->next;
			rdata->next = nullptr;
			pbuf_free(rdata);
			readIndex = 0;

			// Notify LwIP
			if (connectionPcb != nullptr)
			{
				tcp_recved(connectionPcb, rlen);
			}
		}
	}
}

// Poll a socket to see if it needs to be serviced
void LwipSocket::Poll() noexcept
{
	// Deal with transfers that went so quickly that we haven't got a responder yet
	bool wasShortTransfer = !responderFound && (state == SocketState::peerDisconnecting);
	if (wasShortTransfer)
	{
		state = SocketState::connected;
	}

	switch (state)
	{
	case SocketState::connecting:
		// Check for connection attempt timeout
		if (millis() - whenConnecting >= ConnectTimeout)
		{
			Terminate();
		}
		break;

	case SocketState::listening:
		// Socket is listening but no client has connected to it yet
		break;

	case SocketState::connected:
		if (responderFound)
		{
			// Are we still waiting for data to be written?
			if (whenWritten != 0 && millis() - whenWritten >= MaxWriteTime)
			{
				Terminate();
			}
		}
		else
		{
			// Try to find a responder to deal with this connection
			if (reprap.GetNetwork().FindResponder(this, protocol))
			{
				responderFound = true;
			}
			else if (millis() - whenConnected >= FindResponderTimeout)
			{
				Terminate();
			}
		}
		break;

	case SocketState::peerDisconnecting:
	case SocketState::closing:
	{
		// The connection is being closed, but we may be waiting for sent data to be ACKed
		// or for the received data to be processed by a NetworkResponder
		bool timeoutExceeded = millis() - whenClosed > MaxAckTime;
		if (unAcked == 0 || timeoutExceeded)
		{
			if (connectionPcb != nullptr)
			{
				tcp_err(connectionPcb, nullptr);
				tcp_recv(connectionPcb, nullptr);
				tcp_sent(connectionPcb, nullptr);
				if (unAcked == 0)
				{
					tcp_close(connectionPcb);
				}
				else
				{
					tcp_abort(connectionPcb);
				}
				connectionPcb = nullptr;
			}

			if (receivedData == nullptr || timeoutExceeded)
			{
				DiscardReceivedData();
				state = (localPort == 0 || outgoing) ? SocketState::disabled : SocketState::listening;
			}
		}
		break;
	}

	default:
		// Nothing to do
		break;
	}

	// Restore previous disconnecting state if necessary
	if (wasShortTransfer)
	{
		state = SocketState::peerDisconnecting;
	}
}

// Discard any received data for this transaction. Acquire the lwip mutex before calling this.
void LwipSocket::DiscardReceivedData() noexcept
{
	pbuf *const rdata = receivedData;
	if (rdata != nullptr)
	{
		receivedData = nullptr;
		pbuf_free(rdata);
	}
	readIndex = 0;
}

// Send the data, returning the length buffered
size_t LwipSocket::Send(const uint8_t *data, size_t length) noexcept
{
	MutexLocker lock(lwipMutex);

	if (!CanSend())
	{
		// Don't bother if we cannot send anything at all+
		return 0;
	}

	const size_t bytesLeft = tcp_sndbuf(connectionPcb);
	if (length != 0 && bytesLeft != 0)
	{
		// See how many bytes we can send
		size_t bytesToSend = length;
		if (bytesLeft < length)
		{
			bytesToSend = bytesLeft;
		}

		// Try to send data until we succeed
		err_t err;
		do
		{
			err = tcp_write(connectionPcb, data, bytesToSend, 0);
			if (ERR_IS_FATAL(err))
			{
				Terminate();
				return 0;
			}
			else if (err == ERR_MEM)
			{
				if (bytesToSend == 1 || tcp_sndqueuelen(connectionPcb) >= TCP_SND_QUEUELEN)
				{
					// The buffers are full - try again later
					tcp_output(connectionPcb);
					return 0;
				}
				bytesToSend /= 2;
			}
		}
		while (err == ERR_MEM);

		// Try to send it now
		if (ERR_IS_FATAL(tcp_output(connectionPcb)))
		{
			Terminate();
			return 0;
		}

		// We could successfully send some data
		whenWritten = millis();
		unAcked += bytesToSend;

		return bytesToSend;
	}

	return 0;
}

#endif	// HAS_LWIP_NETWORKING

// End
