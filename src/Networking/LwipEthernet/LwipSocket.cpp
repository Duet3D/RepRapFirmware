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
#include "lwip/altcp.h"

static void conn_err(void *arg, err_t err)
{
	LwipSocket *socket = (LwipSocket *)arg;
	if (socket != nullptr)
	{
		socket->ConnectionError(err);
	}
}

static err_t conn_recv(void *arg, altcp_pcb *pcb, pbuf *p, err_t err)
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

	altcp_abort(pcb);
	return ERR_ABRT;
}

static err_t conn_sent(void *arg, altcp_pcb *pcb, u16_t len)
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
#if LWIP_ALTCP_TLS
		localTlsPort(0),
#endif
		receivedData(nullptr), state(SocketState::disabled), txShutdownRequested(false)
{
	ReInit();
}

bool LwipSocket::UsingTls() const noexcept
{
#if LWIP_ALTCP_TLS
	return connectionPcb != nullptr && altcp_get_port(connectionPcb, 1) == localTlsPort;
#else
	return false;
#endif
}

bool LwipSocket::AcceptConnection(altcp_pcb *pcb) noexcept
{
	const TcpPort incomingPort = altcp_get_port(pcb, 1);
	if ((state == SocketState::listening && (incomingPort == localPort
#if LWIP_ALTCP_TLS
			|| incomingPort == localTlsPort
#endif
		)) ||
		 (state == SocketState::connecting && altcp_get_port(pcb, 0) == remotePort))
	{
		ReInit();
		state = SocketState::connected;
		whenConnected = millis();

		connectionPcb = pcb;
		remoteIPAddress.SetV4LittleEndian(altcp_get_ip(pcb, 0)->addr);

		if (outgoing)
		{
			localPort = altcp_get_port(pcb, 1);
		}
		else
		{
			remotePort = altcp_get_port(pcb, 0);
		}

		altcp_arg(pcb, this);
		altcp_err(pcb, conn_err);
		altcp_recv(pcb, conn_recv);
		altcp_sent(pcb, conn_sent);
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
		altcp_err(connectionPcb, nullptr);
		altcp_recv(connectionPcb, nullptr);
		altcp_sent(connectionPcb, nullptr);
		err_t err = altcp_close(connectionPcb);
		if (err == ERR_OK)
		{
			connectionPcb = nullptr;
		}
		else if (ERR_IS_FATAL(err))
		{
			altcp_abort(connectionPcb);
			connectionPcb = nullptr;
			state = SocketState::aborted;
			return;
		}
	}

	if (connectionPcb == nullptr && state == SocketState::closing && !outgoing)
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
	if (reprap.Debug(Module::Network))
	{
		debugPrintf("LWIP socket error: proto=%d lport=%u rport=%u state=%d err=%d\n", (int)protocol, localPort, remotePort, (int)state, (int)err);
	}

	DiscardReceivedData();
	connectionPcb = nullptr;
	txShutdownRequested = false;

	// For server sockets, always return to listening after an error so new
	// inbound connections are not rejected in conn_accept.
	state = (localPort == 0 || outgoing)
				? SocketState::disabled
				: SocketState::listening;
}

// Initialise a TCP socket
void LwipSocket::Init(SocketNumber skt, TcpPort serverPort, NetworkProtocol p, bool p_outgoing) noexcept
{
	UNUSED(skt);
	outgoing = p_outgoing;

	if (outgoing)
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
#if LWIP_ALTCP_TLS
	localTlsPort = 0;
#endif
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
	txShutdownRequested = false;
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
		if (reprap.Debug(Module::Network))
		{
			debugPrintf("LWIP socket terminate: proto=%d lport=%u rport=%u state=%d\n", (int)protocol, localPort, remotePort, (int)state);
		}

		MutexLocker lock(lwipMutex);
		if (connectionPcb != nullptr)
		{
			altcp_err(connectionPcb, nullptr);
			altcp_recv(connectionPcb, nullptr);
			altcp_sent(connectionPcb, nullptr);
			altcp_abort(connectionPcb);
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
				altcp_recved(connectionPcb, rlen);
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
				altcp_recved(connectionPcb, rlen);
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
			if (reprap.Debug(Module::Network))
			{
				debugPrintf("LWIP connect timeout: proto=%d rport=%u\n", (int)protocol, remotePort);
			}
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
				if (reprap.Debug(Module::Network))
				{
					debugPrintf("LWIP write timeout: proto=%d lport=%u rport=%u\n", (int)protocol, localPort, remotePort);
				}
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
				if (reprap.Debug(Module::Network))
				{
					debugPrintf("LWIP responder timeout: proto=%d lport=%u rport=%u\n", (int)protocol, localPort, remotePort);
				}
				Terminate();
			}
		}
		break;

	case SocketState::peerDisconnecting:
	case SocketState::closing:
	{
		// The connection is being closed. First half-close TX, then wait for peer FIN before full close.
		const bool isTlsConnection = (connectionPcb != nullptr)
#if LWIP_ALTCP_TLS
			&& (localTlsPort != 0)
			&& (altcp_get_port(connectionPcb, 1) == localTlsPort)
#endif
			;
		const uint32_t closeTimeout = isTlsConnection ? (MaxAckTime * 8u) : MaxAckTime;
		const bool timeoutExceeded = millis() - whenClosed > closeTimeout;

		if (connectionPcb != nullptr && state == SocketState::closing && !txShutdownRequested)
		{
			if (isTlsConnection)
			{
				// For TLS sockets, avoid TCP half-close because it bypasses TLS close_notify
				txShutdownRequested = true;
			}
			else
			{
				const err_t err = altcp_shutdown(connectionPcb, 0, 1);
				if (err == ERR_OK || err == ERR_CONN)
				{
					txShutdownRequested = true;
				}
				else if (timeoutExceeded)
				{
					if (!isTlsConnection)
					{
						if (reprap.Debug(Module::Network))
						{
							debugPrintf("LWIP closing timeout abort: proto=%d lport=%u rport=%u unacked=%u\n", (int)protocol, localPort, remotePort, (unsigned int)unAcked);
						}
						altcp_err(connectionPcb, nullptr);
						altcp_recv(connectionPcb, nullptr);
						altcp_sent(connectionPcb, nullptr);
						altcp_abort(connectionPcb);
						connectionPcb = nullptr;
					}
				}
			}
		}

		const bool canFinalize = timeoutExceeded
			|| (state == SocketState::peerDisconnecting && unAcked == 0)
			|| (isTlsConnection && state == SocketState::closing && unAcked == 0);
		if (canFinalize && connectionPcb != nullptr)
		{
			altcp_err(connectionPcb, nullptr);
			altcp_recv(connectionPcb, nullptr);
			altcp_sent(connectionPcb, nullptr);
			if (timeoutExceeded)
			{
				if (reprap.Debug(Module::Network))
				{
					debugPrintf("LWIP close timeout abort: proto=%d lport=%u rport=%u unacked=%u\n", (int)protocol, localPort, remotePort, (unsigned int)unAcked);
				}
				altcp_abort(connectionPcb);
				connectionPcb = nullptr;
			}
			else
			{
				const err_t closeErr = altcp_close(connectionPcb);
				if (closeErr == ERR_OK)
				{
					connectionPcb = nullptr;
				}
				else if (ERR_IS_FATAL(closeErr))
				{
					altcp_abort(connectionPcb);
					connectionPcb = nullptr;
				}
				// If close cannot complete yet (e.g. ERR_INPROGRESS), keep the PCB and retry next poll
			}
		}

		if (connectionPcb == nullptr)
		{
			DiscardReceivedData();
			state = (localPort == 0 || outgoing) ? SocketState::disabled : SocketState::listening;
		}
		break;
	}

	case SocketState::aborted:
		// Keep aborted as a transient state only; recycle passive sockets
		state = (localPort == 0 || outgoing) ? SocketState::disabled : SocketState::listening;
		break;

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
		// Don't bother if we cannot send anything at all
		return 0;
	}

	const size_t bytesLeft = altcp_sndbuf(connectionPcb);
	if (length != 0 && bytesLeft != 0)
	{
		// See how many bytes we can send
		size_t bytesToSend = length;
		if (bytesLeft < length)
		{
			bytesToSend = bytesLeft;
		}

		// Try to send data, halving the size on ERR_MEM until it fits
		err_t err;
		do
		{
			err = altcp_write(connectionPcb, data, bytesToSend, 0);
			if (ERR_IS_FATAL(err))
			{
				Terminate();
				return 0;
			}
			else if (err == ERR_MEM)
			{
				if (bytesToSend == 1 || altcp_sndqueuelen(connectionPcb) >= TCP_SND_QUEUELEN)
				{
					// The buffers are full - try again later
					altcp_output(connectionPcb);
					return 0;
				}
				bytesToSend /= 2;
			}
			else if (err != ERR_OK)
			{
				// Unexpected non-fatal error (e.g. ERR_CONN, ERR_VAL)
				return 0;
			}
		}
		while (err == ERR_MEM);

		// Try to send it now
		if (ERR_IS_FATAL(altcp_output(connectionPcb)))
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
