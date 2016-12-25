/*
 * Socket.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include "Socket.h"
#include "NetworkTransaction.h"
#include "NetworkBuffer.h"
#include <socketlib.h>

//***************************************************************************************************
// Socket class

Socket::Socket() : receivedData(nullptr), sendingTransaction(nullptr)
{
}

// Initialise a TCP socket
void Socket::Init(SocketNumber skt, Port serverPort)
{
	socketNum = skt;
	localPort = serverPort;
	ReInit();
}

void Socket::ReInit()
{
	// Discard any transactions that were queued to send
	NetworkTransaction *st;
	while ((st = sendingTransaction)  != nullptr)
	{
		sendingTransaction = st->GetNext();
		st->Discard();
	}

	// Discard any received data
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}

	persistConnection = true;
	isTerminated = false;
	state = SocketState::inactive;

	// Re-initialise the socket on the W5500
	socket(socketNum, Sn_MR_TCP, localPort, 0x00);
}

// Terminate a connection
void Socket::Terminate()
{
	disconnectNoWait(socketNum);
	isTerminated = true;
	state = SocketState::inactive;
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
}

// Test whether we have a connection on this socket
bool Socket::IsConnected() const
{
	return getSn_SR(socketNum) == SOCK_ESTABLISHED;		//TODO is this right?
}

// Read 1 character from the receive buffers, returning true if successful
bool Socket::ReadChar(char& c)
{
	if (receivedData == nullptr)
	{
		c = 0;
		return false;
	}

	bool ret = receivedData->ReadChar(c);
	if (receivedData->IsEmpty())
	{
		receivedData = receivedData->Release();
	}
	return ret;
}

// Poll a socket to see if it needs to be serviced
void Socket::Poll()
{
	switch(getSn_SR(socketNum))
	{
	case SOCK_INIT:
		// Socket has been initialised but is not listening yet
		if (localPort != 0)			// localPort for the FTP data socket is 0 until we have decided what port number to use
		{
			listen(socketNum);
		}
		break;

	case SOCK_LISTEN:				// Socket is listening but no client has connected to it yet
		break;

	case SOCK_ESTABLISHED:			// A client is connected to this socket
		if (getSn_IR(socketNum) & Sn_IR_CON)
		{
			// New connection, so retrieve the sending IP address and port, and clear the interrupt
			getSn_DIPR(socketNum, reinterpret_cast<uint8_t*>(&remoteIPAddress));
			remotePort = getSn_DPORT(socketNum);
			setSn_IR(socketNum, Sn_IR_CON);
		}

		// See if the socket has received any data
		{
			const uint16_t len = getSn_RX_RSR(socketNum);
			if (len != 0)
			{
				// There is data available, so allocate a buffer
				NetworkBuffer * const buf = NetworkBuffer::Allocate();
				if (buf != nullptr)
				{
					buf->dataLength = recv(socketNum, buf->data, min<size_t>(len, NetworkBuffer::bufferSize));
					buf->readPointer = 0;
					// Append the buffer to the list of receive buffers
					NetworkBuffer** bufp = &receivedData;
					while (*bufp != nullptr)
					{
						bufp = &((*bufp)->next);
					}
					*bufp = buf;
				}
			}
		}

		// See if we can send any data
		//TODO
		break;

	case SOCK_CLOSE_WAIT:			// A client has asked to disconnect
#ifdef _HTTPSERVER_DEBUG_
		printf("> HTTPSocket[%d] : ClOSE_WAIT\r\n", socketNum);
#endif
		disconnect(socketNum);
		break;

	case SOCK_CLOSED:
#ifdef _HTTPSERVER_DEBUG_
		printf("> HTTPSocket[%d] : CLOSED\r\n", s);
#endif
		if (socket(socketNum, Sn_MR_TCP, localPort, 0x00) == socketNum)    // Reinitialize the socket
		{
#ifdef _HTTPSERVER_DEBUG_
			printf("> HTTPSocket[%d] : OPEN\r\n", socketNum);
#endif
		}
		break;

	default:
		break;
	} // end of switch

#ifdef _USE_WATCHDOG_
	HTTPServer_WDT_Reset();
#endif
}

// End
