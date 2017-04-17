/*
 * Socket.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include "Socket.h"

#include "NetworkBuffer.h"
#include "Network.h"
#include "RepRap.h"
#include "socketlib.h"

//***************************************************************************************************
// Socket class

const uint32_t FindResponderTimeout = 2000;			// how long we wait for a responder to become available
const unsigned int MaxBuffersPerSocket = 4;

Socket::Socket() : receivedData(nullptr), state(SocketState::disabled)
{
}

// Initialise a TCP socket
void Socket::Init(SocketNumber skt, Port serverPort, Protocol p)
{
	socketNum = skt;
	localPort = serverPort;
	protocol = p;
	ReInit();
}

void Socket::TerminateAndDisable()
{
	Terminate();
	state = SocketState::disabled;
}

void Socket::ReInit()
{
	// Discard any received data
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}

	persistConnection = true;
	isTerminated = false;
	isSending = false;
	state = SocketState::inactive;

	// Re-initialise the socket on the W5500
	socket(socketNum, Sn_MR_TCP, localPort, 0x00);
}

// Close a connection when the last packet has been sent
void Socket::Close()
{
	ExecCommand(socketNum, Sn_CR_DISCON);
	state = SocketState::closing;
	DiscardReceivedData();
}

// Terminate a connection immediately
void Socket::Terminate()
{
	if (state != SocketState::disabled)
	{
		disconnectNoWait(socketNum);
		isTerminated = true;
		state = SocketState::inactive;
		DiscardReceivedData();
	}
}

// Test whether we have a connection on this socket
bool Socket::IsConnected() const
{
	if (state == SocketState::disabled)
	{
		return false;
	}
	const uint8_t stat = getSn_SR(socketNum);
	return stat == SOCK_ESTABLISHED || stat == SOCK_CLOSE_WAIT;
}

// Return true if there is or may soon be more data to read
bool Socket::CanRead() const
{
	return (state == SocketState::connected)
		|| (state == SocketState::clientDisconnecting && receivedData != nullptr && receivedData->TotalRemaining() != 0);
}

bool Socket::CanSend() const
{
	return state == SocketState::connected;
}

// Read 1 character from the receive buffers, returning true if successful
bool Socket::ReadChar(char& c)
{
	while (receivedData != nullptr && receivedData->IsEmpty())
	{
		receivedData = receivedData->Release();		// discard empty buffer at head of chain
	}

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

// Return a pointer to data in a buffer and a length available, and mark the data as taken
bool Socket::ReadBuffer(const char *&buffer, size_t &len)
{
	while (receivedData != nullptr && receivedData->IsEmpty())
	{
		receivedData = receivedData->Release();		// discard empty buffer at head of chain
	}

	if (receivedData == nullptr)
	{
		return false;
	}

	len = NetworkBuffer::bufferSize;				// initial value passed to TakeData is the maximum amount we will take
	buffer = reinterpret_cast<const char*>(receivedData->TakeData(len));
//	debugPrintf("Taking %d bytes\n", len);
	return true;
}

// Poll a socket to see if it needs to be serviced
void Socket::Poll(bool full)
{
	if (state != SocketState::disabled)
	{
		// The mechanism used by class OutputBuffer and now by NetworkBuffer of marking data taken as soon as we return a pointer to it
		// is DANGEROUS and will have to be rewritten for RTOS. We need to recycle empty buffers, otherwise multiple file uploads get stalled.
		// However, we MUST NOT do this until the data had DEFINITELY been finished with. Temporarily use this conditional to avoid a bug
		// with data corruption when this is not the case.
		if (full)
		{
			// Recycle any receive buffers that are now empty
			while (receivedData != nullptr && receivedData->IsEmpty())
			{
				receivedData = receivedData->Release();		// discard empty buffer at head of chain
			}
		}

		switch(getSn_SR(socketNum))
		{
		case SOCK_INIT:
			// Socket has been initialised but is not listening yet
			if (localPort != 0)			// localPort for the FTP data socket is 0 until we have decided what port number to use
			{
				ExecCommand(socketNum, Sn_CR_LISTEN);
				state = SocketState::listening;
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
				whenConnected = millis();
			}

			if (full && state == SocketState::listening)		// if it is a new connection
			{
				if (reprap.GetNetwork()->FindResponder(this, protocol))
				{
					state = SocketState::connected;
					sendOutstanding = false;
				}
				else
				{
					if (millis() - whenConnected >= FindResponderTimeout)
					{
						Terminate();
					}
				}
			}

			if (state == SocketState::connected)
			{
				// See if the socket has received any data
				//TODO limit the number of buffers used by each socket
				const uint16_t len = getSn_RX_RSR(socketNum);
				if (len != 0 && NetworkBuffer::Count(receivedData) < MaxBuffersPerSocket)
				{
//					debugPrintf("%u available\n", len);
					// There is data available, so allocate a buffer
					//TODO: if there is already a buffer and it is in an appropriate state (i.e. receiving) and it has enough room, we could just append the data
					NetworkBuffer * const buf = NetworkBuffer::Allocate();
					if (buf != nullptr)
					{
						wiz_recv_data(socketNum, buf->Data(), len);
						ExecCommand(socketNum, Sn_CR_RECV);
						buf->dataLength = (size_t)len;
						buf->readPointer = 0;
						NetworkBuffer::AppendToList(&receivedData, buf);
					}
//					else debugPrintf("no buffer\n");
				}
			}
			break;

		case SOCK_CLOSE_WAIT:			// A client has asked to disconnect
			state = SocketState::clientDisconnecting;
			break;

		case SOCK_CLOSED:
			ReInit();
			break;

		default:
			break;
		}
	}
}

// Discard any received data for this transaction
void Socket::DiscardReceivedData()
{
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
}

// Send the data, returning the length buffered
size_t Socket::Send(const uint8_t *data, size_t length)
{
	if (CanSend() && length != 0 && getSn_SR(socketNum) == SOCK_ESTABLISHED)
	{
		// Check for previous send complete
		if (isSending)									// are we already sending?
		{
			const uint8_t tmp = getSn_IR(socketNum);
			if (tmp & Sn_IR_SENDOK)						// did the previous send complete?
			{
				setSn_IR(socketNum, Sn_IR_SENDOK);		// if yes
				isSending = false;
			}
			else if (tmp & Sn_IR_TIMEOUT)				// did it time out?
			{
				isSending = false;
				disconnectNoWait(socketNum);			// if so, close the socket
				state = SocketState::aborted;
				return 0;								// and release buffers etc.
			}
			else
			{
				return 0;								// last send is still in progress
			}
		}

		if (!sendOutstanding)
		{
			wizTxBufferLeft = getSn_TX_FSR(socketNum);	// get free buffer space
			if (wizTxBufferLeft == 0)
			{
				return 0;
			}
			wizTxBufferPtr = getSn_TX_WR(socketNum);
		}

		if (length > wizTxBufferLeft)
		{
			length = wizTxBufferLeft;
		}
		wiz_send_data_at(socketNum, data, length, wizTxBufferPtr);
		wizTxBufferLeft -= length;
		if (wizTxBufferLeft == 0)
		{
			// Buffer is full so send it
			ExecCommand(socketNum, Sn_CR_SEND);
			isSending = true;
			sendOutstanding = false;
		}
		else
		{
			wizTxBufferPtr += length;
			sendOutstanding = true;
		}
		return length;
	}
	return 0;
}

void Socket::Send()
{
	if (CanSend() && sendOutstanding)
	{
		ExecCommand(socketNum, Sn_CR_SEND);
		isSending = true;
		sendOutstanding = false;
	}
}

// End
