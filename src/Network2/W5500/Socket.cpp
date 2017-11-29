/*
 * Socket.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include <Network2/W5500/Network.h>
#include <Network2/W5500/Socket.h>
#include <Network2/W5500/Wiznet/Ethernet/socketlib.h>
#include "NetworkBuffer.h"
#include "RepRap.h"

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
	if (state != SocketState::disabled && state != SocketState::inactive)
	{
		ExecCommand(socketNum, Sn_CR_DISCON);
		state = SocketState::closing;
		DiscardReceivedData();
		if (protocol == FtpDataProtocol)
		{
			localPort = 0;					// don't re-listen automatically
		}
	}
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
	if (receivedData != nullptr)
	{
		const bool ret = receivedData->ReadChar(c);
		if (receivedData->IsEmpty())
		{
			receivedData = receivedData->Release();
		}
		return ret;
	}

	c = 0;
	return false;
}

// Return a pointer to data in a buffer and a length available
bool Socket::ReadBuffer(const uint8_t *&buffer, size_t &len)
{
	if (receivedData != nullptr)
	{
		len = receivedData->Remaining();
		buffer = receivedData->UnreadData();
		return true;
	}

	return false;
}

// Flag some data as taken from the receive buffers. We never take data from more than one buffer at a time.
void Socket::Taken(size_t len)
{
	if (receivedData != nullptr)
	{
		receivedData->Taken(len);
		if (receivedData->IsEmpty())
		{
			receivedData = receivedData->Release();		// discard empty buffer at head of chain
		}
	}
}

// Poll a socket to see if it needs to be serviced
void Socket::Poll(bool full)
{
	if (state != SocketState::disabled)
	{
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
				if (reprap.GetNetwork().FindResponder(this, protocol))
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
				ReceiveData();
			}
			break;

		case SOCK_CLOSE_WAIT:			// A client has asked to disconnect
			// Check for further incoming packets before this socket is finally closed.
			// This must be done to ensure that FTP uploads are not cut off.
			ReceiveData();

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

// Try to receive more incoming data from the socket
void Socket::ReceiveData()
{
	const uint16_t len = getSn_RX_RSR(socketNum);
	if (len != 0)
	{
//		debugPrintf("%u available\n", len);
		NetworkBuffer * const lastBuffer = NetworkBuffer::FindLast(receivedData);
		if (lastBuffer != nullptr && (lastBuffer->SpaceLeft() >= len || (lastBuffer->SpaceLeft() != 0 && NetworkBuffer::Count(receivedData) >= MaxBuffersPerSocket)))
		{
			const size_t lengthToRead = min<size_t>((size_t)len, lastBuffer->SpaceLeft());
			wiz_recv_data(socketNum, lastBuffer->UnwrittenData(), (uint16_t)lengthToRead);
			lastBuffer->dataLength += lengthToRead;
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("Received %u bytes\n", (unsigned int)lengthToRead);
			}
		}
		else if (NetworkBuffer::Count(receivedData) < MaxBuffersPerSocket)
		{
			NetworkBuffer * const buf = NetworkBuffer::Allocate();
			if (buf != nullptr)
			{
				wiz_recv_data(socketNum, buf->Data(), len);
				ExecCommand(socketNum, Sn_CR_RECV);
				buf->dataLength = (size_t)len;
				NetworkBuffer::AppendToList(&receivedData, buf);
				if (reprap.Debug(moduleNetwork))
				{
					debugPrintf("Received %u bytes\n", (unsigned int)len);
				}
			}
		}
//		else debugPrintf("no buffer\n");
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
		wizTxBufferPtr += length;
		sendOutstanding = true;
		if (wizTxBufferLeft == 0)
		{
			Send();
		}
		return length;
	}
	return 0;
}

// Tell the interface to send the outstanding data
void Socket::Send()
{
	if (CanSend() && sendOutstanding)
	{
		setSn_TX_WR(socketNum, wizTxBufferPtr);
		ExecCommand(socketNum, Sn_CR_SEND);
		isSending = true;
		sendOutstanding = false;
	}
}

// End
