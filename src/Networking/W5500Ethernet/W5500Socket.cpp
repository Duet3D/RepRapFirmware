/*
 * Socket.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include "W5500Socket.h"
#include "Network.h"
#include "NetworkDefs.h"
#include "NetworkInterface.h"
#include "Wiznet/Ethernet/socketlib.h"
#include "NetworkBuffer.h"
#include "RepRap.h"

//***************************************************************************************************
// Socket class

const unsigned int MaxBuffersPerSocket = 4;

W5500Socket::W5500Socket(NetworkInterface *iface)
	: Socket(iface), receivedData(nullptr)
{
}

// Initialise a TCP socket
void W5500Socket::Init(SocketNumber skt, Port serverPort, NetworkProtocol p)
{
	socketNum = skt;
	localPort = serverPort;
	protocol = p;
	ReInit();
}

void W5500Socket::TerminateAndDisable()
{
	if (state != SocketState::disabled)		// we must not call close() if the socket has never been initialised, because socketNum won't have been initialised
	{
		MutexLocker lock(interface->interfaceMutex);

		Terminate();
		close(socketNum);
		state = SocketState::disabled;
	}
}

void W5500Socket::ReInit()
{
	MutexLocker lock(interface->interfaceMutex);

	// Discard any received data
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}

	persistConnection = true;
	isTerminated = false;
	isSending = false;

	// Re-initialise the socket on the W5500
	if (protocol != MdnsProtocol)
	{
		state = SocketState::inactive;

		socket(socketNum, Sn_MR_TCP, localPort, 0x00);
	}
	else
	{
		state = SocketState::listening;

		uint8_t har[6];
		memcpy(har, MdnsMacAddress, sizeof(har));
		setSn_DHAR(socketNum, har);							// NB: Using a constexpr value directly does not work here!
		setSn_DIPR(socketNum, (IPAddress)MdnsIPAddress);
		setSn_DPORT(socketNum, MdnsPort);

		socket(socketNum, Sn_MR_UDP, MdnsPort, SF_MULTI_ENABLE);
	}
}

// Close a connection when the last packet has been sent
void W5500Socket::Close()
{
	MutexLocker lock(interface->interfaceMutex);

	if (state != SocketState::disabled && state != SocketState::inactive)
	{
		if (protocol != MdnsProtocol)
		{
			ExecCommand(socketNum, Sn_CR_DISCON);
		}
		state = SocketState::closing;
		DiscardReceivedData();
		if (protocol == FtpDataProtocol)
		{
			localPort = 0;					// don't re-listen automatically
		}
	}
}

// Terminate a connection immediately
void W5500Socket::Terminate()
{
	MutexLocker lock(interface->interfaceMutex);

	if (state != SocketState::disabled)
	{
		disconnectNoWait(socketNum);
		isTerminated = true;
		state = SocketState::inactive;
		DiscardReceivedData();
	}
}

// Return true if there is or may soon be more data to read
bool W5500Socket::CanRead() const
{
	return (state == SocketState::connected)
		|| (state == SocketState::listening && protocol == MdnsProtocol)
		|| (state == SocketState::clientDisconnecting && receivedData != nullptr && receivedData->TotalRemaining() != 0);
}

bool W5500Socket::CanSend() const
{
	return state == SocketState::connected || (state == SocketState::listening && protocol == MdnsProtocol);
}

// Read 1 character from the receive buffers, returning true if successful
bool W5500Socket::ReadChar(char& c)
{
	if (receivedData != nullptr)
	{
		MutexLocker lock(interface->interfaceMutex);

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
bool W5500Socket::ReadBuffer(const uint8_t *&buffer, size_t &len)
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
void W5500Socket::Taken(size_t len)
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
void W5500Socket::Poll(bool full)
{
	if (state != SocketState::disabled)
	{
		MutexLocker lock(interface->interfaceMutex);

		switch(getSn_SR(socketNum))
		{
		case SOCK_INIT:					// Socket has been initialised but is not listening yet
			if (localPort != 0)			// localPort for the FTP data socket is 0 until we have decided what port number to use
			{
				ExecCommand(socketNum, Sn_CR_LISTEN);
				state = SocketState::listening;
			}
			break;

		case SOCK_UDP:					// Socket is ready to receive UDP data
			ReceiveData();
			break;

		case SOCK_LISTEN:				// Socket is listening but no client has connected to it yet
			break;

		case SOCK_ESTABLISHED:			// A client is connected to this socket
			if (getSn_IR(socketNum) & Sn_IR_CON)
			{
				// New connection, so retrieve the sending IP address and port, and clear the interrupt
				getSn_DIPR(socketNum, remoteIPAddress);
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
				else if (millis() - whenConnected >= FindResponderTimeout)
				{
					if (reprap.Debug(moduleNetwork))
					{
						debugPrintf("Timed out waiting for resonder for port %u\n", localPort);
					}
					Terminate();
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

// Try to receive more incoming data from the socket. The mutex is already owned.
void W5500Socket::ReceiveData()
{
	const uint16_t len = getSn_RX_RSR(socketNum);
	if (len != 0 && len <= NetworkBuffer::bufferSize)
	{
//		debugPrintf("%u available\n", len);
		NetworkBuffer * const lastBuffer = NetworkBuffer::FindLast(receivedData);
		// NOTE: reading only part of the received data doesn't work because the wizchip doesn't track the buffer pointer properly.
		// We could probably make it work by tracking the buffer pointer ourselves, just as we do when sending data, and using wiz_recv_data_at.
		if (lastBuffer != nullptr && lastBuffer->SpaceLeft() >= len)
		{
			wiz_recv_data(socketNum, lastBuffer->UnwrittenData(), len);
			ExecCommand(socketNum, Sn_CR_RECV);
			lastBuffer->dataLength += len;
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("Appended %u bytes\n", (unsigned int)len);
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

// Discard any received data for this transaction. The mutex is already owned.
void W5500Socket::DiscardReceivedData()
{
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
}

// Send the data, returning the length buffered
size_t W5500Socket::Send(const uint8_t *data, size_t length)
{
	MutexLocker lock(interface->interfaceMutex);

	const uint8_t status = getSn_SR(socketNum);
	if (CanSend() && length != 0 && (status == SOCK_ESTABLISHED || status == SOCK_UDP))
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
void W5500Socket::Send()
{
	MutexLocker lock(interface->interfaceMutex);

	if (CanSend() && sendOutstanding)
	{
		setSn_TX_WR(socketNum, wizTxBufferPtr);
		ExecCommand(socketNum, (protocol != MdnsProtocol) ? Sn_CR_SEND : Sn_CR_SEND_MAC);
		isSending = true;
		sendOutstanding = false;
	}
}

// End
