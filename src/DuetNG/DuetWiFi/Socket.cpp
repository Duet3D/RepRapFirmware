/*
 * WiFiSocket.cpp
 *
 *  Created on: 22 Apr 2017
 *      Author: David
 */

#include "Socket.h"
#include "NetworkBuffer.h"

Socket::Socket() : localPort(0), receivedData(nullptr), state(SocketState::disabled)
{

}

void Socket::Init(SocketNumber n)
{
	socketNum = n;
	state = SocketState::disabled;
}

// Poll a socket to see if it needs to be serviced
void Socket::Poll(bool full)
{
	//TODO
}

// Close a connection when the last packet has been sent
void Socket::Close()
{
	//TODO
}

// Terminate a connection immediately
void Socket::Terminate()
{
	//TODO
	state = SocketState::disabled;
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

// Return a pointer to data in a buffer and a length available, and mark the data as taken
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

// Send the data, returning the length buffered
size_t Socket::Send(const uint8_t *data, size_t length)
{
	//TODO
	return 0;
}

// Tell the interface to send the outstanding data
void Socket::Send()
{
	//TODO
}

// End
