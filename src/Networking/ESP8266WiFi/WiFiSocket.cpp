/*
 * WiFiSocket.cpp
 *
 *  Created on: 22 Apr 2017
 *      Author: David
 */

#include "WiFiSocket.h"

#if HAS_WIFI_NETWORKING

#include <Networking/NetworkBuffer.h>
#include <Platform/RepRap.h>
#include "WiFiInterface.h"

const unsigned int MaxBuffersPerSocket = 4;

WiFiSocket::WiFiSocket(NetworkInterface *iface) noexcept : Socket(iface), receivedData(nullptr), state(SocketState::inactive), needsPolling(false)
{
}

void WiFiSocket::Init(SocketNumber n) noexcept
{
	socketNum = n;
	state = SocketState::inactive;
	txBufferSpace = 0;
}

// Close a connection when the last packet has been sent
void WiFiSocket::Close() noexcept
{
	if (state == SocketState::connected || state == SocketState::clientDisconnecting)
	{
		const int32_t reply = GetInterface()->SendCommand(NetworkCommand::connClose, socketNum, 0, 0, nullptr, 0, nullptr, 0);
		if (reply == ResponseEmpty)
		{
			state = (state == SocketState::connected) ? SocketState::closing : SocketState::inactive;
			return;
		}
	}

	if (reprap.Debug(moduleNetwork))
	{
		debugPrintf("close failed, in wrong state\n");
	}
	Terminate();							// something is not right, so terminate the socket for safety
}

// Terminate a connection immediately
// We can call this after any sort of error on a socket as long as it is in use.
void WiFiSocket::Terminate() noexcept
{
	if (state != SocketState::inactive)
	{
		const int32_t reply = GetInterface()->SendCommand(NetworkCommand::connAbort, socketNum, 0, 0, nullptr, 0, nullptr, 0);
		state = (reply != 0) ? SocketState::broken : SocketState::inactive;
	}
	DiscardReceivedData();
	txBufferSpace = 0;
}

// Return true if there is or may soon be more data to read
bool WiFiSocket::CanRead() const noexcept
{
	return (state == SocketState::connected)
		|| (state == SocketState::clientDisconnecting && receivedData != nullptr && receivedData->TotalRemaining() != 0);
}

// Return true if we can send data to this socket
bool WiFiSocket::CanSend() const noexcept
{
	return state == SocketState::connected;
}

// Read 1 character from the receive buffers, returning true if successful
bool WiFiSocket::ReadChar(char& c) noexcept
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
bool WiFiSocket::ReadBuffer(const uint8_t *&buffer, size_t &len) noexcept
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
void WiFiSocket::Taken(size_t len) noexcept
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
void WiFiSocket::Poll() noexcept
{
	// Get the socket status
	Receiver<ConnStatusResponse> resp;
	const int32_t ret = GetInterface()->SendCommand(NetworkCommand::connGetStatus, socketNum, 0, nullptr, 0, resp);
	if (ret != (int32_t)resp.Size())
	{
		// We can't do much here other than disable and restart wifi, or hope the next status call succeeds
		if (reprap.Debug(moduleNetwork))
		{
			debugPrintf("Bad recv status size\n");
		}
		return;
	}

	// As well as getting the status for the socket we asked about, we also received bitmaps of connected sockets.
	// Pass these to the Network module so that it can avoid polling idle sockets.
	GetInterface()->UpdateSocketStatus(resp.Value().connectedSockets, resp.Value().otherEndClosedSockets);

	switch (resp.Value().state)
	{
	case ConnState::otherEndClosed:
		// Check for further incoming packets before this socket is finally closed.
		// This must be done to ensure that FTP uploads are not cut off.
		ReceiveData(resp.Value().bytesAvailable);

		if (state == SocketState::clientDisconnecting)
		{
			// We already got here before, so close the connection once and for all
			Close();
			break;
		}
		else if (state != SocketState::inactive)
		{
			state = SocketState::clientDisconnecting;
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("Client disconnected on socket %u\n", socketNum);
			}
			break;
		}
		// We can get here if a client has sent very little data and then instantly closed
		// the connection, e.g. when an FTP client transferred very small files over the
		// data port. In such cases we must notify the responder about this transmission!
		// no break

	case ConnState::connected:
		if (state != SocketState::connected)
		{
			// It's a new connection
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("New conn on socket %u for local port %u\n", socketNum, localPort);
			}
			localPort = resp.Value().localPort;
			remotePort = resp.Value().remotePort;
			remoteIPAddress.SetV4LittleEndian(resp.Value().remoteIp);
			DiscardReceivedData();
			if (state != SocketState::waitingForResponder)
			{
				WiFiInterface *iface = static_cast<WiFiInterface *>(interface);
				protocol = iface->GetProtocolByLocalPort(localPort);

				whenConnected = millis();
				state = SocketState::waitingForResponder;
			}
			if (reprap.GetNetwork().FindResponder(this, protocol))
			{
				state = (resp.Value().state == ConnState::connected) ? SocketState::connected : SocketState::clientDisconnecting;
				if (reprap.Debug(moduleNetwork))
				{
					debugPrintf("Found responder\n");
				}
			}
			else if (millis() - whenConnected >= FindResponderTimeout)
			{
				Terminate();
				if (reprap.Debug(moduleNetwork))
				{
					debugPrintf("No responder, new conn %u terminated\n", socketNum);
				}
			}
		}

		if (state == SocketState::connected)
		{
			txBufferSpace = resp.Value().writeBufferSpace;
			ReceiveData(resp.Value().bytesAvailable);
		}
		break;

	case ConnState::aborted:
		if (reprap.Debug( moduleNetwork))
		{
			debugPrintf("Socket %u aborted\n", socketNum);
		}
		state = SocketState::broken;			// make sure that the Terminate call send a command to the WiFi module
		Terminate();
		break;

	default:
		if (state == SocketState::connected || state == SocketState::waitingForResponder)
		{
			// Unexpected change of state
			if (state != SocketState::clientDisconnecting && reprap.Debug(moduleNetwork))
			{
				debugPrintf("Unexpected state change on socket %u\n", socketNum);
			}
			state = SocketState::broken;
		}
		else if (state == SocketState::closing)
		{
			// Socket closed
			state = SocketState::inactive;
		}
		break;
	}

	needsPolling = false;
}

WiFiInterface *WiFiSocket::GetInterface() const noexcept
{
	return static_cast<WiFiInterface *>(interface);
}

// Try to receive more incoming data from the socket.
void WiFiSocket::ReceiveData(uint16_t bytesAvailable) noexcept
{
	if (bytesAvailable != 0)
	{
//		debugPrintf("%u available\n", bytesAvailable);
		// First see if we already have a buffer with enough room
		NetworkBuffer *const lastBuffer = NetworkBuffer::FindLast(receivedData);
		if (lastBuffer != nullptr && (bytesAvailable <= lastBuffer->SpaceLeft() || (lastBuffer->SpaceLeft() != 0 && NetworkBuffer::Count(receivedData) >= MaxBuffersPerSocket)))
		{
			// Read data into the existing buffer
			const size_t maxToRead = min<size_t>(lastBuffer->SpaceLeft(), MaxDataLength);
			const int32_t ret = GetInterface()->SendCommand(NetworkCommand::connRead, socketNum, 0, 0, nullptr, 0, lastBuffer->UnwrittenData(), maxToRead);
			if (ret > 0 && (size_t)ret <= maxToRead)
			{
				lastBuffer->dataLength += (size_t)ret;
				if (reprap.Debug(moduleNetwork))
				{
					debugPrintf("Received %u bytes\n", (unsigned int)ret);
				}
			}
		}
		else if (NetworkBuffer::Count(receivedData) < MaxBuffersPerSocket)
		{
			NetworkBuffer * const buf = NetworkBuffer::Allocate();
			if (buf != nullptr)
			{
				const size_t maxToRead = min<size_t>(NetworkBuffer::bufferSize, MaxDataLength);
				const int32_t ret = GetInterface()->SendCommand(NetworkCommand::connRead, socketNum, 0, 0, nullptr, 0, buf->Data(), maxToRead);
				if (ret > 0 && (size_t)ret <= maxToRead)
				{
					buf->dataLength = (size_t)ret;
					NetworkBuffer::AppendToList(&receivedData, buf);
					if (reprap.Debug(moduleNetwork))
					{
						debugPrintf("Received %u bytes\n", (unsigned int)ret);
					}
				}
				else
				{
					buf->Release();
				}
			}
//			else debugPrintf("no buffer\n");
		}
	}
}

// Discard any received data for this transaction
void WiFiSocket::DiscardReceivedData() noexcept
{
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
}

// Send the data, returning the length buffered
size_t WiFiSocket::Send(const uint8_t *data, size_t length) noexcept
{
	if (state == SocketState::connected && txBufferSpace != 0)
	{
		const size_t lengthToSend = min<size_t>(length, min<size_t>(txBufferSpace, MaxDataLength));
		const int32_t reply = GetInterface()->SendCommand(NetworkCommand::connWrite, socketNum, 0, 0, data, lengthToSend, nullptr, 0);
		if (reply >= 0 && (size_t)reply <= lengthToSend)
		{
			txBufferSpace -= (size_t)reply;
			return (size_t)reply;
		}
		if (reprap.Debug(moduleNetwork))
		{
			debugPrintf("Send failed, terminating\n");
		}
		state = SocketState::broken;							// something is not right, terminate the socket soon
	}
	return 0;
}

// Tell the interface to send the outstanding data
void WiFiSocket::Send() noexcept
{
	if (state == SocketState::connected)
	{
		const int32_t reply = GetInterface()->SendCommand(NetworkCommand::connWrite, socketNum, MessageHeaderSamToEsp::FlagPush, 0, nullptr, 0, nullptr, 0);
		if (reply < 0)
		{
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("Send failed, terminating\n");
			}
			state = SocketState::broken;						// something is not right, terminate the socket soon
		}
	}
}

// Return true if we need to poll this socket
bool WiFiSocket::NeedsPolling() const noexcept
{
	return state != SocketState::inactive || needsPolling;
}

#endif	// HAS_WIFI_NETWORKING

// End
