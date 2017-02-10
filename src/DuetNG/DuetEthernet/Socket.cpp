/*
 * Socket.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include "Socket.h"

#include "NetworkTransaction.h"
#include "NetworkBuffer.h"
#include "Network.h"
#include "RepRap.h"
#include "Webserver.h"
#include "socketlib.h"

//***************************************************************************************************
// Socket class

Socket::Socket() : currentTransaction(nullptr), receivedData(nullptr), state(SocketState::inactive)
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
	// Discard any received data
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
	ReleaseTransaction();

	persistConnection = true;
	isTerminated = false;
	isSending = false;
	needTransaction = false;
	state = SocketState::inactive;

	// Re-initialise the socket on the W5500
	socket(socketNum, Sn_MR_TCP, localPort, 0x00);
}

// Close a connection when the last packet has been sent
void Socket::Close()
{
	disconnectNoWait(socketNum);
}

// Release the current transaction
void Socket::ReleaseTransaction()
{
	if (currentTransaction != nullptr)
	{
		currentTransaction->Release();
		currentTransaction = nullptr;
	}
}

// Terminate a connection immediately
void Socket::Terminate()
{
	disconnectNoWait(socketNum);
	isTerminated = true;
	state = SocketState::inactive;
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
	ReleaseTransaction();
}

// Test whether we have a connection on this socket
bool Socket::IsConnected() const
{
	const uint8_t stat = getSn_SR(socketNum);
	return stat == SOCK_ESTABLISHED || stat == SOCK_CLOSE_WAIT;
}

// Return true if there is or may soon be more data to read
bool Socket::HasMoreDataToRead() const
{
	return (receivedData != nullptr && receivedData->TotalRemaining() != 0)		// already have more data
		|| getSn_SR(socketNum) == SOCK_ESTABLISHED;								// still fully connected, so we may receive more
}

bool Socket::CanWrite() const
{
	const uint8_t stat = getSn_SR(socketNum);
	return stat == SOCK_ESTABLISHED || stat == SOCK_CLOSE_WAIT;
}

// Return true if we are in the sending phase
bool Socket::IsSending() const
{
	return currentTransaction != nullptr && currentTransaction->IsSending();
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
	// Recycle any receive buffers that are now empty
	while (receivedData != nullptr && receivedData->IsEmpty())
	{
		receivedData = receivedData->Release();		// discard empty buffer at head of chain
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
		}

		if (state == SocketState::listening)		// if it is a new connection
		{
			needTransaction = false;
			if (socketNum == FtpSocketNumber || socketNum == TelnetSocketNumber)
			{
				// FTP and Telnet protocols need a connection reply, so tell the Webserver module about the new connection
				if (currentTransaction == nullptr)
				{
					currentTransaction = NetworkTransaction::Allocate();
					if (currentTransaction != nullptr)
					{
						currentTransaction->Set(this, TransactionStatus::connected);
					}
				}
				else
				{
					// This should not happen
					debugPrintf("ERROR:currentTransation should be null but isn't\n");
				}
			}
			state = SocketState::connected;
		}

		{
			// See if the socket has received any data
			const uint16_t len = getSn_RX_RSR(socketNum);
			if (len != 0)
			{
//				debugPrintf("%u available\n", len);
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
//				else debugPrintf("no buffer\n");
			}
		}

		if (currentTransaction == nullptr && (receivedData != nullptr || needTransaction))
		{
			currentTransaction = NetworkTransaction::Allocate();
			if (currentTransaction != nullptr)
			{
				currentTransaction->Set(this, (needTransaction) ? TransactionStatus::acquired : TransactionStatus::receiving);
				needTransaction = false;
			}
		}

		// See if we can send any data.
		// Currently we don't send if we are being called from hsmci because we don't want to risk releasing a buffer that we may be reading data into.
		// We could use a buffer locking mechanism instead. However, the speed of sending is not critical, so we don't do that yet.
		if (full && IsSending() && TrySendData())
		{
			const bool closeAfterSending = currentTransaction->CloseAfterSending();
			ReleaseTransaction();
			if (closeAfterSending)
			{
				ExecCommand(socketNum, Sn_CR_DISCON);
				state = SocketState::closing;
				DiscardReceivedData();
			}
		}
		break;

	case SOCK_CLOSE_WAIT:			// A client has asked to disconnect
#ifdef _HTTPSERVER_DEBUG_
		printf("> HTTPSocket[%d] : ClOSE_WAIT\r\n", socketNum);
#endif
		state = SocketState::clientDisconnecting;
		if (IsSending() && TrySendData())
		{
			ReleaseTransaction();				// finished sending
		}

		// Although there is a transaction status for a client disconnecting, the webserver module does nothing with those transactions.
		// So we don't bother to generate them here.
		if (!IsSending() && currentTransaction == nullptr)
		{
			if (HasMoreDataToRead())
			{
				// We have more received data, so make it available to the webserver to process
				currentTransaction = NetworkTransaction::Allocate();
				if (currentTransaction != nullptr)
				{
					currentTransaction->Set(this, TransactionStatus::receiving);
				}
			}
			else
			{
				ExecCommand(socketNum, Sn_CR_DISCON);
				state = SocketState::closing;
			}
		}
		break;

	case SOCK_CLOSED:
#ifdef _HTTPSERVER_DEBUG_
		printf("> HTTPSocket[%d] : CLOSED\r\n", s);
#endif
		reprap.GetWebserver()->ConnectionLost(this);						// the webserver needs this to be called for both graceful and disgraceful disconnects
		state = SocketState::inactive;

		if (socket(socketNum, Sn_MR_TCP, localPort, 0x00) == socketNum)		// Reinitialize the socket
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

// Try to send data, returning true if all data has been sent and we ought to close the socket
// We have already checked that the socket is in the ESTABLISHED or CLOSE_WAIT state.
bool Socket::TrySendData()
{
	if (isSending)									// are we already sending?
	{
		const uint8_t tmp = getSn_IR(socketNum);
		if (tmp & Sn_IR_SENDOK)						// did the previous send complete?
		{
			setSn_IR(socketNum, Sn_IR_SENDOK);		// if yes
			isSending = false;
		}
		else if(tmp & Sn_IR_TIMEOUT)				// did it time out?
		{
			isSending = false;
			disconnectNoWait(socketNum);			// if so, close the socket
			return true;							// and release buffers etc.
		}
		else
		{
			//debugPrintf("Waiting for data to be sent\n");
			return false;							// last send is still in progress
		}
	}

	// Socket is free to send
	if (currentTransaction->GetStatus() == TransactionStatus::finished)
	{
		return true;
	}

	size_t freesize = (size_t)getSn_TX_FSR(socketNum);
	uint16_t ptr = getSn_TX_WR(socketNum);
	bool sent = false;
	while (freesize != 0)
	{
		size_t length = freesize;
		const uint8_t *data = currentTransaction->GetDataToSend(length);
		if (data == nullptr)
		{
			break;									// no more data or can't allocate a file buffer
		}
		//debugPrintf("rp=%04x tp=%04x\n", getSn_TX_RD(socketNum), getSn_TX_WR(socketNum));
		wiz_send_data_at(socketNum, data, length, ptr);
		//debugPrintf("Wrote %u bytes of %u free, %02x %02x %02x\n", length, freesize, data[0], data[1], data[2]);
		freesize -= length;
		ptr += (uint16_t)length;
		sent = true;
	}

	if (sent)
	{
		//debugPrintf("Sending data, rp=%04x tp=%04x\n", getSn_TX_RD(socketNum), getSn_TX_WR(socketNum));
		ExecCommand(socketNum, Sn_CR_SEND);
		isSending = true;
	}
	else if (currentTransaction->GetStatus() == TransactionStatus::finished)
	{
		return true;								// there was no more data to send
	}

	return false;
}

// Discard any received data for this transaction
void Socket::DiscardReceivedData()
{
	while (receivedData != nullptr)
	{
		receivedData = receivedData->Release();
	}
}

// The webserver calls this to tell the socket that it needs a transaction, e.g. for sending a Telnet os FTP response.
// An empty transaction will do.
// Return true if we can do it, false if the connection is closed or closing.
bool Socket::AcquireTransaction()
{
	if (currentTransaction != nullptr && currentTransaction->GetStatus() == TransactionStatus::acquired)
	{
		return true;
	}

	if (getSn_SR(socketNum) == SOCK_ESTABLISHED)
	{
		needTransaction = true;
		return true;
	}
	return false;
}

// End
