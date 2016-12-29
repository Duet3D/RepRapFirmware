/*
 * NetworkTransacrion.cpp
 *
 *  Created on: 23 Dec 2016
 *      Author: David
 */

#include "NetworkTransaction.h"

#include "Network.h"
#include "RepRap.h"
#include "Socket.h"
#include <cstdarg>

//***************************************************************************************************
// NetworkTransaction class

NetworkTransaction *NetworkTransaction::freelist = nullptr;

NetworkTransaction::NetworkTransaction(NetworkTransaction *n)
	: next(n), fileBuffer(nullptr), sendBuffer(nullptr), sendStack(new OutputStack()), fileBeingSent(nullptr), status(TransactionStatus::released)
{
}

void NetworkTransaction::Set(Socket *skt, TransactionStatus s)
{
	cs = skt;
	status = s;
	closeRequested = false;
}

bool NetworkTransaction::HasMoreDataToRead() const
{
	return cs != nullptr && cs->HasMoreDataToRead();
}

bool NetworkTransaction::IsConnected() const
{
	return cs != nullptr && cs->IsConnected();
}

bool NetworkTransaction::IsSending() const
{
	return status == TransactionStatus::sending || status == TransactionStatus::finished;
}

bool NetworkTransaction::CanWrite() const
{
	return status != TransactionStatus::released && cs != nullptr && cs->CanWrite();
}

// Read one char from the NetworkTransaction
bool NetworkTransaction::Read(char& b)
{
	if (cs == nullptr)
	{
		b = 0;
		return false;
	}

	return cs->ReadChar(b);
}

// Read data from the NetworkTransaction and return true on success
bool NetworkTransaction::ReadBuffer(const char *&buffer, size_t &len)
{
	return cs != NoConnection && cs->ReadBuffer(buffer, len);
}

void NetworkTransaction::Write(char b)
{
	if (CanWrite())
	{
		if (sendBuffer == nullptr && !OutputBuffer::Allocate(sendBuffer))
		{
			// Should never get here
			return;
		}
		sendBuffer->cat(b);
	}
}

void NetworkTransaction::Write(const char* s)
{
	if (CanWrite())
	{
		if (sendBuffer == nullptr && !OutputBuffer::Allocate(sendBuffer))
		{
			// Should never get here
			return;
		}
		sendBuffer->cat(s);
	}
}

void NetworkTransaction::Write(StringRef ref)
{
	Write(ref.Pointer(), ref.strlen());
}

void NetworkTransaction::Write(const char* s, size_t len)
{
	if (CanWrite())
	{
		if (sendBuffer == nullptr && !OutputBuffer::Allocate(sendBuffer))
		{
			// Should never get here
			return;
		}
		sendBuffer->cat(s, len);
	}
}

void NetworkTransaction::Write(OutputBuffer *buffer)
{
	if (CanWrite())
	{
		// Note we use an individual stack here, because we don't want to link different
		// OutputBuffers for different destinations together...
		sendStack->Push(buffer);
	}
	else
	{
		// Don't keep buffers we can't send...
		OutputBuffer::ReleaseAll(buffer);
	}
}

void NetworkTransaction::Write(OutputStack *stack)
{
	if (stack != nullptr)
	{
		if (CanWrite())
		{
			sendStack->Append(stack);
		}
		else
		{
			stack->ReleaseAll();
		}
	}
}

void NetworkTransaction::Printf(const char* fmt, ...)
{
	if (CanWrite() && (sendBuffer != nullptr || OutputBuffer::Allocate(sendBuffer)))
	{
		va_list p;
		va_start(p, fmt);
		sendBuffer->vprintf(fmt, p);
		va_end(p);
	}
}

void NetworkTransaction::SetFileToWrite(FileStore *file)
{
	if (CanWrite())
	{
		fileBeingSent = file;
	}
	else if (file != nullptr)
	{
		debugPrintf("Want to write file but can't write\n");
		file->Close();
	}
}

// Get some data to send up to the specified length, which must be nonzero.
// A null return means there is nothing left to send or we couldn't allocate a buffer.
// If there was nothing left to send then the status is changed to 'finished'.
const uint8_t *NetworkTransaction::GetDataToSend(size_t& length)
{
	// Discard any empty output buffers
	while (sendBuffer != nullptr && sendBuffer->BytesLeft() == 0)
	{
		sendBuffer = OutputBuffer::Release(sendBuffer);
		if (sendBuffer == nullptr)
		{
			sendBuffer = sendStack->Pop();
		}
	}

	// If we have an output buffer, send it
	if (sendBuffer != nullptr)
	{
		if (length > sendBuffer->BytesLeft())
		{
			length = sendBuffer->BytesLeft();
		}
		return reinterpret_cast<const uint8_t*>(sendBuffer->Read(length));
	}

	// If we have a file to send, send it
	if (fileBeingSent != nullptr && fileBuffer == nullptr)
	{
		fileBuffer = NetworkBuffer::Allocate();
		if (fileBuffer == nullptr)
		{
			return nullptr;			// no buffer available
		}
	}

	// If we have a file buffer here, we must be in the process of sending a file
	if (fileBuffer != nullptr)
	{
		if (fileBuffer->IsEmpty() && fileBeingSent != nullptr)
		{
			const int bytesRead = fileBuffer->ReadFromFile(fileBeingSent);
			if (bytesRead != (int)NetworkBuffer::bufferSize)
			{
				fileBeingSent->Close();
				fileBeingSent = nullptr;
			}
		}

		if (!fileBuffer->IsEmpty())
		{
			if (length > fileBuffer->Remaining())
			{
				length = fileBuffer->Remaining();
			}
			return fileBuffer->TakeData(length);
		}

		fileBuffer->Release();
		fileBuffer = nullptr;
	}

	status = TransactionStatus::finished;
	return nullptr;
}

// This is called by the Webserver to send output data to a client. If keepConnectionAlive is set to false,
// the current connection will be terminated once everything has been sent.
void NetworkTransaction::Commit(bool keepConnectionAlive)
{
	closeAfterSending = !keepConnectionAlive;
	status = TransactionStatus::sending;
	if (closeAfterSending)
	{
		cs->DiscardReceivedData();
	}
}

// Call this to perform some networking tasks while processing deferred requests,
// and to move this transaction and all transactions that are associated with its
// connection to the end of readyTransactions. There are three ways to do this:
//
// 1) DeferOnly: Do not modify any of the processed data and don't send an ACK.
//               This will ensure that zero-window packets are sent back to the client
// 2) ResetData: Reset the read pointers and acknowledge that the data has been processed
// 3) DiscardData: Free the processed data, acknowledge it and append this transaction as
//                 an empty item again without payload
//
void NetworkTransaction::Defer(DeferralMode mode)
{
	if (mode == DeferralMode::DiscardData)
	{
		cs->DiscardReceivedData();		// discard the incoming data, because we don't need to process it any more
	}

	status = TransactionStatus::deferred;
	reprap.GetNetwork()->Defer(this);
}

// This method should be called if we don't want to send data to the client and if we
// don't want to interfere with the connection state. May also be called from ISR!
void NetworkTransaction::Discard()
{
	status = TransactionStatus::finished;
	cs->ReleaseTransaction();
}

uint32_t NetworkTransaction::GetRemoteIP() const
{
	return (cs != nullptr) ? cs->GetRemoteIP() : 0;
}

Port NetworkTransaction::GetRemotePort() const
{
	return (cs != nullptr) ? cs->GetRemotePort() : 0;
}

Port NetworkTransaction::GetLocalPort() const
{
	return (cs != nullptr) ? cs->GetLocalPort() : 0;
}

// Create the specified number of buffers and put them on the freelist
/*static*/ void NetworkTransaction::AllocateTransactions(unsigned int number)
{
	if (freelist == nullptr)
	{
		while (number != 0)
		{
			freelist = new NetworkTransaction(freelist);
			--number;
		}
	}
}

// Allocate a buffer from the freelist
/*static*/ NetworkTransaction *NetworkTransaction::Allocate()
{
	NetworkTransaction *ret = freelist;
	if (ret != nullptr)
	{
		freelist = ret->next;
		ret->next = nullptr;
		ret->fileBuffer = nullptr;
		ret->fileBeingSent = nullptr;
	}
	return ret;
}

// Free resources and recycle this buffer
NetworkTransaction *NetworkTransaction::Release()
{
	if (fileBuffer != nullptr)
	{
		fileBuffer->Release();
		fileBuffer = nullptr;
	}
	if (fileBeingSent != nullptr)
	{
		fileBeingSent->Close();
		fileBeingSent = nullptr;
	}

	OutputBuffer::ReleaseAll(sendBuffer);
	sendStack->ReleaseAll();

	NetworkTransaction *ret = next;
	next = freelist;
	freelist = this;
	return ret;
}

// End
