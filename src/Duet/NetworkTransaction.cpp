/*
 * NetworkTransaction.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include "NetworkTransaction.h"

#include "ConnectionState.h"
#include "Network.h"
#include "OutputMemory.h"
#include "Platform.h"
#include "RepRap.h"
#include "Storage/FileStore.h"

#include "lwip/src/include/lwip/tcp.h"
#include "lwip/src/include/lwip/tcp_impl.h"

extern "C" {

static err_t conn_poll(void *arg, tcp_pcb *pcb)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs == sendingConnection)
	{
		// Data could not be sent last time, check if the connection has to be timed out
		sendingRetries++;
		if (sendingRetries == TCP_MAX_SEND_RETRIES)
		{
			reprap.GetPlatform().MessageF(UsbMessage, "Network: Could not transmit data after %.1f seconds\n", (double)((float)TCP_WRITE_TIMEOUT / 1000.0));
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		// Try to write the remaining data once again (if required)
		if (writeResult != ERR_OK)
		{
			writeResult = tcp_write(pcb, sendingWindow + (sendingWindowSize - sentDataOutstanding), sentDataOutstanding, 0);
			if (ERR_IS_FATAL(writeResult))
			{
				reprap.GetPlatform().MessageF(UsbMessage, "Network: Failed to write data in conn_poll (code %d)\n", writeResult);
				tcp_abort(pcb);
				return ERR_ABRT;
			}

			if (writeResult != ERR_OK && reprap.Debug(moduleNetwork))
			{
				reprap.GetPlatform().MessageF(UsbMessage, "Network: tcp_write resulted in error code %d\n", writeResult);
			}
		}

		// If that worked, try to output the remaining data (if required)
		if (outputResult != ERR_OK)
		{
			outputResult = tcp_output(pcb);
			if (ERR_IS_FATAL(outputResult))
			{
				reprap.GetPlatform().MessageF(UsbMessage, "Network: Failed to output data in conn_poll (code %d)\n", outputResult);
				tcp_abort(pcb);
				return ERR_ABRT;
			}

			if (outputResult != ERR_OK && reprap.Debug(moduleNetwork))
			{
				reprap.GetPlatform().MessageF(UsbMessage, "Network: tcp_output resulted in error code %d\n", outputResult);
			}
		}
	}
	else
	{
		reprap.GetPlatform().Message(UsbMessage, "Network: Mismatched pcb in conn_poll!\n");
	}
	return ERR_OK;
}

static err_t conn_sent(void *arg, tcp_pcb *pcb, u16_t len)
{
	ConnectionState *cs = (ConnectionState*)arg;
	if (cs == sendingConnection)
	{
		if (sentDataOutstanding > len)
		{
			sentDataOutstanding -= len;
		}
		else
		{
			tcp_poll(pcb, nullptr, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
			sendingConnection = nullptr;
		}
	}
	else
	{
		reprap.GetPlatform().Message(UsbMessage, "Network: Mismatched pcb in conn_sent!\n");
	}
	return ERR_OK;
}


}

//***************************************************************************************************
// NetworkTransaction class

NetworkTransaction::NetworkTransaction(NetworkTransaction *n) : next(n), status(released)
{
	sendStack = new OutputStack();
}

void NetworkTransaction::Set(pbuf *p, ConnectionState *c, TransactionStatus s)
{
	cs = c;
	pb = readingPb = p;
	status = s;
	inputPointer = 0;
	sendBuffer = nullptr;
	fileBeingSent = nullptr;
	closeRequested = false;
	nextWrite = nullptr;
	dataAcknowledged = false;
}

// Read one char from the NetworkTransaction
bool NetworkTransaction::Read(char& b)
{
	if (readingPb == nullptr)
	{
		b = 0;
		return false;
	}

	b = ((const char*)readingPb->payload)[inputPointer++];
	if (inputPointer == readingPb->len)
	{
		readingPb = readingPb->next;
		inputPointer = 0;
	}
	return true;
}

// Read data from the NetworkTransaction and return true on success
bool NetworkTransaction::ReadBuffer(const char *&buffer, size_t &len)
{
	if (readingPb == nullptr)
	{
		return false;
	}

	if (inputPointer >= readingPb->len)
	{
		readingPb = readingPb->next;
		inputPointer = 0;
		if (readingPb == nullptr)
		{
			return false;
		}
	}

	buffer = (const char*)readingPb->payload + inputPointer;
	len = readingPb->len - inputPointer;
	readingPb = readingPb->next;
	inputPointer = 0;
	return true;
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
			sendStack->Append(*stack);
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
		file->Close();
	}
}

// Send exactly one TCP window of data and return true when this transaction can be released
bool NetworkTransaction::Send()
{
	// Free up this transaction if the connection is supposed to be closed
	if (closeRequested)
	{
		reprap.GetNetwork().ConnectionClosed(cs, true);	// This will release the transaction too
		return false;
	}

	// Fill up the TCP window with some data chunks from our OutputBuffer instances
	size_t bytesBeingSent = 0, bytesLeftToSend = TCP_WND;
	while (sendBuffer != nullptr && bytesLeftToSend > 0)
	{
		size_t copyLength = min<size_t>(bytesLeftToSend, sendBuffer->BytesLeft());
		memcpy(sendingWindow + bytesBeingSent, sendBuffer->Read(copyLength), copyLength);
		bytesBeingSent += copyLength;
		bytesLeftToSend -= copyLength;

		if (sendBuffer->BytesLeft() == 0)
		{
			sendBuffer = OutputBuffer::Release(sendBuffer);
			if (sendBuffer == nullptr)
			{
				sendBuffer = sendStack->Pop();
			}
		}
	}

	// We also intend to send a file, so check if we can fill up the TCP window
	if (sendBuffer == nullptr && bytesLeftToSend != 0 && fileBeingSent != nullptr)
	{
		// For HSMCI efficiency, read from the file in multiples of 4 bytes except at the end.
		// This ensures that the second and subsequent chunks can be DMA'd directly into sendingWindow.
		size_t bytesToRead = bytesLeftToSend & (~3);
		if (bytesToRead != 0)
		{
			int bytesRead = fileBeingSent->Read(sendingWindow + bytesBeingSent, bytesToRead);
			if (bytesRead > 0)
			{
				bytesBeingSent += bytesRead;
			}

			if (bytesRead != (int)bytesToRead)
			{
				fileBeingSent->Close();
				fileBeingSent = nullptr;
			}
		}
	}

	if (bytesBeingSent == 0)
	{
		// If we have no data to send, this connection can be closed next time
		if (!cs->persistConnection && nextWrite == nullptr)
		{
			Close();
			return false;
		}

		// We want to send data from another transaction as well, so only free up this one
		cs->sendingTransaction = nextWrite;
		return true;
	}

	// The TCP window has been filled up as much as possible, so send it now. There is no need to check
	// the available space in the SNDBUF queue, because we really write only one TCP window at once.
	writeResult = tcp_write(cs->pcb, sendingWindow, bytesBeingSent, 0);
	if (ERR_IS_FATAL(writeResult))
	{
		reprap.GetPlatform().MessageF(UsbMessage, "Network: Failed to write data in Send (code %d)\n", writeResult);
		tcp_abort(cs->pcb);
		return false;
	}

	outputResult = tcp_output(cs->pcb);
	if (ERR_IS_FATAL(outputResult))
	{
		reprap.GetPlatform().MessageF(UsbMessage, "Network: Failed to output data in Send (code %d)\n", outputResult);
		tcp_abort(cs->pcb);
		return false;
	}

	if (outputResult != ERR_OK && reprap.Debug(moduleNetwork))
	{
		reprap.GetPlatform().MessageF(UsbMessage, "Network: tcp_output resulted in error code %d\n", outputResult);
	}

	// Set LwIP callbacks for ACK and retransmission handling
	tcp_poll(cs->pcb, conn_poll, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
	tcp_sent(cs->pcb, conn_sent);

	// Set all values for the send process
	sendingConnection = cs;
	sendingRetries = 0;
	sendingWindowSize = sentDataOutstanding = bytesBeingSent;
	return false;
}

// This is called by the Webserver to send output data to a client. If keepConnectionAlive is set to false,
// the current connection will be terminated once everything has been sent.
void NetworkTransaction::Commit(bool keepConnectionAlive)
{
	// If the connection has been terminated (e.g. RST received while writing upload data), discard this transaction
	if (!IsConnected() || status == released)
	{
		Discard();
		return;
	}

	// Free buffer holding the incoming data and prepare some values for the sending process
	FreePbuf();
	cs->persistConnection = keepConnectionAlive;
	if (sendBuffer == nullptr)
	{
		sendBuffer = sendStack->Pop();
	}
	status = sending;

	// Unlink the item(s) from the list of ready transactions
	if (keepConnectionAlive)
	{
		// Our connection is still of interest, remove only this transaction from the list
		NetworkTransaction *previous = nullptr;
		for(NetworkTransaction *item = reprap.GetNetwork().readyTransactions; item != nullptr; item = item->next)
		{
			if (item == this)
			{
				if (previous == nullptr)
				{
					reprap.GetNetwork().readyTransactions = next;
				}
				else
				{
					previous->next = next;
				}
				break;
			}
			previous = item;
		}
	}
	else
	{
		// We will close this connection soon, stop receiving data from this PCB
		tcp_recv(cs->pcb, nullptr);

		// Also remove all ready transactions pointing to our ConnectionState
		NetworkTransaction *previous = nullptr, *item = reprap.GetNetwork().readyTransactions;
		while (item != nullptr)
		{
			if (item->cs == cs)
			{
				if (item == this)
				{
					// Only unlink this item
					if (previous == nullptr)
					{
						reprap.GetNetwork().readyTransactions = next;
					}
					else
					{
						previous->next = next;
					}
					item = next;
				}
				else
				{
					// Remove all others
					item->Discard();
					item = (previous == nullptr) ? reprap.GetNetwork().readyTransactions : previous->next;
				}
			}
			else
			{
				previous = item;
				item = item->next;
			}
		}
	}

	// Enqueue this transaction, so it's sent in the right order
	NetworkTransaction *mySendingTransaction = cs->sendingTransaction;
	if (mySendingTransaction == nullptr)
	{
		cs->sendingTransaction = this;
		reprap.GetNetwork().AppendTransaction(&reprap.GetNetwork().writingTransactions, this);
	}
	else
	{
		while (mySendingTransaction->nextWrite != nullptr)
		{
			mySendingTransaction = mySendingTransaction->nextWrite;
		}
		mySendingTransaction->nextWrite = this;
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
//                 an empty item again without payload (i.e. without pbufs)
//
void NetworkTransaction::Defer(DeferralMode mode)
{
	if (mode == DeferralMode::ResetData)
	{
		// Reset the reading pointers and send an ACK
		inputPointer = 0;
		readingPb = pb;
		if (IsConnected() && pb != nullptr && !dataAcknowledged)
		{
			tcp_recved(cs->pcb, pb->tot_len);
			dataAcknowledged = true;
		}
	}
	else if (mode == DeferralMode::DiscardData)
	{
		// Discard the incoming data, because we don't need to process it any more
		FreePbuf();
	}

	status = deferred;

	// Unlink this transaction from the list of ready transactions and append it again
	Network& network = reprap.GetNetwork();
	NetworkTransaction *item, *previous = nullptr;
	for(item = network.readyTransactions; item != nullptr; item = item->next)
	{
		if (item == this)
		{
			if (previous == nullptr)
			{
				network.readyTransactions = next;
			}
			else
			{
				previous->next = next;
			}
			break;
		}
		previous = item;
	}
	network.AppendTransaction(&network.readyTransactions, this);

	// Append all other transactions that are associated to this connection, so that the
	// Webserver gets a chance to deal with all connected clients even while multiple
	// deferred requests are present in the list.
	item = network.readyTransactions;
	previous = nullptr;
	while (item != this)
	{
		if (item->cs == cs)
		{
			NetworkTransaction *nextItem = item->next;
			if (previous == nullptr)
			{
				network.readyTransactions = item->next;
				network.AppendTransaction(&network.readyTransactions, item);
			}
			else
			{
				previous->next = item->next;
				network.AppendTransaction(&network.readyTransactions, item);
			}
			item = nextItem;
		}
		else
		{
			previous = item;
			item = item->next;
		}
	}
}


// This method should be called if we don't want to send data to the client and if we
// don't want to interfere with the connection state. May also be called from ISR!
void NetworkTransaction::Discard()
{
	// Can we do anything?
	if (status == released)
	{
		// No - don't free up released items multiple times
		return;
	}

	// Free up some resources
	FreePbuf();

	if (fileBeingSent != nullptr)
	{
		fileBeingSent->Close();
		fileBeingSent = nullptr;
	}

	OutputBuffer::ReleaseAll(sendBuffer);
	sendStack->ReleaseAll();

	// Unlink this transactions from the list of ready transactions and free it. It is then appended to the list of
	// free transactions because we don't want to risk reusing it when the ethernet ISR processes incoming data
	NetworkTransaction *previous = nullptr;
	for(NetworkTransaction *item = reprap.GetNetwork().readyTransactions; item != nullptr; item = item->next)
	{
		if (item == this)
		{
			if (previous == nullptr)
			{
				reprap.GetNetwork().readyTransactions = next;
			}
			else
			{
				previous->next = next;
			}
			break;
		}
		previous = item;
	}
	reprap.GetNetwork().AppendTransaction(&reprap.GetNetwork().freeTransactions, this);
	bool callDisconnectHandler = (cs != nullptr && status == disconnected);
	status = released;

	// Call disconnect event if this transaction indicates a graceful disconnect and if the connection
	// still persists (may not be the case if a RST packet was received before)
	if (callDisconnectHandler)
	{
		if (reprap.Debug(moduleNetwork))
		{
			reprap.GetPlatform().Message(UsbMessage, "Network: Discard() is handling a graceful disconnect\n");
		}
		reprap.GetNetwork().ConnectionClosed(cs, false);
	}
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

void NetworkTransaction::Close()
{
	tcp_pcb *pcb = cs->pcb;
	tcp_recv(pcb, nullptr);
	closeRequested = true;
}

void NetworkTransaction::FreePbuf()
{
	// See if we have to send an ACK to the client
	if (IsConnected() && pb != nullptr && !dataAcknowledged)
	{
		tcp_recved(cs->pcb, pb->tot_len);
		dataAcknowledged = true;
	}

	// Free all pbufs (pbufs are thread-safe)
	if (pb != nullptr)
	{
		pbuf_free(pb);
		pb = readingPb = nullptr;
	}
}

bool NetworkTransaction::IsConnected() const
{
	return (cs != nullptr && cs->IsConnected());
}

// End
