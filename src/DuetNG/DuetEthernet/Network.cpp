/*
 * Network.cpp
 *
 *  Created on: 13 Dec 2016
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "compiler.h"
#include "Pins.h"
#include "Ethernet3/Ethernet3.h"


void Network::SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[])
{
	memcpy(ipAddress, p_ipAddress, sizeof(ipAddress));
	memcpy(netmask, p_netmask, sizeof(netmask));
	memcpy(gateway, p_gateway, sizeof(gateway));
}

Network::Network(Platform* p) : platform(p), responseCode(0), responseBody(nullptr), responseText(nullptr), responseFile(nullptr),
	httpPort(DEFAULT_HTTP_PORT),
	state(disabled), activated(false)
{
	SetIPAddress(IP_ADDRESS, NET_MASK, GATE_WAY);
	strcpy(hostname, HOSTNAME);
}

void Network::Init()
{
	// Ensure that the chip is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);
	state = disabled;
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void Network::Activate()
{
	activated = true;
	if (state == enabled)
	{
		Start();
	}
}

void Network::Exit()
{
	Stop();
}

void Network::Spin()
{
#if 1
#if 0
	if (state == starting)
	{
		const int rc = Ethernet.begin(platform->MACAddress(), 12000, 5000);		// for now we always use DHCP
		if (rc == 1)
		{
			state = running;
		}
	}
	else if (state == running)
#else
	if (state == starting || state == running)
#endif
	{
		// Check DHCP
		const int rc = Ethernet.maintain();
		if (state == starting && (rc == DHCP_CHECK_RENEW_OK || rc == DHCP_CHECK_REBIND_OK))
		{
			state = running;
		}
	}
#endif
	platform->ClassReport(longWait);
}

void Network::Diagnostics(MessageType mtype)
{

}

void Network::Start()
{
	pinMode(EspResetPin, OUTPUT_LOW);
	delayMicroseconds(550);						// W550 reset pulse must be at least 500us long
	Platform::WriteDigital(EspResetPin, HIGH);	// raise /Reset pin
	delay(55);									// W5500 needs 50ms to start up
	state = starting;
#if 0
	w5500.init();
	w5500.setMACAddress(platform->MACAddress());
#endif
	const int rc = Ethernet.begin(platform->MACAddress(), 12000, 5000);		// for now we always use DHCP
	if (rc == 1)
	{
		state = running;
	}
	else
	{
#if 1
		{
			uint8_t tmp = w5500.readPHYCFGR();
			uint8_t version = w5500.readVERSIONR();
			uint8_t macBuf[6];
			w5500.readSHAR(macBuf);
			platform->MessageF(GENERIC_MESSAGE, "Phy %02x ver %02x Mac %02x:%02x:%02x:%02x:%02x:%02x\n",
					tmp, version, macBuf[0], macBuf[1], macBuf[2], macBuf[3], macBuf[4], macBuf[5]);
		}
#endif
		platform->Message(GENERIC_MESSAGE, "Failed to start Ethernet interface\n");
		Stop();
	}
}

void Network::Stop()
{
	if (state != disabled)
	{
		Ethernet.stop();
		digitalWrite(EspResetPin, LOW);	// put the ESP back into reset
		state = disabled;
	}
}

void Network::Enable()
{
	if (state == disabled)
	{
		state = enabled;
		if (activated)
		{
			Start();
		}
	}
#if 1
	else if (state == starting)
	{
		uint8_t tmp = w5500.readPHYCFGR();
		uint8_t version = w5500.readVERSIONR();
		uint8_t macBuf[6];
		w5500.readSHAR(macBuf);
		platform->MessageF(GENERIC_MESSAGE, "Phy %02x ver %02x Mac %02x:%02x:%02x:%02x:%02x:%02x\n",
				tmp, version, macBuf[0], macBuf[1], macBuf[2], macBuf[3], macBuf[4], macBuf[5]);
	}
#endif
}

void Network::Disable()
{
	if (activated && state != disabled)
	{
		Stop();
		platform->Message(GENERIC_MESSAGE, "WiFi server stopped\n");
	}
}

bool Network::IsEnabled() const
{
	return state != disabled;
}

const uint8_t *Network::GetIPAddress() const
{
	if (state == running)
	{
		static IPAddress ip;
		ip = Ethernet.localIP();
		return ip.GetRawAddress();
	}
	else
	{
		return ipAddress;
	}
}

void Network::SetHttpPort(uint16_t port)
{
	httpPort = port;
}

uint16_t Network::GetHttpPort() const
{
	return httpPort;
}

void Network::SetHostname(const char *name)
{
	size_t i = 0;
	while (*name && i < ARRAY_UPB(hostname))
	{
		char c = *name++;
		if (c >= 'A' && c <= 'Z')
		{
			c += 'a' - 'A';
		}

		if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-') || (c == '_'))
		{
			hostname[i++] = c;
		}
	}

	if (i)
	{
		hostname[i] = 0;
	}
	else
	{
		strcpy(hostname, HOSTNAME);
	}
}

bool Network::Lock()
{
	//TODO
	return true;
}

void Network::Unlock()
{
	//TODO
}

bool Network::InLwip() const
{
	//TODO
	return false;
}

// This is called by the web server to get the next networking transaction.
//
// If cs is NULL, the transaction from the head of readyTransactions will be retrieved.
// If cs is not NULL, the first transaction with the matching connection will be returned.
//
// This method also ensures that the retrieved transaction is moved to the first item of
// readyTransactions, so that a subsequent call with a NULL cs parameter will return exactly
// the same instance.
NetworkTransaction *Network::GetTransaction(const ConnectionState *cs)
{
#if 1
	return nullptr;
#else
	// See if there is any transaction at all
	NetworkTransaction *transaction = readyTransactions;
	if (transaction == nullptr)
	{
		return nullptr;
	}

	// If no specific connection is specified or if the first item already matches the
	// connection we are looking for, just return it
	if (cs == nullptr || transaction->GetConnection() == cs)
	{
		return transaction;
	}

	// We are looking for a specific transaction, but it's not the first item.
	// Search for it and move it to the head of readyTransactions
	NetworkTransaction *previous = transaction;
	for(NetworkTransaction *item = transaction->next; item != nullptr; item = item->next)
	{
		if (item->GetConnection() == cs)
		{
			previous->next = item->next;
			item->next = readyTransactions;
			readyTransactions = item;
			return item;
		}
		previous = item;
	}

	// We failed to find a valid transaction for the given connection
	return nullptr;
#endif
}

void Network::OpenDataPort(uint16_t port)
{
	//TODO
#if 0
	closingDataPort = false;
	tcp_pcb* pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, port);
	ftp_pasv_pcb = tcp_listen(pcb);
	tcp_accept(ftp_pasv_pcb, conn_accept);
#endif
}

uint16_t Network::GetDataPort() const
{
#if 1
	return 0;	//TODO
#else
	return (closingDataPort || (ftp_pasv_pcb == nullptr) ? 0 : ftp_pasv_pcb->local_port);
#endif
}

// Close FTP data port and purge associated PCB
void Network::CloseDataPort()
{
	//TODO
#if 0
	// See if it's already being closed
	if (closingDataPort)
	{
		return;
	}
	closingDataPort = true;

	// Close remote connection of our data port or do it as soon as the last packet has been sent
	if (dataCs != nullptr)
	{
		NetworkTransaction *mySendingTransaction = dataCs->sendingTransaction;
		if (mySendingTransaction != nullptr)
		{
			mySendingTransaction->Close();
			return;
		}
	}

	// We can close it now, so do it here
	if (ftp_pasv_pcb != nullptr)
	{
		tcp_accept(ftp_pasv_pcb, nullptr);
		tcp_close(ftp_pasv_pcb);
		ftp_pasv_pcb = nullptr;
	}
	closingDataPort = false;
#endif
}

// These methods keep track of our connections in case we need to send to one of them
void Network::SaveDataConnection()
{
	//TODO
#if 0
	dataCs = readyTransactions->cs;
#endif
}

void Network::SaveFTPConnection()
{
	//TODO
#if 0
	ftpCs = readyTransactions->cs;
#endif
}

void Network::SaveTelnetConnection()
{
	//TODO
#if 0
	telnetCs = readyTransactions->cs;
#endif
}

bool Network::AcquireFTPTransaction()
{
#if 1
	return false;	//TODO
#else
	return AcquireTransaction(ftpCs);
#endif
}

bool Network::AcquireDataTransaction()
{
#if 1
	return false;	//TODO
#else
	return AcquireTransaction(dataCs);
#endif
}

bool Network::AcquireTelnetTransaction()
{
#if 1
	return false;	//TODO
#else
	return AcquireTransaction(telnetCs);
#endif
}

//***************************************************************************************************

// ConnectionState class

#if 0
void ConnectionState::Init(tcp_pcb *p)
{
	pcb = p;
	localPort = p->local_port;
	remoteIPAddress = p->remote_ip.addr;
	remotePort = p->remote_port;
	next = nullptr;
	sendingTransaction = nullptr;
	persistConnection = true;
	isTerminated = false;
}
#endif

void ConnectionState::Terminate()
{
	//TODO
#if 0
	if (pcb != nullptr)
	{
		tcp_abort(pcb);
	}
#endif
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
//	pb = readingPb = p;
	status = s;
//	inputPointer = 0;
	sendBuffer = nullptr;
	fileBeingSent = nullptr;
	closeRequested = false;
	nextWrite = nullptr;
	dataAcknowledged = false;
}

bool NetworkTransaction::HasMoreDataToRead() const
{
	//TODO
	return false;
}

// Read one char from the NetworkTransaction
bool NetworkTransaction::Read(char& b)
{
#if 1
	return false;
#else
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
#endif
}

// Read data from the NetworkTransaction and return true on success
bool NetworkTransaction::ReadBuffer(const char *&buffer, size_t &len)
{
#if 1
	return false;
#else
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
#endif
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
		file->Close();
	}
}

// Send exactly one TCP window of data and return true when this transaction can be released
bool NetworkTransaction::Send()
{
#if 1
	return true;
#else
	// Free up this transaction if the connection is supposed to be closed
	if (closeRequested)
	{
		reprap.GetNetwork()->ConnectionClosed(cs, true);	// This will release the transaction too
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
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Failed to write data in Send (code %d)\n", writeResult);
		tcp_abort(cs->pcb);
		return false;
	}

	outputResult = tcp_output(cs->pcb);
	if (ERR_IS_FATAL(outputResult))
	{
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: Failed to output data in Send (code %d)\n", outputResult);
		tcp_abort(cs->pcb);
		return false;
	}

	if (outputResult != ERR_OK && reprap.Debug(moduleNetwork))
	{
		reprap.GetPlatform()->MessageF(HOST_MESSAGE, "Network: tcp_output resulted in error code %d\n", outputResult);
	}

	// Set LwIP callbacks for ACK and retransmission handling
	tcp_poll(cs->pcb, conn_poll, TCP_WRITE_TIMEOUT / TCP_SLOW_INTERVAL / TCP_MAX_SEND_RETRIES);
	tcp_sent(cs->pcb, conn_sent);

	// Set all values for the send process
	sendingConnection = cs;
	sendingRetries = 0;
	sendingWindowSize = sentDataOutstanding = bytesBeingSent;
	return false;
#endif
}

// This is called by the Webserver to send output data to a client. If keepConnectionAlive is set to false,
// the current connection will be terminated once everything has been sent.
void NetworkTransaction::Commit(bool keepConnectionAlive)
{
#if 0
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
		for(NetworkTransaction *item = reprap.GetNetwork()->readyTransactions; item != nullptr; item = item->next)
		{
			if (item == this)
			{
				if (previous == nullptr)
				{
					reprap.GetNetwork()->readyTransactions = next;
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
		NetworkTransaction *previous = nullptr, *item = reprap.GetNetwork()->readyTransactions;
		while (item != nullptr)
		{
			if (item->cs == cs)
			{
				if (item == this)
				{
					// Only unlink this item
					if (previous == nullptr)
					{
						reprap.GetNetwork()->readyTransactions = next;
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
					item = (previous == nullptr) ? reprap.GetNetwork()->readyTransactions : previous->next;
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
		reprap.GetNetwork()->AppendTransaction(&reprap.GetNetwork()->writingTransactions, this);
	}
	else
	{
		while (mySendingTransaction->nextWrite != nullptr)
		{
			mySendingTransaction = mySendingTransaction->nextWrite;
		}
		mySendingTransaction->nextWrite = this;
	}
#endif
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
#if 0
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
	Network *network = reprap.GetNetwork();
	NetworkTransaction *item, *previous = nullptr;
	for(item = network->readyTransactions; item != nullptr; item = item->next)
	{
		if (item == this)
		{
			if (previous == nullptr)
			{
				network->readyTransactions = next;
			}
			else
			{
				previous->next = next;
			}
			break;
		}
		previous = item;
	}
	network->AppendTransaction(&network->readyTransactions, this);

	// Append all other transactions that are associated to this connection, so that the
	// Webserver gets a chance to deal with all connected clients even while multiple
	// deferred requests are present in the list.
	item = network->readyTransactions;
	previous = nullptr;
	while (item != this)
	{
		if (item->cs == cs)
		{
			NetworkTransaction *nextItem = item->next;
			if (previous == nullptr)
			{
				network->readyTransactions = item->next;
				network->AppendTransaction(&network->readyTransactions, item);
			}
			else
			{
				previous->next = item->next;
				network->AppendTransaction(&network->readyTransactions, item);
			}
			item = nextItem;
		}
		else
		{
			previous = item;
			item = item->next;
		}
	}
#endif
}


// This method should be called if we don't want to send data to the client and if we
// don't want to interfere with the connection state. May also be called from ISR!
void NetworkTransaction::Discard()
{
#if 0
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
	for(NetworkTransaction *item = reprap.GetNetwork()->readyTransactions; item != nullptr; item = item->next)
	{
		if (item == this)
		{
			if (previous == nullptr)
			{
				reprap.GetNetwork()->readyTransactions = next;
			}
			else
			{
				previous->next = next;
			}
			break;
		}
		previous = item;
	}
	reprap.GetNetwork()->AppendTransaction(&reprap.GetNetwork()->freeTransactions, this);
	bool callDisconnectHandler = (cs != nullptr && status == disconnected);
	status = released;

	// Call disconnect event if this transaction indicates a graceful disconnect and if the connection
	// still persists (may not be the case if a RST packet was received before)
	if (callDisconnectHandler)
	{
		if (reprap.Debug(moduleNetwork))
		{
			reprap.GetPlatform()->Message(HOST_MESSAGE, "Network: Discard() is handling a graceful disconnect\n");
		}
		reprap.GetNetwork()->ConnectionClosed(cs, false);
	}
#endif
}

uint32_t NetworkTransaction::GetRemoteIP() const
{
	return (cs != nullptr) ? cs->GetRemoteIP() : 0;
}

uint16_t NetworkTransaction::GetRemotePort() const
{
	return (cs != nullptr) ? cs->GetRemotePort() : 0;
}

uint16_t NetworkTransaction::GetLocalPort() const
{
	return (cs != nullptr) ? cs->GetLocalPort() : 0;
}

void NetworkTransaction::Close()
{
#if 0
	tcp_pcb *pcb = cs->pcb;
	tcp_recv(pcb, nullptr);
	closeRequested = true;
#endif
}

bool ConnectionState::IsConnected() const
{
	//TODO
	return false;
}

// End
