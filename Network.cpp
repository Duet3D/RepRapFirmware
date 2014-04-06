/****************************************************************************************************

 RepRapFirmware - Network: RepRapPro Ormerod with Arduino Due controller

  Separated out from Platform.cpp by dc42, 2014-04-05

 ****************************************************************************************************/

#include "RepRapFirmware.h"
#include "DueFlashStorage.h"

const uint8_t windowedSendPackets = 2;


//***************************************************************************************************

// Network/Ethernet class

// C calls to interface with LWIP (http://savannah.nongnu.org/projects/lwip/)
// These are implemented in, and called from, a modified version of httpd.c
// in the network directory.

extern "C"
{

// Transmit data to the Network

void RepRapNetworkSendOutput(char* data, int length, void* pcb, void* hs);

// Ask whether it is OK to send
int RepRapNetworkCanSend(void* hs);

void RepRapNetworkConnectionError(void* h)
{
	reprap.GetPlatform()->GetNetwork()->ConnectionError(h);
}

// Called to put out a message via the RepRap firmware.

void RepRapNetworkMessage(const char* s)
{
	reprap.GetPlatform()->Message(HOST_MESSAGE, s);
}

// Called to push data into the RepRap firmware.

void RepRapNetworkReceiveInput(const char* data, int length, void* pcb, void* hs)
{
	reprap.GetPlatform()->GetNetwork()->ReceiveInput(data, length, pcb, hs);
}

// Called when transmission of outgoing data is complete to allow
// the RepRap firmware to write more.

void RepRapNetworkSentPacketAcknowledged(void *hs)
{
	reprap.GetPlatform()->GetNetwork()->SentPacketAcknowledged(hs);
}

}	// extern "C"

Network::Network()
{
	active = false;
	ethPinsInit();

	//ResetEther();

	// Construct the free list buffer

	freeTransactions = NULL;
	readyTransactions = NULL;
	writingTransactions = NULL;
	closingTransactions = NULL;
	for (int8_t i = 0; i < HTTP_STATE_SIZE; i++)
	{
		freeTransactions = new NetRing(freeTransactions);
	}
}

void Network::AppendTransaction(NetRing** list, NetRing *r)
{
	r->next = NULL;
	while (*list != NULL)
	{
		list = &((*list)->next);
	}
	*list = r;
}

void Network::Init()
{
	init_ethernet(reprap.GetPlatform()->IPAddress(), reprap.GetPlatform()->NetMask(), reprap.GetPlatform()->GateWay());
	active = true;
}

void Network::Spin()
{
//	debugPrintf("NetSpinEnter\n");
	if (active)
	{
		ethernet_task();			// keep the Ethernet running

		// See if we can send anything
		NetRing* r = writingTransactions;
		if (r != NULL)
		{
			bool doClose = r->TrySendData();	// we must leave r on the list for now because of possible callback to release the input buffer
			writingTransactions = r->next;
			if (doClose)
			{
				r->closeRequestedTime = reprap.GetPlatform()->Time();
				AppendTransaction(&closingTransactions, r);
			}
			else
			{
				AppendTransaction(&writingTransactions, r);
			}
		}

		// See if we can close any connections
		r = closingTransactions;
		if (r != NULL)
		{
			if (reprap.GetPlatform()->Time() - r->closeRequestedTime >= CLIENT_CLOSE_DELAY)
			{
				r->Close();
				closingTransactions = r->next;
				AppendTransaction(&freeTransactions, r);
			}
		}
	}
//	debugPrintf("NetSpinExit\n");
}

bool Network::HaveData() const
{
	return active && readyTransactions != NULL;
}

bool Network::Read(char& b)
{
	if (readyTransactions != NULL)
	{
		return readyTransactions->Read(b);
	}
	return false;
}

void Network::Write(char b)
{
	if (readyTransactions != NULL)
	{
		readyTransactions->Write(b);
	}
}

void Network::Write(const char* s)
{
	if (readyTransactions != NULL)
	{
		readyTransactions->Write(s);
	}
}

void Network::SentPacketAcknowledged(void *hs)
{
	NetRing *r = writingTransactions;
	while (r != NULL && r->hs != hs)
	{
		r = r->next;
	}
	if (r != NULL)
	{
		r->SentPacketAcknowledged();
		return;
	}

	r = closingTransactions;
	while (r != NULL && r->hs != hs)
	{
		r = r->next;
	}
	if (r != NULL)
	{
		r->SentPacketAcknowledged();
		return;
	}
	debugPrintf("Network SentPacketAcknowledged: didn't find hs=%08x\n", (unsigned int)hs);
}

void Network::ConnectionError(void* hs)
{
	// h points to an http state block that the caller is about to release, so we need to stop referring to it.
	debugPrintf("Network: ConnectionError\n");

	// See if it's a ready transaction
	NetRing* r = readyTransactions;
	while (r != NULL && r->hs == hs)
	{
		r = r->next;
	}
	if (r != NULL)
	{
		r->SetConnectionLost();
		return;
	}

	// See if we were sending it
	r = writingTransactions;
	while (r != NULL && r->hs == hs)
	{
		r = r->next;
	}
	if (r != NULL)
	{
		r->SetConnectionLost();
		return;
	}

	// See if we were closing it
	r = closingTransactions;
	while (r != NULL && r->hs == hs)
	{
		r = r->next;
	}
	if (r != NULL)
	{
		r->SetConnectionLost();
		return;
	}

	// Else we didn't identify the transaction - maybe we already asked to close it
	debugPrintf("Network ConnectionError: didn't find hs=%08x\n", (unsigned int)hs);
}

void Network::ReceiveInput(const char* data, int length, void* pcb, void* hs)
{
	NetRing* r = freeTransactions;
	if (r == NULL)
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::ReceiveInput() - no free transactions!\n");
		return;
	}

	freeTransactions = r->next;

	r->Set(data, length, pcb, hs);
	AppendTransaction(&readyTransactions, r);
//	debugPrintf("Network - input received\n");
}

// Send the output data we already have, optionally with a file appended, then close the connection.
// The file may be too large for our buffer, so we may have to send it in multiple transactions.
void Network::SendAndClose(FileStore *f)
{
	NetRing *r = readyTransactions;
	if (r != NULL)
	{
		readyTransactions = r->next;
		if (r->LostConnection())
		{
			if (f != NULL)
			{
				f->Close();
			}
			AppendTransaction(&freeTransactions, r);
			debugPrintf("Conn lost before send\n");
		}
		else
		{
			r->fileBeingSent = f;
			AppendTransaction(&writingTransactions, r);
//debug
//			r->outputBuffer[r->outputPointer] = 0;
//			debugPrintf("Transaction queued for writing to network, file=%c, data=%s\n", (f ? 'Y' : 'N'), r->outputBuffer);
		}
	}
}

//queries the PHY for link status, true = link is up, false, link is down or there is some other error
bool Network::LinkIsUp()
{
	return status_link_up();
}

bool Network::Active() const
{
	return active;
}


// NetRing class members
NetRing::NetRing(NetRing* n) : next(n)
{
}

void NetRing::Set(const char* d, int l, void* pc, void* h)
{
	pcb = pc;
	hs = h;
	inputData = d;
	inputLength = l;
	inputPointer = 0;
	outputPointer = 0;
	sentPacketsOutstanding = 0;
	fileBeingSent = NULL;
}

// Webserver calls this to read bytes that have come in from the network

bool NetRing::Read(char& b)
{
	if (LostConnection() || inputPointer >= inputLength)
	{
		return false;
	}

	b = inputData[inputPointer];
	inputPointer++;
	return true;
}

// Webserver calls this to write bytes that need to go out to the network

void NetRing::Write(char b)
{
	if (LostConnection()) return;

	if (outputPointer >= ARRAY_SIZE(outputBuffer))
	{
		reprap.GetPlatform()->Message(HOST_MESSAGE, "Network::Write(char b) - Output buffer overflow! \n");
		return;
	}

	// Add the byte to the buffer

	outputBuffer[outputPointer] = b;
	outputPointer++;
}

// This is not called for data, only for internally-
// generated short strings at the start of a transmission,
// so it should never overflow the buffer (which is checked
// anyway).

void NetRing::Write(const char* s)
{
	while (*s)
	{
		Write(*s++);
	}
}

// Send some data if we can, returning true if all data has been sent
bool NetRing::TrySendData()
{
	if (LostConnection())
	{
		return true;
	}

	if (!RepRapNetworkCanSend(hs))
	{
//		debugPrintf("Send busy\n");
		return false;
	}

	if (sentPacketsOutstanding >= windowedSendPackets)
	{
//		debugPrintf("Awaiting ack\n");
		return false;		// still waiting for earlier packets to be acknowledged
	}

	if (fileBeingSent != NULL)
	{
		while (outputPointer < ARRAY_SIZE(outputBuffer))
		{
			bool ok = fileBeingSent->Read(outputBuffer[outputPointer]);
			if (!ok)
			{
				fileBeingSent->Close();
				fileBeingSent = NULL;
				break;
			}
			++outputPointer;
		}
//		debugPrintf("Read from file\n");
	}

	if (outputPointer == 0)
	{
//		debugPrintf("All data sent\n");
		return true;
//		return sentPacketsOutstanding == 0;		// no more data to send, so tell caller to close file if all packets acknowledged
	}
	else
	{
//		debugPrintf("Sending data, hs=%08x,  length=%d\n", (unsigned int)hs, outputPointer);
		++sentPacketsOutstanding;
		RepRapNetworkSendOutput(outputBuffer, outputPointer, pcb, hs);
		outputPointer = 0;
		return false;
	}
}

void NetRing::SentPacketAcknowledged()
{
	if (sentPacketsOutstanding != 0)
	{
		--sentPacketsOutstanding;
	}
}

// Close this connection. Return true if it really is closed, false if it needs to go in the deferred close list.
bool NetRing::Close()
{
	if (LostConnection())
	{
		return true;
	}

//	debugPrintf("Closing connection hs=%08x\n", (unsigned int)hs);
	RepRapNetworkSendOutput((char*) NULL, 0, pcb, hs);
	return true;		// try not using deferred close for now
}

void NetRing::SetConnectionLost()
{
	hs = NULL;
}

bool NetRing::LostConnection() const
{
	return hs == NULL;
}


// End
