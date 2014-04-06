/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <malloc.h>
#include <stdlib.h>
#include <limits.h>

// Platform-specific includes

#include "Arduino.h"
#include "ethernet_sam.h"

// This class handles the network - typically an Ethernet.

// The size of the http output buffer is critical to getting fast load times in the browser.
// If this value is less than the TCP MSS, then Chrome under Windows will delay ack messages by about 120ms,
// which results in very slow page loading. Any value higher than that will cause the TCP packet to be split
// into multiple transmissions, which avoids this behaviour. Using a value of twice the MSS is most efficient because
// each TCP packet will be full.
// Currently we set the MSS (in file network/lwipopts.h) to 1432 which matches the value used by most versions of Windows
// and therefore avoids additional memory use and fragmentation.
const unsigned int httpOutputBufferSize = 2 * 1432;

// Start with a ring buffer to hold input from the network that needs to be responded to.

class NetRing
{
public:
	friend class Network;

protected:
	NetRing(NetRing* n);
	void Set(const char* d, int l, void* pc, void* h);
	bool Read(char& b);
	void SentPacketAcknowledged();
	void Write(char b);
	void Write(const char* s);
	bool Close();
	bool TrySendData();
	void SetConnectionLost();
	bool LostConnection() const;

private:
	void Reset();
	void* pcb;
	void* hs;

	NetRing* next;
	const char* inputData;
	int inputLength;
	int inputPointer;
	uint8_t sentPacketsOutstanding;		// count of TCP packets we have sent that have not been acknowledged
	char outputBuffer[httpOutputBufferSize];
	int outputPointer;
	bool closePending;
	FileStore *fileBeingSent;
	float closeRequestedTime;
};

// The main network class that drives the network.

class Network
{
public:

	void ReceiveInput(const char* data, int length, void* pc, void* h);
	void InputBufferReleased(void *hs, void* pb);
	void SentPacketAcknowledged(void *hs);
	void ConnectionError(void* h);
	bool Active() const;
	bool LinkIsUp();
	bool Read(char& b);
	void Write(char b);
	void Write(const char* s);
	void SendAndClose(FileStore *f);
	bool HaveData() const;

	Network();
	void Init();
	void Spin();

private:

	void AppendTransaction(NetRing** list, NetRing *r);

	NetRing *freeTransactions;
	NetRing *readyTransactions;
	NetRing *writingTransactions;
	NetRing *closingTransactions;
	bool active;
};

#endif
