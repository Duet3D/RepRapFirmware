/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <cctype>
#include <cstring>
#include <malloc.h>
#include <cstdlib>
#include <climits>

// This class handles the network - typically an Ethernet.

// The size of the http output buffer is critical to getting fast load times in the browser.
// If this value is less than the TCP MSS, then Chrome under Windows will delay ack messages by about 120ms,
// which results in very slow page loading. Any value higher than that will cause the TCP packet to be split
// into multiple transmissions, which avoids this behaviour. Using a value of twice the MSS is most efficient because
// each TCP packet will be full.
// Currently we set the MSS (in file network/lwipopts.h) to 1432 which matches the value used by most versions of Windows
// and therefore avoids additional memory use and fragmentation.

const unsigned int httpOutputBufferSize = 2 * 1432;

const float clientCloseDelay = 0.002;	 	// seconds to wait after serving a page

#define IP_ADDRESS {192, 168, 1, 10} // Need some sort of default...
#define NET_MASK {255, 255, 255, 0}
#define GATE_WAY {192, 168, 1, 1}


/****************************************************************************************************/

// HttpState structure that we use to track Http connections. This could be combined with class RequestState.

struct HttpState
{
	// Receive fields
	struct pbuf *pb;

	// Transmit fields
	char *file;
	uint16_t left;
	uint8_t retries;

	bool SendInProgress() const { return left > 0; }
};

// Start with a class to hold input and output from the network that needs to be responded to.

class RequestState
{
public:
	friend class Network;

	RequestState(RequestState* n);
	void Set(const char* d, int l, void* pc, HttpState* h);
	bool Read(char& b);
	void SentPacketAcknowledged();
	void Write(char b);
	void Write(const char* s);
	void Printf(const char *fmt, ...);
	void Close();
	bool TrySendData();
	void SetConnectionLost();
	bool LostConnection() const;

private:
	void Reset();
	void* pcb;
	HttpState* hs;

	RequestState* volatile next;
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

	void ReceiveInput(const char* data, int length, void* pc, HttpState* h);
	void InputBufferReleased(HttpState *hs, void* pb);
	void SentPacketAcknowledged(HttpState *hs);
	void ConnectionError(HttpState* h);
	bool Active() const;
	bool LinkIsUp();
	RequestState *GetRequest() const { return readyTransactions; }
	void SendAndClose(FileStore *f);
	bool HaveData() const;

	Network();
	void Init();
	void Spin();

private:

	void AppendTransaction(RequestState* volatile * list, RequestState *r);
	RequestState *FindHs(RequestState* const volatile * list, HttpState *hs);

	RequestState * volatile freeTransactions;
	RequestState * volatile readyTransactions;
	RequestState * volatile writingTransactions;
	RequestState * volatile closingTransactions;
	bool active;
};

// Webserver calls this to read bytes that have come in from the network.
// Inlined to improve upload speed.

inline bool RequestState::Read(char& b)
{
	if (LostConnection() || inputPointer >= inputLength)
	{
		return false;
	}

	b = inputData[inputPointer++];
	return true;
}

#endif
