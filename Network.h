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

#define IP_ADDRESS {192, 168, 1, 10} // Need some sort of default...
#define NET_MASK {255, 255, 255, 0}
#define GATE_WAY {192, 168, 1, 1}


/****************************************************************************************************/

struct tcp_pcb;
struct pbuf;
class RequestState;

// HttpState structure that we use to track Http connections. This could be combined with class RequestState.

struct HttpState
{
	tcp_pcb *pcb;				// connection
	RequestState *sendingRs;	// RequestState that is currently sending via this connection
	const char *file;			// pointer to data to send
	uint16_t left;				// amount of data to send
	uint8_t retries;			// number of retries left

	// Methods
	bool SendInProgress() const { return left > 0; }
};

// Start with a class to hold input and output from the network that needs to be responded to.

class RequestState
{
public:
	friend class Network;

	RequestState(RequestState* n);
	void Set(pbuf *p, HttpState* h);
	bool Read(char& b);
	void SentPacketAcknowledged(unsigned int len);
	void Write(char b);
	void Write(const char* s);
	void Printf(const char *fmt, ...);
	bool Send();
	void SetConnectionLost();
	bool LostConnection() const;
	bool IsReady() const { return hs == NULL || hs->sendingRs == NULL; }
	HttpState *GetConnection() const { return hs; }

private:
	void Reset();
	void FreePbuf();

	HttpState* hs;

	RequestState* volatile next;			// next RequestState in the list we are in
	RequestState* nextWrite;				// next RequestState queued to write to this hs
	pbuf *pb;								// linked list of incoming packet buffers
	unsigned int inputPointer;				// amount of data already taken from the first packet buffer

	unsigned int sentDataOutstanding;		// amount of TCP data we have sent that has not been acknowledged
	char outputBuffer[httpOutputBufferSize];
	unsigned int unsentPointer;
	unsigned int outputPointer;
	FileStore *fileBeingSent;
	float lastWriteTime;
	bool persistConnection;
	bool closeRequested;
};

// The main network class that drives the network.

class Network
{
public:

	void ReceiveInput(pbuf *pb, HttpState* h);
	void SentPacketAcknowledged(HttpState *hs, unsigned int len);
	void ConnectionClosing(HttpState* h);
	bool Active() const;
	bool LinkIsUp();
	RequestState *GetRequest(const HttpState *connection);
	void SendAndClose(FileStore *f, bool keepConnectionOpen = false);

	Network();
	void Init();
	void Spin();
	bool InLwip() const { return inLwip; }

private:

	void AppendTransaction(RequestState* volatile * list, RequestState *r);

	RequestState * volatile freeTransactions;
	RequestState * volatile readyTransactions;
	RequestState * volatile writingTransactions;
	bool active;
	uint8_t inLwip;
};

#endif
