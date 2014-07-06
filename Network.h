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

#include "lwipopts.h"

// This class handles the network - typically an Ethernet.

// The size of the TCP output buffer is critical to getting fast load times in the browser.
// If this value is less than the TCP MSS, then Chrome under Windows will delay ack messages by about 120ms,
// which results in very slow page loading. Any value higher than that will cause the TCP packet to be split
// into multiple transmissions, which avoids this behaviour. Using a value of twice the MSS is most efficient because
// each TCP packet will be full.
// Currently we set the MSS (in file network/lwipopts.h) to 1432 which matches the value used by most versions of Windows
// and therefore avoids additional memory use and fragmentation.

const unsigned int tcpOutputBufferCount = MEMP_NUM_TCP_PCB;		// number of send buffers
const unsigned int tcpOutputBufferSize = 2 * 1432;				// size of each send buffer

#define IP_ADDRESS {192, 168, 1, 10} // Need some sort of default...
#define NET_MASK {255, 255, 255, 0}
#define GATE_WAY {192, 168, 1, 1}


/****************************************************************************************************/

struct tcp_pcb;
struct pbuf;

class RequestState;
class SendBuffer;

// ConnectionState structure that we use to track TCP connections. This could be combined with class RequestState.
struct ConnectionState
{
	tcp_pcb *pcb;				// connection PCB
	RequestState *sendingRs;	// RequestState that is currently sending via this connection
	const char *file;			// pointer to data to send
	uint16_t left;				// amount of data to send
	uint8_t retries;			// number of retries left

	// Methods
	bool SendInProgress() const { return left > 0; }
	uint16_t GetLocalPort() const;
};

// Assign a status to each RequestState
enum RequestStatus
{
	connected,
	dataReceiving,
	dataSending,
	disconnected
};

// Start with a class to hold input and output from the network that needs to be responded to.
class RequestState
{
public:
	friend class Network;

	RequestState(RequestState* n);
	void Set(pbuf *p, ConnectionState* c, RequestStatus s);
	bool Read(char& b);
	bool ReadBuffer(char *&buffer, unsigned int &len);
	void SentPacketAcknowledged(unsigned int len);
	void Write(char b);
	void Write(const char* s);
	void Printf(const char *fmt, ...);
	bool Send();

	void SetConnectionLost();
	bool LostConnection() const { return cs == NULL; }
	bool IsReady() const { return cs != NULL && cs->sendingRs == NULL; }
	ConnectionState *GetConnection() const { return cs; }
	uint16_t GetLocalPort() const;
	RequestStatus GetStatus() const { return status; }

private:
	void Close();
	void FreePbuf();

	ConnectionState* cs;
	RequestState* volatile next;			// next RequestState in the list we are in
	RequestState* nextWrite;				// next RequestState queued to write to this cs
	pbuf *pb;								// linked list of incoming packet buffers
	unsigned int bufferLength;				// total length of the packet buffer
	unsigned int inputPointer;				// amount of data already taken from the first packet buffer

	unsigned int sentDataOutstanding;		// amount of TCP data we have sent that has not been acknowledged
	SendBuffer *sendBuffer;
	char *outputBuffer;
	unsigned int unsentPointer;
	unsigned int outputPointer;
	FileStore *fileBeingSent;

	RequestStatus status;
	float lastWriteTime;
	bool persistConnection;
	bool closeRequested;
};

class SendBuffer
{
	public:
		friend class Network;
		friend class RequestState;

		SendBuffer(SendBuffer *n);

	private:
		char tcpOutputBuffer[tcpOutputBufferSize];
		SendBuffer* volatile next;
};


// The main network class that drives the network.
class Network
{
public:
	friend class RequestState;

	void ReceiveInput(pbuf *pb, ConnectionState *cs);
	void SentPacketAcknowledged(ConnectionState *cs, unsigned int len);
	void ConnectionAccepted(ConnectionState *cs);
	void ConnectionClosed(ConnectionState* cs);
	void ConnectionClosedGracefully(ConnectionState *cs);

	RequestState *GetRequest(const ConnectionState *cs = NULL);
	void SendAndClose(FileStore *f, bool keepConnectionOpen = false);
	void CloseRequest();
	void RepeatRequest();

	void OpenDataPort(uint16_t port);
	bool CloseDataPort();

	void SaveDataConnection();
	void SaveFTPConnection();
	void SaveTelnetConnection();
	bool MakeDataRequest();
	bool MakeFTPRequest();
	bool MakeTelnetRequest();

	Network();
	void Init();
	void Spin();

	bool InLwip() const { return inLwip; }

private:

	void AppendTransaction(RequestState* volatile * list, RequestState *r);
	void PrependTransaction(RequestState* volatile * list, RequestState *r);

	bool AllocateSendBuffer(SendBuffer *&buffer);
	void FreeSendBuffer(SendBuffer *buffer);

	RequestState * volatile freeTransactions;
	RequestState * volatile readyTransactions;
	RequestState * volatile writingTransactions;

	enum { NetworkInactive, NetworkInitializing, NetworkActive } state;
	uint8_t inLwip;

	ConnectionState *dataCs;
	ConnectionState *ftpCs;
	ConnectionState *telnetCs;

	SendBuffer * volatile sendBuffer;
};

#endif
