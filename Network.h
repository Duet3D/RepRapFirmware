/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

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

const uint16_t tcpOutputBufferCount = 20;					// number of send buffers
const uint16_t tcpOutputBufferSize = 358;					// size of each send buffer (MUST be 1/n-th of TCP_WND)
const uint8_t numConnections = 16;							// number of ConnectionState instances
const uint8_t networkTransactionCount = 24;					// number of NetworkTransactions to be used for network IO
const float writeTimeout = 4.0;	 							// seconds to wait for data we have written to be acknowledged

#define IP_ADDRESS {192, 168, 1, 10} // Need some sort of default...
#define NET_MASK {255, 255, 255, 0}
#define GATE_WAY {192, 168, 1, 1}


/****************************************************************************************************/

struct tcp_pcb;
struct pbuf;

class NetworkTransaction;
class SendBuffer;

// ConnectionState structure that we use to track TCP connections. It is usually combined with NetworkTransactions.
struct ConnectionState
{
	tcp_pcb *pcb;								// connection PCB
	NetworkTransaction *sendingTransaction;		// NetworkTransaction that is currently sending via this connection
	ConnectionState *next;						// next ConnectionState in this list
	bool persistConnection;						// do we expect this connection to stay alive?

	void Init(tcp_pcb *p);
	uint16_t GetLocalPort() const;
};

// Assign a status to each NetworkTransaction
enum TransactionStatus
{
	connected,
	dataReceiving,
	dataSending,
	disconnected
};

// Start with a class to hold input and output from the network that needs to be responded to.
// This includes changes in the connection state, e.g. connects and disconnects.
class NetworkTransaction
{
public:
	friend class Network;

	NetworkTransaction(NetworkTransaction* n) : next(n) { }
	void Set(pbuf *p, ConnectionState* c, TransactionStatus s);
	bool Read(char& b);
	bool ReadBuffer(char *&buffer, unsigned int &len);
	void Write(char b);
	void Write(const char* s);
	void Write(StringRef ref);
	void Write(const char* s, size_t len);
	void Printf(const char *fmt, ...);
	bool Send();

	void SetConnectionLost();
	bool LostConnection() const { return cs == NULL; }
	bool IsReady() const { return cs != NULL && cs->sendingTransaction == NULL; }
	ConnectionState *GetConnection() const { return cs; }
	uint16_t GetLocalPort() const;
	TransactionStatus GetStatus() const { return status; }

private:
	void Close();
	void FreePbuf();

	ConnectionState* cs;
	NetworkTransaction* volatile next;			// next NetworkTransaction in the list we are in
	NetworkTransaction* nextWrite;				// next NetworkTransaction queued to write to assigned connection
	pbuf *pb;									// linked list of incoming packet buffers
	unsigned int bufferLength;					// total length of the packet buffer
	unsigned int inputPointer;					// amount of data already taken from the first packet buffer

	SendBuffer *sendBuffer;
	FileStore *fileBeingSent;

	TransactionStatus status;
	float lastWriteTime;
	bool closeRequested;
};

// This class holds data left to be sent to TCP clients.
class SendBuffer
{
public:
	friend class Network;
	friend class NetworkTransaction;

	SendBuffer(SendBuffer *n) : next(n) { };

private:
	SendBuffer *next;

	uint16_t bytesToWrite;
	char tcpOutputBuffer[tcpOutputBufferSize];
};


// The main network class that drives the network.
class Network
{
public:
	friend class NetworkTransaction;

	void ReceiveInput(pbuf *pb, ConnectionState *cs);
	void SentPacketAcknowledged(ConnectionState *cs, unsigned int len);
	ConnectionState *ConnectionAccepted(tcp_pcb *pcb);
	void ConnectionClosed(ConnectionState* cs, bool closeConnection);
	void ConnectionClosedGracefully(ConnectionState *cs);

	NetworkTransaction *GetTransaction(const ConnectionState *cs = NULL);
	void SendAndClose(FileStore *f, bool keepConnectionOpen = false);
	void CloseTransaction();
	void RepeatTransaction();

	void OpenDataPort(uint16_t port);
	bool CloseDataPort();

	void SaveDataConnection();
	void SaveFTPConnection();
	void SaveTelnetConnection();

	bool CanAcquireTransaction();
	bool AcquireFTPTransaction();
	bool AcquireDataTransaction();
	bool AcquireTelnetTransaction();

	Network();
	void Init();
	void Spin();
	void Interrupt();
	void Diagnostics();

	bool InLwip() const;
	void ReadPacket();

	void Enable();
	void Disable();
	bool IsEnabled() const;

private:
	void AppendTransaction(NetworkTransaction* volatile * list, NetworkTransaction *r);
	void PrependTransaction(NetworkTransaction* volatile * list, NetworkTransaction *r);
	bool AcquireTransaction(ConnectionState *cs);

	bool AllocateSendBuffer(SendBuffer *&buffer);
	SendBuffer *ReleaseSendBuffer(SendBuffer *buffer);

	NetworkTransaction * volatile freeTransactions;
	NetworkTransaction * volatile readyTransactions;
	NetworkTransaction * volatile writingTransactions;

	enum { NetworkInactive, NetworkInitializing, NetworkActive } state;
	bool isEnabled;
	bool volatile readingData;

	ConnectionState *dataCs;
	ConnectionState *ftpCs;
	ConnectionState *telnetCs;

	ConnectionState * volatile freeConnections;

	SendBuffer *freeSendBuffers;
};

#endif
