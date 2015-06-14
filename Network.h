/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <cctype>
#include <cstring>
#include <cstdlib>
#include <climits>

#include "lwipopts.h"

// This class handles the network - typically an Ethernet.

const uint16_t ftpPort = 21;
const uint16_t telnetPort = 23;

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
#define HOSTNAME "duet"


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
	uint32_t GetRemoteIP() const;
	uint16_t GetRemotePort() const;
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
	uint16_t DataLength() const;
	bool Read(char& b);
	bool ReadBuffer(char *&buffer, unsigned int &len);
	void Write(char b);
	void Write(const char* s);
	void Write(StringRef ref);
	void Write(const char* s, size_t len);
	void Printf(const char *fmt, ...);
	bool Send();

	void SetConnectionLost();
	bool LostConnection() const { return cs == NULL || cs->pcb == NULL; }
	ConnectionState *GetConnection() const { return cs; }
	uint32_t GetRemoteIP() const;
	uint16_t GetRemotePort() const;
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
	bool waitingForDataConnection;
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

	void ReadPacket();
	void ReceiveInput(pbuf *pb, ConnectionState *cs);
	void SentPacketAcknowledged(ConnectionState *cs, unsigned int len);
	ConnectionState *ConnectionAccepted(tcp_pcb *pcb);
	void ConnectionClosed(ConnectionState* cs, bool closeConnection);
	void ConnectionClosedGracefully(ConnectionState *cs);

	NetworkTransaction *GetTransaction(const ConnectionState *cs = NULL);
	void SendAndClose(FileStore *f, bool keepConnectionOpen = false);
	void CloseTransaction();
	void WaitForDataConection();

	uint8_t *IPAddress() const;
	void OpenDataPort(uint16_t port);
	uint16_t GetDataPort() const;
	void CloseDataPort();

	void SaveDataConnection();
	void SaveFTPConnection();
	void SaveTelnetConnection();

	bool CanAcquireTransaction();
	bool AcquireFTPTransaction();
	bool AcquireDataTransaction();
	bool AcquireTelnetTransaction();

	Network(Platform* p);
	void Init();
	void Spin();
	void Interrupt();
	void Diagnostics();

	bool Lock();
	void Unlock();
	bool InLwip() const;

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(uint16_t port);
	uint16_t GetHttpPort() const;

	void SetHostname(const char *name);

private:

	Platform* platform;
	float longWait;

	void AppendTransaction(NetworkTransaction* volatile * list, NetworkTransaction *r);
	void PrependTransaction(NetworkTransaction* volatile * list, NetworkTransaction *r);
	bool AcquireTransaction(ConnectionState *cs);

	bool AllocateSendBuffer(SendBuffer *&buffer);
	SendBuffer *ReleaseSendBuffer(SendBuffer *buffer);

	NetworkTransaction * volatile freeTransactions;
	NetworkTransaction * volatile readyTransactions;
	NetworkTransaction * volatile writingTransactions;

	enum { NetworkPreInitializing, NetworkPostInitializing, NetworkInactive, NetworkActive } state;
	bool isEnabled;
	bool volatile readingData;
	char hostname[16];			// limit DHCP hostname to 15 characters + terminating 0

	ConnectionState *dataCs;
	ConnectionState *ftpCs;
	ConnectionState *telnetCs;

	ConnectionState * volatile freeConnections;

	SendBuffer *freeSendBuffers;
};

#endif
