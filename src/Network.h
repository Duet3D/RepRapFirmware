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

#include "OutputMemory.h"

// This class handles the network - typically an Ethernet.

// The size of the TCP output buffer is critical to getting fast load times in the browser.
// If this value is less than the TCP MSS, then Chrome under Windows will delay ack messages by about 120ms,
// which results in very slow page loading. Any value higher than that will cause the TCP packet to be split
// into multiple transmissions, which avoids this behaviour. Using a value of twice the MSS is most efficient because
// each TCP packet will be full.
// Currently we set the MSS (in file network/lwipopts.h) to 1432 which matches the value used by most versions of Windows
// and therefore avoids additional memory use and fragmentation.

const size_t NETWORK_TRANSACTION_COUNT = 24;							// Number of NetworkTransactions to be used for network IO

const uint32_t TCP_WRITE_TIMEOUT = 4000;	 							// Miliseconds to wait for data we have written to be acknowledged
const uint32_t TCP_MAX_SEND_RETRIES = 8;								// How many times can we attempt to write data

const uint8_t MAC_ADDRESS[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };	// Need some sort of default...
const uint8_t IP_ADDRESS[4] = { 192, 168, 1, 10 };
const uint8_t NET_MASK[4] = { 255, 255, 255, 0 };
const uint8_t GATE_WAY[4] = { 192, 168, 1, 1 };

const uint16_t DEFAULT_HTTP_PORT = 80;
const uint16_t FTP_PORT = 21;
const uint16_t TELNET_PORT = 23;

/****************************************************************************************************/

struct tcp_pcb;
struct pbuf;

class NetworkTransaction;

// ConnectionState structure that we use to track TCP connections. It is usually combined with NetworkTransactions.
struct ConnectionState
{
	tcp_pcb *volatile pcb;								// Connection PCB
	uint16_t localPort, remotePort;						// Copy of the local and remote ports, because the PCB may be unavailable
	uint32_t remoteIPAddress;							// Same for the remote IP address
	NetworkTransaction * volatile sendingTransaction;	// NetworkTransaction that is currently sending via this connection
	ConnectionState * volatile next;					// Next ConnectionState in this list
	bool persistConnection;								// Do we expect this connection to stay alive?
	volatile bool isTerminated;							// Will be true if the connection has gone down unexpectedly (TCP RST)

	void Init(tcp_pcb *p);
	uint16_t GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	uint16_t GetRemotePort() const { return remotePort; }
	bool IsConnected() const { return pcb != nullptr; }
	bool IsTerminated() const { return isTerminated; }
	void Terminate();
};

// Assign a status to each NetworkTransaction
enum TransactionStatus
{
	released,
	connected,
	receiving,
	sending,
	disconnected,
	deferred,
	acquired
};

// Start with a class to hold input and output from the network that needs to be responded to.
// This includes changes in the connection state, e.g. connects and disconnects.
class NetworkTransaction
{
	public:
		friend class Network;

		NetworkTransaction(NetworkTransaction* n);
		void Set(pbuf *p, ConnectionState* c, TransactionStatus s);
		TransactionStatus GetStatus() const { return status; }
		bool IsConnected() const;

		bool HasMoreDataToRead() const { return readingPb != nullptr; }
		bool Read(char& b);
		bool ReadBuffer(const char *&buffer, size_t &len);
		void Write(char b);
		void Write(const char* s);
		void Write(StringRef ref);
		void Write(const char* s, size_t len);
		void Write(OutputBuffer *buffer);
		void Write(OutputStack *stack);
		void Printf(const char *fmt, ...);
		void SetFileToWrite(FileStore *file);

		ConnectionState *GetConnection() const { return cs; }
		uint16_t GetLocalPort() const;
		uint32_t GetRemoteIP() const;
		uint16_t GetRemotePort() const;

		void Commit(bool keepConnectionAlive);
		void Defer(bool keepData);
		void Discard();

	private:
		bool CanWrite() const;
		bool Send();
		void Close();
		void FreePbuf();

		ConnectionState* cs;
		NetworkTransaction* volatile next;			// next NetworkTransaction in the list we are in
		NetworkTransaction* volatile nextWrite;		// next NetworkTransaction queued to write to assigned connection
		pbuf *pb, *readingPb;						// received packet queue and a pointer to the pbuf being read from
		size_t inputPointer;						// amount of data already taken from the first packet buffer

		OutputBuffer *sendBuffer;
		OutputStack *sendStack;
		FileStore * volatile fileBeingSent;

		volatile TransactionStatus status;
		volatile bool closeRequested, dataAcknowledged;
};

// The main network class that drives the network.
class Network
{
	public:
		friend class NetworkTransaction;

		Network(Platform* p);
		void Init();
		void Exit() {}
		void Spin();
		void Interrupt();
		void Diagnostics();

		// Deal with LwIP

		void ResetCallback();
		bool ReceiveInput(pbuf *pb, ConnectionState *cs);
		ConnectionState *ConnectionAccepted(tcp_pcb *pcb);
		void ConnectionClosed(ConnectionState* cs, bool closeConnection);
		bool ConnectionClosedGracefully(ConnectionState *cs);

		bool Lock();
		void Unlock();
		bool InLwip() const;

		// Global settings

		const uint8_t *IPAddress() const;
		void SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[]);
		void SetHostname(const char *name);

		void Enable();
		void Disable();
		bool IsEnabled() const { return isEnabled; }

		// Interfaces for the Webserver

		NetworkTransaction *GetTransaction(const ConnectionState *cs = nullptr);

		void OpenDataPort(uint16_t port);
		uint16_t GetDataPort() const;
		void CloseDataPort();

		void SetHttpPort(uint16_t port);
		uint16_t GetHttpPort() const;

		void SaveDataConnection();
		void SaveFTPConnection();
		void SaveTelnetConnection();

		bool AcquireFTPTransaction();
		bool AcquireDataTransaction();
		bool AcquireTelnetTransaction();

	private:

		Platform* platform;
		float longWait;

		void AppendTransaction(NetworkTransaction* volatile * list, NetworkTransaction *r);
		void PrependTransaction(NetworkTransaction* volatile * list, NetworkTransaction *r);
		bool AcquireTransaction(ConnectionState *cs);

		NetworkTransaction * volatile freeTransactions;
		NetworkTransaction * volatile readyTransactions;
		NetworkTransaction * volatile writingTransactions;

		enum { NetworkInactive, NetworkEstablishingLink, NetworkObtainingIP, NetworkActive } state;
		bool isEnabled;
		volatile bool resetCallback;
		char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0

		ConnectionState * volatile dataCs;
		ConnectionState * volatile ftpCs;
		ConnectionState * volatile telnetCs;

		ConnectionState * volatile freeConnections;
};

inline bool NetworkTransaction::IsConnected() const
{
	return (cs != nullptr && cs->IsConnected());
}

inline bool NetworkTransaction::CanWrite() const
{
	return (IsConnected() && status != released);
}

#endif

// vim: ts=4:sw=4
