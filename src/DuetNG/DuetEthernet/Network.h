/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <cstdint>
#include <cctype>
#include <cstring>
#include <cstdlib>

#include "MessageType.h"

const uint8_t MAC_ADDRESS[6] = { 0xBE, 0xEF, 0xDE, 0xAD, 0xFE, 0xED };	// Need some sort of default...
const uint8_t IP_ADDRESS[4] = { 192, 168, 1, 10 };				// Need some sort of default...
const uint8_t NET_MASK[4] = { 255, 255, 255, 0 };
const uint8_t GATE_WAY[4] = { 192, 168, 1, 1 };

const uint16_t DEFAULT_HTTP_PORT = 80;
const uint16_t FTP_PORT = 21;
const uint16_t TELNET_PORT = 23;

class Platform;

struct tcp_pcb;
struct pbuf;

class NetworkTransaction;

// ConnectionState structure that we use to track TCP connections. It is usually combined with NetworkTransactions.
struct ConnectionState
{
//	tcp_pcb *volatile pcb;								// Connection PCB
	uint16_t localPort, remotePort;						// Copy of the local and remote ports, because the PCB may be unavailable
	uint32_t remoteIPAddress;							// Same for the remote IP address
	NetworkTransaction * volatile sendingTransaction;	// NetworkTransaction that is currently sending via this connection
	ConnectionState * volatile next;					// Next ConnectionState in this list
	bool persistConnection;								// Do we expect this connection to stay alive?
	volatile bool isTerminated;							// Will be true if the connection has gone down unexpectedly (TCP RST)

//	void Init(tcp_pcb *p);
	uint16_t GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	uint16_t GetRemotePort() const { return remotePort; }
	bool IsConnected() const; // { return pcb != nullptr; }
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

// How is a deferred request supposed to be handled?
enum class DeferralMode
{
	DeferOnly,			// don't change anything, because we want to read more of it next time
	ResetData,			// keep the data and reset all reading pointers allowing us to process it again
	DiscardData			// discard all incoming data and re-enqueue the empty transaction
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

		bool HasMoreDataToRead() const; // { return readingPb != nullptr; }
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
		void Defer(DeferralMode mode);
		void Discard();

	private:
		bool CanWrite() const;
		bool Send();
		void Close();

		ConnectionState* cs;
		NetworkTransaction* volatile next;			// next NetworkTransaction in the list we are in
		NetworkTransaction* volatile nextWrite;		// next NetworkTransaction queued to write to assigned connection
//		pbuf *pb, *readingPb;						// received packet queue and a pointer to the pbuf being read from
//		size_t inputPointer;						// amount of data already taken from the first packet buffer

		OutputBuffer *sendBuffer;
		OutputStack *sendStack;
		FileStore * volatile fileBeingSent;

		volatile TransactionStatus status;
		volatile bool closeRequested, dataAcknowledged;
};

// The main network class that drives the network.
class Network
{
	enum NetworkState
	{
		disabled,					// WiFi not active
		enabled,					// WiFi enabled but not started yet
		starting,					// starting up (waiting for initialisation)
		running
//		idle,						// nothing happening
//		receivePending,				// we have asserted TransferReady and await completion of a receive-only transaction
//		sendReceivePending,			// we have asserted TransferReady and await completion of a transmit/receive
//		transferDone,				// transfer completed but receive DMA fifo may not have been flushed yet
//		processing,					// a transaction has been completed but we haven't released the input buffer yet
//		sending						// a transaction has been completed and we are sending the response
	};
public:
	const uint8_t *GetIPAddress() const;
	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);

	Network(Platform* p);
	void Init();
	void Activate();
	void Exit();
	void Spin();
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	bool Lock();
	void Unlock();
	bool InLwip() const;

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(uint16_t port);
	uint16_t GetHttpPort() const;

	void SetHostname(const char *name);

	// Interfaces for the Webserver

	NetworkTransaction *GetTransaction(const ConnectionState *cs = nullptr);

	void OpenDataPort(uint16_t port);
	uint16_t GetDataPort() const;
	void CloseDataPort();

	void SaveDataConnection();
	void SaveFTPConnection();
	void SaveTelnetConnection();

	bool AcquireFTPTransaction();
	bool AcquireDataTransaction();
	bool AcquireTelnetTransaction();

private:
	void SetupSpi();

	Platform *platform;

//	uint32_t responseIp;
	uint32_t responseCode;
//	uint32_t responseFragment;
	OutputBuffer *responseBody;
	const char* responseText;
	FileStore *responseFile;
//	uint32_t responseFileBytes;

    float longWait;

    uint16_t httpPort;
	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0

    NetworkState state;
    bool activated;
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
