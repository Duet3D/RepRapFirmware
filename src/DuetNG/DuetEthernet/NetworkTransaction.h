/*
 * NetworkTransaction.h
 *
 *  Created on: 23 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_NETWORKTRANSACTION_H_
#define SRC_DUETNG_DUETETHERNET_NETWORKTRANSACTION_H_

#include <cstdint>
#include <cstddef>
#include "Libraries/General/StringRef.h"
#include "OutputMemory.h"
#include "Storage/FileStore.h"

typedef uint8_t SocketNumber;

const SocketNumber NoSocket = 255;

// Network buffer class. These buffers are 2K long so that they can accept as much data as the W5500 can provide in one go.
class NetworkBuffer
{
public:
	friend class NetworkTransaction;

	NetworkBuffer();

	// Release this buffer and return the next one in the chain
	NetworkBuffer *Release();

	// Read 1 character, returning true of successful, false if no data left
	bool Read(char& b);

	// Read some data
	bool ReadBuffer(const char *&buffer, size_t &len);

	// Return the amount of data available, not including continuation buffers
	size_t Remaining() const;

	// Return the amount of data available, including continuation buffers
	size_t TotalRemaining() const;

private:
	NetworkBuffer *next;
	size_t dataLength;
	size_t readPointer;
	uint8_t data[2048];
};

class NetworkTransaction;

// ConnectionState structure that we use to track TCP connections. It is usually combined with NetworkTransactions.
class ConnectionState
{
public:
	void Init(SocketNumber s);
	uint16_t GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	uint16_t GetRemotePort() const { return remotePort; }
	bool IsConnected() const; // { return pcb != nullptr; }
	bool IsTerminated() const { return isTerminated; }
	void Terminate();

private:
	uint16_t localPort, remotePort;						// Copy of the local and remote ports, because the PCB may be unavailable
	uint32_t remoteIPAddress;							// Same for the remote IP address
	NetworkTransaction * volatile sendingTransaction;	// NetworkTransaction that is currently sending via this connection
	ConnectionState * volatile next;					// Next ConnectionState in this list
	bool persistConnection;								// Do we expect this connection to stay alive?
	bool isTerminated;									// Will be true if the connection has gone down unexpectedly (TCP RST)
	SocketNumber socketNum;								// The W5500 socket number we are using
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
//	void Set(pbuf *p, ConnectionState* c, TransactionStatus s);
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

	bool Send();
	void Commit(bool keepConnectionAlive);
	void Defer(DeferralMode mode);
	void Discard();

	NetworkTransaction *GetNext() const { return next; }
	NetworkTransaction *GetNextWrite() const { return nextWrite; }

private:
	bool CanWrite() const;
	void Close();

	ConnectionState* cs;
	NetworkTransaction* next;					// next NetworkTransaction in the list we are in
	NetworkTransaction* nextWrite;				// next NetworkTransaction queued to write to assigned connection
	NetworkBuffer *pb, *readingPb;				// received packet queue and a pointer to the pbuf being read from
//	size_t inputPointer;						// amount of data already taken from the first packet buffer

	OutputBuffer *sendBuffer;
	OutputStack *sendStack;
	FileStore * volatile fileBeingSent;

	volatile TransactionStatus status;
	volatile bool closeRequested, dataAcknowledged;
};

inline bool NetworkTransaction::IsConnected() const
{
	return (cs != nullptr && cs->IsConnected());
}

inline bool NetworkTransaction::CanWrite() const
{
	return (IsConnected() && status != released);
}

#endif /* SRC_DUETNG_DUETETHERNET_NETWORKTRANSACTION_H_ */
