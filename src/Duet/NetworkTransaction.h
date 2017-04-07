/*
 * NetworkTransaction.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUET_NETWORKTRANSACTION_H_
#define SRC_DUET_NETWORKTRANSACTION_H_

#include <NetworkDefs.h>
#include "RepRapFirmware.h"

class pbuf;

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
	void Defer(DeferralMode mode);
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

inline bool NetworkTransaction::CanWrite() const
{
	return (IsConnected() && status != released);
}

#endif /* SRC_DUET_NETWORKTRANSACTION_H_ */
