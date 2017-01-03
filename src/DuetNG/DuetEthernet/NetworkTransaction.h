/*
 * NetworkTransaction.h
 *
 *  Created on: 23 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_NETWORKTRANSACTION_H_
#define SRC_DUETNG_DUETETHERNET_NETWORKTRANSACTION_H_

#include <NetworkDefs.h>
#include <cstdint>
#include <cstddef>
#include "Libraries/General/StringRef.h"
#include "OutputMemory.h"
#include "Storage/FileStore.h"
#include "NetworkBuffer.h"

// Assign a status to each NetworkTransaction
enum class TransactionStatus
{
	released,
	connected,
	receiving,
	sending,
	disconnected,
	deferred,
	acquired,
	finished
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
	NetworkTransaction(NetworkTransaction* n);
	void Set(Socket *skt, TransactionStatus s);
	TransactionStatus GetStatus() const { return status; }
	bool IsConnected() const;
	bool IsSending() const;
	bool CloseAfterSending() const { return closeAfterSending; }

	bool HasMoreDataToRead() const;
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

	Connection GetConnection() const { return cs; }
	Port GetLocalPort() const;
	uint32_t GetRemoteIP() const;
	Port GetRemotePort() const;

	const uint8_t *GetDataToSend(size_t& length);
	void Commit(bool keepConnectionAlive);
	void Defer(DeferralMode mode);
	void Discard();
	NetworkTransaction *Release();

	static void AllocateTransactions(unsigned int number);
	static NetworkTransaction *Allocate();

private:
	bool CanWrite() const;

	static NetworkTransaction *freelist;

	Socket* cs;									// the network socket that this transaction came from
	NetworkTransaction* next;					// next NetworkTransaction in the list we are in
	NetworkBuffer *fileBuffer;					// buffer holding the file chunk we are writing

	OutputBuffer *sendBuffer;
	OutputStack *sendStack;
	FileStore *fileBeingSent;

	TransactionStatus status;
	bool closeAfterSending;
	bool closeRequested;
};

#endif /* SRC_DUETNG_DUETETHERNET_NETWORKTRANSACTION_H_ */
