/*
 * NetworkResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#ifndef SRC_NETWORKING_NETWORKRESPONDER_H_
#define SRC_NETWORKING_NETWORKRESPONDER_H_

#include "RepRapFirmware.h"
#include "RepRap.h"
#include "NetworkDefs.h"
#include "Storage/FileData.h"
#include "NetworkBuffer.h"
#include "OutputMemory.h"

// Forward declarations
class NetworkResponder;
class NetworkInterface;
class Socket;

// Network responder base class
class NetworkResponder
{
public:
	NetworkResponder *GetNext() const { return next; }
	virtual bool Spin() = 0;							// do some work, returning true if we did anything significant
	virtual bool Accept(Socket *s, NetworkProtocol protocol) = 0;	// ask the responder to accept this connection, returns true if it did
	virtual void Terminate(NetworkProtocol protocol, NetworkInterface *interface) = 0;	// terminate the responder if it is serving the specified protocol on the specified interface
	virtual void Diagnostics(MessageType mtype) const = 0;

protected:
	// State machine control. Not all derived classes use all states.
	enum class ResponderState
	{
		free = 0,										// ready to be allocated
		reading,										// ready to receive data
		sending,										// sending data
		uploading,										// uploading a file to SD card

		// HTTP responder additional states
		processingRequest,
		gettingFileInfo,								// getting file info

		// FTP responder additional states
		waitingForPasvPort,
		pasvPortOpened,
		sendingPasvData,
		pasvTransferComplete,

		// Telnet responder additional states
		justConnected,
		authenticating
	};

	NetworkResponder(NetworkResponder *n);

	void Commit(ResponderState nextState = ResponderState::free, bool report = true);
	virtual void SendData();
	virtual void ConnectionLost();

	IPAddress GetRemoteIP() const;
	void ReportOutputBufferExhaustion(const char *sourceFile, int line);

	static Platform& GetPlatform() { return reprap.GetPlatform(); }
	static Network& GetNetwork() { return reprap.GetNetwork(); }

	// General state
	NetworkResponder *next;								// next responder in the list
	ResponderState responderState;						// the current state
	ResponderState stateAfterSending;					// if we are sending, the state to enter when sending is complete
	Socket *skt;
	uint32_t timer;										// a general purpose millisecond timer

	// Buffers for sending responses
	OutputBuffer *outBuf;
	OutputStack outStack;								// not volatile because only one task accesses it
	FileStore *fileBeingSent;
	NetworkBuffer *fileBuffer;
};

#endif /* SRC_NETWORKING_NETWORKRESPONDER_H_ */
