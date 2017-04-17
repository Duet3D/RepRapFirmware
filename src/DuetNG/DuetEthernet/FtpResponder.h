/*
 * FtpResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_FTPRESPONDER_H_
#define SRC_DUETNG_DUETETHERNET_FTPRESPONDER_H_

#include "NetworkResponder.h"

class FtpResponder : public NetworkResponder
{
public:
	FtpResponder(NetworkResponder *n);
	bool Spin() override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, Protocol protocol) override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(Protocol protocol) override;			// terminate the responder if it is serving the specified protocol

	void ConnectionLost() override;
	void Diagnostics(MessageType mtype);

private:
	static const size_t ftpMessageLength = 128;			// maximum line length for incoming FTP commands
	static const uint32_t ftpPasvPortTimeout = 10000;	// maximum time to wait for an FTP data connection in milliseconds

	Socket *dataSocket;
	Port passivePort;

	char clientMessage[ftpMessageLength];
	size_t clientPointer;

	char filename[FILENAME_LENGTH];
	char currentDir[FILENAME_LENGTH];
};

#endif /* SRC_DUETNG_DUETETHERNET_FTPRESPONDER_H_ */
