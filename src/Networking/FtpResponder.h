/*
 * FtpResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Authors: David and Christian
 */

#ifndef SRC_NETWORKING_FTPRESPONDER_H_
#define SRC_NETWORKING_FTPRESPONDER_H_

#include "UploadingNetworkResponder.h"

class FtpResponder : public UploadingNetworkResponder
{
public:
	FtpResponder(NetworkResponder *n);
	bool Spin() override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, NetworkProtocol protocol) override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(NetworkProtocol protocol) override;			// terminate the responder if it is serving the specified protocol

	void Diagnostics(MessageType mtype) const override;

protected:
	static const size_t ftpMessageLength = 128;			// maximum line length for incoming FTP commands
	static const uint32_t ftpPasvPortTimeout = 10000;	// maximum time to wait for an FTP data connection in milliseconds

	Socket *dataSocket;
	Port passivePort;
	uint32_t passivePortOpenTime;
	OutputBuffer *dataBuf;

	void ConnectionLost() override;
	void SendData() override;

	bool sendError;
	void SendPassiveData();

	void DoUpload();

	bool ReadData();
	void CharFromClient(char c);
	void ProcessLine();
	const char *GetParameter(const char *after) const;	// return the parameter followed by whitespaces after a command
	void ChangeDirectory(const char *newDirectory);

	void CloseDataPort();

	bool haveCompleteLine;
	bool haveFileToMove;
	char clientMessage[ftpMessageLength];
	size_t clientPointer;

	String<MaxFilenameLength> currentDirectory;
};

#endif /* SRC_NETWORKING_FTPRESPONDER_H_ */
