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
	explicit FtpResponder(NetworkResponder *_ecv_from _ecv_null n) noexcept;

	bool Spin() noexcept override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, NetworkProtocol protocol) noexcept override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(NetworkProtocol protocol, const NetworkInterface *_ecv_from interface) noexcept override;	// terminate the responder if it is serving the specified protocol on the specified interface

	void Diagnostics(MessageType mtype) const noexcept override;

	static void InitStatic() noexcept;
	static void Disable() noexcept;

protected:
	void ConnectionLost() noexcept override;
	void SendData() noexcept override;
	void SendPassiveData() noexcept;
	void DoUpload() noexcept;
	bool ReadData() noexcept;
	void CharFromClient(char c) noexcept;
	void ProcessLine() noexcept;
	const char *_ecv_array GetParameter(const char *_ecv_array after) const noexcept;	// return the parameter followed by whitespaces after a command
	void ChangeDirectory(const char *_ecv_array newDirectory) noexcept;
	void CloseDataPort() noexcept;

	static const size_t ftpMessageLength = 128;			// maximum line length for incoming FTP commands
	static const uint32_t ftpPasvPortTimeout = 10000;	// maximum time to wait for an FTP data connection in milliseconds

	Socket *_ecv_null dataSocket;
	TcpPort passivePort;
	uint32_t passivePortOpenTime;
	OutputBuffer *_ecv_null dataBuf;

	bool sendError;
	bool haveCompleteLine;
	bool haveFileToMove;
	char clientMessage[ftpMessageLength];
	size_t clientPointer;

	String<MaxFilenameLength> currentDirectory;
};

#endif /* SRC_NETWORKING_FTPRESPONDER_H_ */
