/*
 * TelnetResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#ifndef SRC_NETWORKING_TELNETRESPONDER_H_
#define SRC_NETWORKING_TELNETRESPONDER_H_

#include "NetworkResponder.h"

class TelnetResponder : public NetworkResponder
{
public:
	TelnetResponder(NetworkResponder *n) noexcept;
	bool Spin() noexcept override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, NetworkProtocol protocol) noexcept override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(NetworkProtocol protocol, NetworkInterface *interface) noexcept override;	// terminate the responder if it is serving the specified protocol on the specified interface

	static void InitStatic() noexcept;
	static void Disable() noexcept;
	static void HandleGCodeReply(const char *reply) noexcept;
	static void HandleGCodeReply(OutputBuffer *reply) noexcept;
	void Diagnostics(MessageType mtype) const noexcept override;

private:
	void CharFromClient(char c) noexcept;
	void ProcessLine() noexcept;
	void ConnectionLost() noexcept override;

	bool SendGCodeReply() noexcept;

	bool haveCompleteLine;
	char clientMessage[MaxGCodeLength];
	size_t clientPointer;
	uint32_t connectTime;

	static unsigned int numSessions;
	static unsigned int clientsServed;
	static OutputBuffer *gcodeReply;
	static Mutex gcodeReplyMutex;

	static const uint32_t TelnetSetupDuration = 4000;	// ignore the first Telnet request within this duration (in ms)
};

#endif /* SRC_NETWORKING_TELNETRESPONDER_H_ */
