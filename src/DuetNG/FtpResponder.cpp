/*
 * FtpResponder.cpp
 *
 *  Created on: 15 Apr 2017
 *      Author: David
 */

#include "FtpResponder.h"
#include "Network.h"
#include "Platform.h"

FtpResponder::FtpResponder(NetworkResponder *n) : NetworkResponder(n), dataSocket(nullptr), passivePort(0)
{
	strcpy(currentDir, "/");
}

// Ask the responder to accept this connection, returns true if it did
bool FtpResponder::Accept(Socket *s, Protocol protocol)
{
	if (responderState == ResponderState::free && protocol == FtpProtocol)
	{
		skt = s;
		responderState = ResponderState::reading;
		if (reprap.Debug(moduleWebserver))
		{
			debugPrintf("FTP connection accepted\n");
		}
		return true;
	}
	else if (responderState == ResponderState::waitingForPasvPort && protocol == FtpDataProtocol && s->GetLocalPort() == passivePort)
	{
		dataSocket = s;
		responderState = ResponderState::pasvPortOpened;
		if (reprap.Debug(moduleWebserver))
		{
			debugPrintf("FTP data connection accepted\n");
		}
		return true;
	}
	//TODO also we need to handle a second connection to the main port number, to transfer files
	return false;
}

// This is called to force termination if we implement the specified protocol
void FtpResponder::Terminate(Protocol protocol)
{
	if (protocol == FtpProtocol || protocol == AnyProtocol)
	{
		ConnectionLost();
	}
}

// Do some work, returning true if we did anything significant
bool FtpResponder::Spin()
{
	switch (responderState)
	{
	case ResponderState::free:
	default:
		return false;

	case ResponderState::reading:
		// TODO
		return true;
	}
	return false;
}

void FtpResponder::ConnectionLost()
{
	if (dataSocket != nullptr)
	{
		dataSocket->Terminate();
		dataSocket = nullptr;
	}
	NetworkResponder::ConnectionLost();
}

void FtpResponder::Diagnostics(MessageType mt)
{
	GetPlatform()->MessageF(mt, "FTP state %d\n", (int)responderState);
}

// End
