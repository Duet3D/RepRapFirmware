/*
 * TelnetResponder.cpp
 *
 *  Created on: 15 Apr 2017
 *      Author: David
 */

#include "TelnetResponder.h"

#if SUPPORT_TELNET

#include "Socket.h"
#include "OutputMemory.h"
#include "GCodes/GCodes.h"
#include "Platform.h"

TelnetResponder::TelnetResponder(NetworkResponder *n) : NetworkResponder(n)
{
}

// Ask the responder to accept this connection, returns true if it did
bool TelnetResponder::Accept(Socket *s, NetworkProtocol protocol)
{
	if (responderState == ResponderState::free && protocol == TelnetProtocol)
	{
		// Make sure we can get an output buffer before we accept the connection, or we won't be able to reply
		if (outBuf != nullptr || OutputBuffer::Allocate(outBuf))
		{
			skt = s;
			clientPointer = 0;
			responderState = ResponderState::justConnected;
			connectTime = millis();
			haveCompleteLine = false;
			if (reprap.Debug(moduleWebserver))
			{
				debugPrintf("Telnet connection accepted\n");
			}
			return true;
		}
	}
	return false;
}

// This is called to force termination if we implement the specified protocol
void TelnetResponder::Terminate(NetworkProtocol protocol)
{
	if (responderState != ResponderState::free && (protocol == TelnetProtocol || protocol == AnyProtocol))
	{
		ConnectionLost();
	}
}

void TelnetResponder::ConnectionLost()
{
	if ((responderState == ResponderState::reading) || (responderState == ResponderState::sending))
	{
		MutexLocker lock(gcodeReplyMutex);

		if (numSessions != 0)
		{
			numSessions--;
		}
		if (gcodeReply != nullptr && clientsServed > numSessions)
		{
			// Make sure the G-code reply is freed after it is sent to all clients
			OutputBuffer::ReleaseAll(gcodeReply);
			clientsServed = 0;
		}
	}

	NetworkResponder::ConnectionLost();
}

bool TelnetResponder::SendGCodeReply()
{
	MutexLocker lock(gcodeReplyMutex);

	if (gcodeReply != nullptr)
	{
		bool clearReply = false;
		clientsServed++;
		if (clientsServed < numSessions)
		{
			// Yes - make sure the Network class doesn't discard its buffers yet
			// NB: This must happen here, because NetworkTransaction::Write() might already release OutputBuffers
			gcodeReply->IncreaseReferences(1);
		}
		else
		{
			// No - clean up again later
			clearReply = true;
		}

		if (reprap.Debug(moduleWebserver))
		{
			GetPlatform().MessageF(UsbMessage, "Sending G-Code reply to Telnet client %d of %d (length %u)\n", clientsServed, numSessions, gcodeReply->DataLength());
		}

		// Send the whole G-Code reply as plain text to the client
		outStack.Push(gcodeReply);

		// Possibly clean up the G-code reply once again
		if (clearReply)
		{
			gcodeReply = nullptr;
		}
		return true;
	}
	return false;
}

// Do some work, returning true if we did anything significant
bool TelnetResponder::Spin()
{
	switch (responderState)
	{
	case ResponderState::free:
	default:
		return false;

	case ResponderState::justConnected:
		if (millis() - connectTime < TelnetSetupDuration)
		{
			// Discard the setup message
			char c;
			while (skt->ReadChar(c)) { }
			connectTime = 0;
			return true;
		}

		// Check whether we need a password to log in
		if (reprap.NoPasswordSet())
		{
			// Don't send a login prompt if no password is set, so we don't mess up Pronterface
			responderState = ResponderState::reading;
			numSessions++;
		}
		else
		{
			outBuf->copy(	"RepRapFirmware Telnet interface\r\n\r\n"
							"Please enter your password:\r\n"
							"> "
						);
			Commit(ResponderState::authenticating);
		}
		return true;

	case ResponderState::authenticating:
		{
			bool readSomething = false;
			char c;
			while (!haveCompleteLine && skt->ReadChar(c))
			{
				CharFromClient(c);
				readSomething = true;
			}

			if (!readSomething && !skt->CanRead())
			{
				ConnectionLost();
				return true;
			}

			if (haveCompleteLine && (outBuf != nullptr || OutputBuffer::Allocate(outBuf)))
			{
				haveCompleteLine = false;
				clientPointer = 0;
				if (reprap.CheckPassword(clientMessage))
				{
					numSessions++;
					outBuf->copy("Log in successful!\r\n");
					Commit(ResponderState::reading);
				}
				else
				{
					outBuf->copy("Invalid password.\r\n> ");
					Commit(ResponderState::authenticating);
				}
				return true;
			}
			return readSomething;
		}

	case ResponderState::reading:
		if (outBuf != nullptr && outBuf->BytesLeft() != 0)
		{
			// We have a gcode reply to send
			Commit(ResponderState::reading);
			return true;
		}
		{
			// See if we can read anything
			bool readSomething = false;
			char c;
			while (!haveCompleteLine && skt->ReadChar(c))
			{
				CharFromClient(c);
				readSomething = true;
			}

			if (!readSomething && !skt->CanRead())
			{
				ConnectionLost();
				return true;
			}

			if (haveCompleteLine)
			{
				ProcessLine();
				return true;
			}

			// Try to send the G-code reply (if any)
			if (SendGCodeReply())
			{
				Commit(ResponderState::reading);
			}
			return readSomething;
		}

	case ResponderState::sending:
		SendGCodeReply();
		SendData();
		return true;
	}
}

// Process a character from the client, returning true if we have a complete line
void TelnetResponder::CharFromClient(char c)
{
	switch (c)
	{
	case 0:
		break;

	case '\b':
		// Allow backspace for pure Telnet clients like PuTTY
		if (clientPointer != 0)
		{
			clientPointer--;
		}
		break;

	case '\r':
	case '\n':
		if (clientPointer != 0)
		{
			// This line is complete
			clientMessage[clientPointer] = 0;
			haveCompleteLine = true;
		}
		break;

	default:
		clientMessage[clientPointer++] = c;

		// Make sure we don't overflow the line buffer
		if (clientPointer > ARRAY_UPB(clientMessage))
		{
			clientPointer = 0;
			GetPlatform().Message(UsbMessage, "Webserver: Buffer overflow in Telnet server!\n");
		}
		break;
	}
}

// Usually we should not try to send any data here, because that would purge the packet's
// payload and mess with TCP streaming mode if Pronterface is used. However, under special
// circumstances this must happen.
void TelnetResponder::ProcessLine()
{
	// Special commands for Telnet
	if (StringEqualsIgnoreCase(clientMessage, "exit") || StringEqualsIgnoreCase(clientMessage, "quit"))
	{
		if (outBuf != nullptr || OutputBuffer::Allocate(outBuf))
		{
			haveCompleteLine = false;
			clientPointer = 0;
			numSessions--;

			outBuf->copy("Goodbye.\r\n");
			Commit();
		}
	}
	else if (reprap.GetGCodes().GetTelnetInput()->BufferSpaceLeft() >= clientPointer + 1)
	{
		// All other codes are stored for the GCodes class
		NetworkGCodeInput * const telnetInput = reprap.GetGCodes().GetTelnetInput();
		telnetInput->Put(TelnetMessage, clientMessage);
		haveCompleteLine = false;
		clientPointer = 0;
	}
}

/*static*/ void TelnetResponder::InitStatic()
{
	gcodeReplyMutex.Create("TelnetGCodeReply");
}

/*static*/ void TelnetResponder::HandleGCodeReply(const char *reply)
{
	if (reply != nullptr && numSessions > 0)
	{
		MutexLocker lock(gcodeReplyMutex);

		// We need a valid OutputBuffer to start the conversion from NL to CRNL
		if (gcodeReply == nullptr && !OutputBuffer::Allocate(gcodeReply))
		{
			// No more space available to store this reply, stop here
			return;
		}

		// Write entire content to new output buffers, but this time with \r\n instead of \n
		while (*reply != 0)
		{
			if (*reply == '\n' && gcodeReply->cat('\r') == 0)
			{
				// No more space available, stop here
				return;
			}
			if (gcodeReply->cat(*reply) == 0)
			{
				// No more space available, stop here
				return;
			}
			reply++;
		}

		// Send it to the clients
		clientsServed = 0;
	}
}

/*static*/ void TelnetResponder::HandleGCodeReply(OutputBuffer *reply)
{
	if (reply != nullptr && numSessions > 0)
	{
		MutexLocker lock(gcodeReplyMutex);

		// We need a valid OutputBuffer to start the conversion from NL to CRNL
		if (gcodeReply == nullptr && !OutputBuffer::Allocate(gcodeReply))
		{
			OutputBuffer::Truncate(reply, OUTPUT_BUFFER_SIZE);
			if (!OutputBuffer::Allocate(gcodeReply))
			{
				// If we're really short on memory, release the G-Code reply instantly
				OutputBuffer::ReleaseAll(reply);
				return;
			}
		}

		// Write entire content to new output buffers, but this time with \r\n instead of \n
		do {
			const char *data = reply->Data();
			for(size_t i = 0; i < reply->DataLength(); i++)
			{
				if (*data == '\n')
				{
					gcodeReply->cat('\r');
				}
				gcodeReply->cat(*data);
				data++;
			}
			reply = OutputBuffer::Release(reply);
		} while (reply != nullptr);

		// Send it to the clients
		clientsServed = 0;
	}
	else
	{
		// Don't store buffers that may never get released...
		OutputBuffer::ReleaseAll(reply);
	}
}

void TelnetResponder::Diagnostics(MessageType mt) const
{
	GetPlatform().MessageF(mt, " Telnet(%d)", (int)responderState);
}

unsigned int TelnetResponder::numSessions = 0;
unsigned int TelnetResponder::clientsServed = 0;
OutputBuffer *TelnetResponder::gcodeReply = nullptr;
Mutex TelnetResponder::gcodeReplyMutex;

#endif

// End
