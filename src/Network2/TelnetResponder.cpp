/*
 * TelnetResponder.cpp
 *
 *  Created on: 15 Apr 2017
 *      Author: David
 */

#include "TelnetResponder.h"
#include "OutputMemory.h"
#include "GCodes/GCodes.h"
#include "Platform.h"

TelnetResponder::TelnetResponder(NetworkResponder *n) : NetworkResponder(n)
{
}

// Ask the responder to accept this connection, returns true if it did
bool TelnetResponder::Accept(Socket *s, Protocol protocol)
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
void TelnetResponder::Terminate(Protocol protocol)
{
	if (responderState != ResponderState::free && (protocol == TelnetProtocol || protocol == AnyProtocol))
	{
		ConnectionLost();
	}
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
			return readSomething;
		}

	case ResponderState::sending:
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
	if (StringEquals(clientMessage, "exit") || StringEquals(clientMessage, "quit"))
	{
		if (outBuf != nullptr || OutputBuffer::Allocate(outBuf))
		{
			haveCompleteLine = false;
			clientPointer = 0;

			outBuf->copy("Goodbye.\r\n");
			Commit();
		}
	}
	else if (reprap.GetGCodes().GetTelnetInput()->BufferSpaceLeft() >= clientPointer + 1)
	{
		// All other codes are stored for the GCodes class
		RegularGCodeInput * const telnetInput = reprap.GetGCodes().GetTelnetInput();
		telnetInput->Put(TelnetMessage, clientMessage);
		haveCompleteLine = false;
		clientPointer = 0;
	}
}

void TelnetResponder::HandleGCodeReply(const char *reply)
{
	if (reply != nullptr && (responderState == ResponderState::reading || responderState == ResponderState::sending))
	{
		// We need a valid OutputBuffer to start the conversion from NL to CRNL
		if (outBuf == nullptr)
		{
			if (!OutputBuffer::Allocate(outBuf))
			{
				// No more space available to store this reply, stop here
				return;
			}
		}

		// Write entire content to new output buffers, but this time with \r\n instead of \n
		while (*reply != 0)
		{
			if (*reply == '\n' && outBuf->cat('\r') == 0)
			{
				// No more space available, stop here
				return;
			}
			if (outBuf->cat(*reply) == 0)
			{
				// No more space available, stop here
				return;
			}
			reply++;
		}
	}
}

void TelnetResponder::HandleGCodeReply(OutputBuffer *reply)
{
	if (reply != nullptr && (responderState == ResponderState::reading || responderState == ResponderState::sending))
	{
		// We need a valid OutputBuffer to start the conversion from NL to CRNL
		if (outBuf == nullptr)
		{
			if (!OutputBuffer::Allocate(outBuf))
			{
				OutputBuffer::Truncate(reply, OUTPUT_BUFFER_SIZE);
				if (!OutputBuffer::Allocate(outBuf))
				{
					// If we're really short on memory, release the G-Code reply instantly
					OutputBuffer::ReleaseAll(reply);
					return;
				}
			}
		}

		// Write entire content to new output buffers, but this time with \r\n instead of \n
		do {
			const char *data = reply->Data();
			for(size_t i = 0; i < reply->DataLength(); i++)
			{
				if (*data == '\n')
				{
					outBuf->cat('\r');
				}
				outBuf->cat(*data);
				data++;
			}
			reply = OutputBuffer::Release(reply);
		} while (reply != nullptr);
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

// End
