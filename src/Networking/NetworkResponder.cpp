/*
 * NetworkResponder.cpp
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#include "NetworkResponder.h"
#include "Socket.h"
#include <Platform/Platform.h>

// NetworkResponder members

NetworkResponder::NetworkResponder(NetworkResponder *n) noexcept
	: next(n), responderState(ResponderState::free), skt(nullptr),
	  outBuf(nullptr),
#if HAS_MASS_STORAGE
	  fileBeingSent(nullptr),
#endif
	  fileBuffer(nullptr), terminateResponder(false)
{
}

// Send the contents of the output buffers
void NetworkResponder::Commit(ResponderState nextState, bool report) noexcept
{
	stateAfterSending = nextState;
	responderState = ResponderState::sending;
	if (report && reprap.Debug(Module::Webserver))
	{
#if HAS_MASS_STORAGE
		debugPrintf("Sending reply, file = %s\n", (fileBeingSent != nullptr) ? "yes" : "no");
#else
		debugPrintf("Sending reply");
#endif
	}
}

// Send our data.
// We send outBuf first, then outStack, and finally fileBeingSent.
void NetworkResponder::SendData() noexcept
{
	// Send our output buffer and output stack
	while (!terminateResponder)
	{
		if (outBuf == nullptr)
		{
			outBuf = outStack.Pop();
			if (outBuf == nullptr)
			{
				break;
			}
		}
		const size_t bytesLeft = outBuf->BytesLeft();
		if (bytesLeft == 0)
		{
			outBuf = OutputBuffer::Release(outBuf);
		}
		else
		{
			const size_t sent = skt->Send(reinterpret_cast<const uint8_t *>(outBuf->UnreadData()), bytesLeft);
			if (sent == 0)
			{
				// Check whether the connection has been closed
				if (!skt->CanSend())
				{
					// The connection has been lost or the other end has closed it
					if (reprap.Debug(Module::Webserver))
					{
						debugPrintf("Can't send anymore\n");
					}
					ConnectionLost();
				}
				return;
			}

			outBuf->Taken(sent);				// tell the output buffer how much data we have taken
			if (sent < bytesLeft)
			{
				return;
			}
			outBuf = OutputBuffer::Release(outBuf);
		}
	}

	// If we get here then there are no output buffers left to send

#if HAS_MASS_STORAGE
	// If we have a file to send, send it
	if (fileBeingSent != nullptr && fileBuffer == nullptr)
	{
		fileBuffer = NetworkBuffer::Allocate();
		if (fileBuffer == nullptr)
		{
			return;					// no buffer available, try again later
		}
	}

	// If we have a file buffer here, we must be in the process of sending a file
	while (fileBuffer != nullptr && !terminateResponder)
	{
		if (fileBuffer->IsEmpty() && fileBeingSent != nullptr)
		{
			const int bytesRead = fileBuffer->ReadFromFile(fileBeingSent);
			if (bytesRead != (int)NetworkBuffer::bufferSize)
			{
				// We had a read error or we reached the end of the file
				fileBeingSent->Close();
				fileBeingSent = nullptr;
			}
		}

		if (fileBuffer->IsEmpty())
		{
			// Must have sent the whole file
			fileBuffer->Release();
			fileBuffer = nullptr;
		}
		else
		{
			const size_t remaining = fileBuffer->Remaining();
			const size_t sent = skt->Send(fileBuffer->UnreadData(), remaining);
			if (sent == 0)
			{
				// Check whether the connection has been closed
				if (!skt->CanSend())
				{
					if (reprap.Debug(Module::Webserver))
					{
						debugPrintf("Can't send anymore\n");
					}
					ConnectionLost();
				}
				return;
			}

			fileBuffer->Taken(sent);

			if (   sent < remaining				// if we couldn't send it all...
				|| fileBuffer->IsEmpty()		// ...or if we've sent the whole buffer, return to allow other sockets to be polled
			   )
			{
				return;
			}
		}
	}
#endif

	// If we get here then there is nothing left to send
	skt->Send();								// tell the socket there is no more data

	// If we are going to free up this responder after sending, then we must close the connection
	if (stateAfterSending == ResponderState::free)
	{
		skt->Close();
		skt = nullptr;
	}
	responderState = stateAfterSending;
}

// This is called when we lose a connection or when we are asked to terminate. Overridden in some derived classes.
void NetworkResponder::ConnectionLost() noexcept
{
	OutputBuffer::ReleaseAll(outBuf);
	outStack.ReleaseAll();

#if HAS_MASS_STORAGE
	if (fileBeingSent != nullptr)
	{
		fileBeingSent->Close();
		fileBeingSent = nullptr;
	}
#endif

	if (fileBuffer != nullptr)
	{
		fileBuffer->Release();
		fileBuffer = nullptr;
	}

	if (skt != nullptr)
	{
		skt->Terminate();
		skt = nullptr;
	}

	responderState = ResponderState::free;
}

IPAddress NetworkResponder::GetRemoteIP() const noexcept
{
	return (skt == nullptr) ? IPAddress() : skt->GetRemoteIP();
}

void NetworkResponder::ReportOutputBufferExhaustion(const char *sourceFile, int line) noexcept
{
	if (reprap.Debug(Module::Webserver))
	{
		debugPrintf("Ran out of output buffers at %s(%d)\n", sourceFile, line);
	}
}

// End
