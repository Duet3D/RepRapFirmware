/*
 * NetworkResponder.cpp
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#include "NetworkResponder.h"
#include "Socket.h"
#include "Platform.h"

// NetworkResponder members

NetworkResponder::NetworkResponder(NetworkResponder *n)
	: next(n), responderState(ResponderState::free), skt(nullptr),
	  outBuf(nullptr), fileBeingSent(nullptr), fileBuffer(nullptr)
{
}

// Send the contents of the output buffers
void NetworkResponder::Commit(ResponderState nextState, bool report)
{
	stateAfterSending = nextState;
	responderState = ResponderState::sending;
	if (report && reprap.Debug(moduleWebserver))
	{
		debugPrintf("Sending reply, file = %s\n", (fileBeingSent != nullptr) ? "yes" : "no");
	}
}

// Send our data.
// We send outBuf first, then outStack, and finally fileBeingSent.
void NetworkResponder::SendData()
{
	// Send our output buffer and output stack
	for(;;)
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
					if (reprap.Debug(moduleWebserver))
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
	while (fileBuffer != nullptr)
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
					if (reprap.Debug(moduleWebserver))
					{
						debugPrintf("Can't send anymore\n");
					}
					ConnectionLost();
				}
				return;
			}

			fileBuffer->Taken(sent);
			if (sent < remaining)
			{
				return;
			}
		}
	}

	// If we get here then there is nothing left to send
	skt->Send();						// tell the socket there is no more data

	// If we are going to free up this responder after sending, then we must close the connection
	if (stateAfterSending == ResponderState::free)
	{
		skt->Close();
		skt = nullptr;
	}
	responderState = stateAfterSending;
}

// This is called when we lose a connection or when we are asked to terminate. Overridden in some derived classes.
void NetworkResponder::ConnectionLost()
{
	CancelUpload();
	OutputBuffer::ReleaseAll(outBuf);
	outStack.ReleaseAll();

	if (fileBeingSent != nullptr)
	{
		fileBeingSent->Close();
		fileBeingSent = nullptr;
	}

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

// Start writing to a new file
void NetworkResponder::StartUpload(FileStore *file, const char *fileName)
{
	fileBeingUploaded.Set(file);
	SafeStrncpy(filenameBeingUploaded, fileName, ARRAY_SIZE(filenameBeingUploaded));
	responderState = ResponderState::uploading;
	uploadError = false;
}

// If this responder has an upload in progress, cancel it
void NetworkResponder::CancelUpload()
{
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
		if (filenameBeingUploaded[0] != 0)
		{
			GetPlatform().GetMassStorage()->Delete(FS_PREFIX, filenameBeingUploaded);
		}
	}
}

// Finish a file upload. Set variable uploadError if anything goes wrong.
void NetworkResponder::FinishUpload(uint32_t fileLength, time_t fileLastModified)
{
	// Flush remaining data for FSO
	if (!fileBeingUploaded.Flush())
	{
		uploadError = true;
		GetPlatform().Message(ErrorMessage, "Could not flush remaining data while finishing upload!\n");
	}

	// Check the file length is as expected
	if (fileLength != 0 && fileBeingUploaded.Length() != fileLength)
	{
		uploadError = true;
		GetPlatform().MessageF(ErrorMessage, "Uploaded file size is different (%lu vs. expected %lu bytes)!\n", fileBeingUploaded.Length(), fileLength);
	}

	// Close the file
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
	}

	// Delete the file again if an error has occurred
	if (filenameBeingUploaded[0] != 0)
	{
		if (uploadError)
		{
			GetPlatform().GetMassStorage()->Delete(FS_PREFIX, filenameBeingUploaded);
		}
		else if (fileLastModified != 0)
		{
			// Update the file timestamp if it was specified
			(void)GetPlatform().GetMassStorage()->SetLastModifiedTime(nullptr, filenameBeingUploaded, fileLastModified);
		}
	}

	// Clean up again
	filenameBeingUploaded[0] = 0;
}

uint32_t NetworkResponder::GetRemoteIP() const
{
	return (skt == nullptr) ? 0 : skt->GetRemoteIP();
}

void NetworkResponder::ReportOutputBufferExhaustion(const char *sourceFile, int line)
{
	if (reprap.Debug(moduleWebserver))
	{
		debugPrintf("Ran out of output buffers at %s(%d)\n", sourceFile, line);
	}
}

// End
