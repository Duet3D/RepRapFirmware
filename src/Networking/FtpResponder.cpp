/*
 * FtpResponder.cpp
 *
 *  Created on: 15 Apr 2017
 *      Authors: David and Christian
 */

#include "FtpResponder.h"

#if SUPPORT_FTP

#include "Socket.h"
#include "Network.h"
#include "NetworkInterface.h"
#include "Platform.h"

FtpResponder::FtpResponder(NetworkResponder *n)
	: UploadingNetworkResponder(n), dataSocket(nullptr), passivePort(0), passivePortOpenTime(0), dataBuf(nullptr), haveFileToMove(false)
{
}

// Ask the responder to accept this connection, returns true if it did
bool FtpResponder::Accept(Socket *s, NetworkProtocol protocol)
{
	if (responderState == ResponderState::free && protocol == FtpProtocol)
	{
		// Make sure we can get an output buffer before we accept the connection, or we won't be able to reply
		if (outBuf != nullptr || OutputBuffer::Allocate(outBuf))
		{
			clientPointer = 0;
			skt = s;
			if (reprap.Debug(moduleWebserver))
			{
				debugPrintf("FTP connection accepted\n");
			}

			outBuf->copy("220 RepRapFirmware FTP server\r\n");
			Commit(ResponderState::authenticating);
			haveCompleteLine = false;
			return true;
		}
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
	return false;
}

// This is called to force termination if we implement the specified protocol
void FtpResponder::Terminate(NetworkProtocol protocol, NetworkInterface *interface)
{
	if (responderState != ResponderState::free && (protocol == FtpProtocol || protocol == AnyProtocol) && skt != nullptr && skt->GetInterface() == interface)
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
		return false;

	case ResponderState::authenticating:
	case ResponderState::reading:
		return ReadData();

	case ResponderState::sending:
		SendData();
		return true;

	case ResponderState::waitingForPasvPort:
		if (millis() - passivePortOpenTime > ftpPasvPortTimeout && (outBuf != nullptr || OutputBuffer::Allocate(outBuf)))
		{
			outBuf->copy("425 Failed to establish connection.\r\n");
			Commit(ResponderState::reading);

			CloseDataPort();
			return true;
		}
		return false;

	case ResponderState::pasvPortOpened:
		if (dataBuf != nullptr || OutputBuffer::Allocate(dataBuf))
		{
			return ReadData();
		}
		return false;

	case ResponderState::uploading:
		DoUpload();

		if (!uploadError && skt->CanRead())
		{
			ReadData();			// check for incoming ABOR requests
		}
		return true;

	case ResponderState::sendingPasvData:
		SendPassiveData();

		if (!sendError && skt->CanRead())
		{
			ReadData();			// check for incoming ABOR requests
		}
		return true;

	case ResponderState::pasvTransferComplete:
		if (outBuf != nullptr || OutputBuffer::Allocate(outBuf))
		{
			// Is the main FTP connection still available?
			if (skt->CanSend())
			{
				// Yes - send a response
				if (uploadError || sendError)
				{
					outBuf->copy("526 Transfer failed!\r\n");
				}
				else
				{
					outBuf->copy("226 Transfer complete.\r\n");
				}
				Commit(ResponderState::reading);
				CloseDataPort();
			}
			else
			{
				// No - reset our state
				ConnectionLost();
			}

			return true;
		}
		return false;

	default:	// should not happen
		return false;
	}
}

void FtpResponder::Diagnostics(MessageType mt) const
{
	GetPlatform().MessageF(mt, " FTP(%d)", (int)responderState);
}

// This must be called only for the main FTP port
void FtpResponder::ConnectionLost()
{
	CloseDataPort();
	NetworkResponder::ConnectionLost();
}

// Send our data over the main FTP port.
// We send outBuf first and then outStack. fileBeingSent is reserved for the data port.
void FtpResponder::SendData()
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

// Send our data over the passive FTP data port.
// We send dataBuf first and then fileBeingSent.
void FtpResponder::SendPassiveData()
{
	// Send our output buffers
	while (dataBuf != nullptr)
	{
		const size_t bytesLeft = dataBuf->BytesLeft();
		if (bytesLeft == 0)
		{
			dataBuf = OutputBuffer::Release(dataBuf);
		}
		else
		{
			const size_t sent = dataSocket->Send(reinterpret_cast<const uint8_t *>(dataBuf->UnreadData()), bytesLeft);
			if (sent == 0)
			{
				// Check whether the connection has been closed
				if (!dataSocket->CanSend())
				{
					if (reprap.Debug(moduleWebserver))
					{
						debugPrintf("Can't send anymore over the data port\n");
					}

					sendError = true;
					dataSocket = nullptr;
					if (fileBeingSent != nullptr)
					{
						fileBeingSent->Close();
						fileBeingSent = nullptr;
					}

					responderState = ResponderState::pasvTransferComplete;
				}
				return;
			}

			// Tell the output buffer how much data we have taken
			dataBuf->Taken(sent);
			if (sent < bytesLeft)
			{
				return;
			}
			dataBuf = OutputBuffer::Release(dataBuf);
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
			const size_t sent = dataSocket->Send(fileBuffer->UnreadData(), remaining);
			if (sent == 0)
			{
				// Check whether the connection has been closed
				if (!dataSocket->CanSend())
				{
					if (reprap.Debug(moduleWebserver))
					{
						debugPrintf("Can't send anymore\n");
					}

					sendError = true;
					dataSocket = nullptr;
					if (fileBeingSent != nullptr)
					{
						fileBeingSent->Close();
						fileBeingSent = nullptr;
					}
					fileBuffer->Release();
					fileBuffer = nullptr;

					responderState = ResponderState::pasvTransferComplete;
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

	// If we get here then there is nothing left to send. Close it as well
	dataSocket->Send();						// tell the socket there is no more data
	dataSocket->Close();
	dataSocket = nullptr;

	responderState = ResponderState::pasvTransferComplete;
}

// Write some more upload data
void FtpResponder::DoUpload()
{
	// Write incoming data to the file
	const uint8_t *buffer;
	size_t len;
	if (dataSocket->ReadBuffer(buffer, len))
	{
		if (reprap.Debug(moduleWebserver))
		{
			GetPlatform().MessageF(UsbMessage, "Writing %u bytes of upload data\n", len);
		}

		dataSocket->Taken(len);
		if (!fileBeingUploaded.Write(buffer, len))
		{
			uploadError = true;
			GetPlatform().Message(ErrorMessage, "FTP: could not write upload data\n");
			CancelUpload();

			responderState = ResponderState::pasvTransferComplete;
			return;
		}
	}

	// Upload has finished if the connection is closed
	if (!dataSocket->CanRead())
	{
		dataSocket = nullptr;
		responderState = ResponderState::pasvTransferComplete;

		FinishUpload(0, 0, false, 0);
	}
}

// Try to read some data from the main FTP port and return true
// if anything significant could be done
bool FtpResponder::ReadData()
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
		ProcessLine();
		return true;
	}
	return readSomething;
}

// Keep track of incoming characters
void FtpResponder::CharFromClient(char c)
{
	switch (c)
	{
	case 0:
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
			GetPlatform().Message(UsbMessage, "Webserver: Buffer overflow in FTP server!\n");
		}
		break;
	}
}

// Process the next FTP command
void FtpResponder::ProcessLine()
{
	if (reprap.Debug(moduleWebserver))
	{
		debugPrintf("FTP request '%s' (state %d)\n", clientMessage, (int)responderState);
	}

	haveCompleteLine = false;
	clientPointer = 0;

	switch (responderState)
	{
	case ResponderState::authenticating:
		haveFileToMove = false;
		filenameBeingProcessed.Clear();

		// don't check the user name
		if (StringStartsWith(clientMessage, "USER"))
		{
			outBuf->copy("331 Please specify the password.\r\n");
			Commit(ResponderState::authenticating);
		}
		// but check the password (if set)
		else if (StringStartsWith(clientMessage, "PASS"))
		{
			const char *password = GetParameter("PASS");
			if (reprap.NoPasswordSet() || reprap.CheckPassword(password))
			{
				currentDirectory.copy("/");

				outBuf->copy("230 Login successful.\r\n");
				Commit(ResponderState::reading);
			}
			else
			{
				outBuf->copy("530 Login incorrect.\r\n");
				Commit();
			}
		}
		// end connection
		else if (StringEqualsIgnoreCase(clientMessage, "QUIT"))
		{
			outBuf->copy("221 Goodbye.\r\n");
			Commit();
		}
		// if it's different, send response 500 to indicate we don't know the code (might be AUTH or so)
		else
		{
			outBuf->copy("500 Unknown login command.\r\n");
			Commit(ResponderState::authenticating);
		}
		break;

	case ResponderState::reading:
		// get system type
		if (StringEqualsIgnoreCase(clientMessage, "SYST"))
		{
			outBuf->copy("215 UNIX Type: L8\r\n");
			Commit(ResponderState::reading);
		}
		// get features
		else if (StringEqualsIgnoreCase(clientMessage, "FEAT"))
		{
			outBuf->copy(	"211-Features:\r\n"
							"PASV\r\n"			// support PASV mode
							"211 End\r\n"
						);
			Commit(ResponderState::reading);
		}
		// get current dir
		else if (StringEqualsIgnoreCase(clientMessage, "PWD"))
		{
			outBuf->printf("257 \"%s\"\r\n", currentDirectory.c_str());
			Commit(ResponderState::reading);
		}
		// set current dir
		else if (StringStartsWith(clientMessage, "CWD"))
		{
			const char *directory = GetParameter("CWD");
			ChangeDirectory(directory);
		}
		// change to parent of current directory
		else if (StringEqualsIgnoreCase(clientMessage, "CDUP"))
		{
			ChangeDirectory("..");
		}
		// switch transfer mode (sends response, but doesn't have any effects)
		else if (StringStartsWith(clientMessage, "TYPE"))
		{
			const char *type = GetParameter("TYPE");
			if (StringEqualsIgnoreCase(type, "I"))
			{
				outBuf->copy("200 Switching to Binary mode.\r\n");
			}
			else if (StringEqualsIgnoreCase(type, "A"))
			{
				outBuf->copy("200 Switching to ASCII mode.\r\n");
			}
			else
			{
				outBuf->copy("500 Unknown command.\r\n");
			}
			Commit(ResponderState::reading);
		}
		// enter passive mode mode
		else if (StringEqualsIgnoreCase(clientMessage, "PASV"))
		{
			// reset error conditions
			uploadError = sendError = false;

			// open random port > 1023
			passivePort = random(1024, 65535);
			passivePortOpenTime = millis();

			skt->GetInterface()->OpenDataPort(passivePort);
			if (reprap.Debug(moduleWebserver))
			{
				debugPrintf("FTP data port open at port %u\n", passivePort);
			}

			// send FTP response
			const IPAddress ipAddress = skt->GetInterface()->GetIPAddress();
			outBuf->printf("227 Entering Passive Mode (%d,%d,%d,%d,%d,%d)\r\n",
					ipAddress.GetQuad(0), ipAddress.GetQuad(1), ipAddress.GetQuad(2), ipAddress.GetQuad(3),
					passivePort / 256, passivePort % 256);
			Commit(ResponderState::waitingForPasvPort);
		}
		// PASV commands are not supported in this state
		else if (StringStartsWith(clientMessage, "LIST") || StringStartsWith(clientMessage, "RETR") || StringStartsWith(clientMessage, "STOR"))
		{
			outBuf->copy("425 Use PASV first.\r\n");
			Commit(ResponderState::reading);
		}
		// delete file
		else if (StringStartsWith(clientMessage, "DELE"))
		{
			const char *filename = GetParameter("DELE");
			if (GetPlatform().Delete(currentDirectory.c_str(), filename))
			{
				outBuf->copy("250 Delete operation successful.\r\n");
			}
			else
			{
				outBuf->copy("550 Delete operation failed.\r\n");
			}
			Commit(ResponderState::reading);
		}
		// delete directory
		else if (StringStartsWith(clientMessage, "RMD"))
		{
			const char *filename = GetParameter("RMD");
			if (GetPlatform().Delete(currentDirectory.c_str(), filename))
			{
				outBuf->copy("250 Remove directory operation successful.\r\n");
			}
			else
			{
				outBuf->copy("550 Remove directory operation failed.\r\n");
			}
			Commit(ResponderState::reading);
		}
		// make new directory
		else if (StringStartsWith(clientMessage, "MKD"))
		{
			const char *filename = GetParameter("MKD");
			String<MaxFilenameLength> location;
			if (MassStorage::CombineName(location.GetRef(), currentDirectory.c_str(), filename)
				&& GetPlatform().GetMassStorage()->MakeDirectory(location.c_str()))
			{
				outBuf->printf("257 \"%s\" created\r\n", location.c_str());
			}
			else
			{
				outBuf->copy("550 Create directory operation failed.\r\n");
			}
			Commit(ResponderState::reading);
		}
		// rename file or directory
		else if (StringStartsWith(clientMessage, "RNFR"))
		{
			const char *filename = GetParameter("RNFR");
			if (MassStorage::CombineName(filenameBeingProcessed.GetRef(), currentDirectory.c_str(), filename)
				&& GetPlatform().GetMassStorage()->FileExists(filenameBeingProcessed.c_str()))
			{
				haveFileToMove = true;
				outBuf->copy("350 Ready to RNTO.\r\n");
			}
			else
			{
				outBuf->copy("550 Invalid file or directory.\r\n");
			}
			Commit(ResponderState::reading);
		}
		else if (StringStartsWith(clientMessage, "RNTO"))
		{
			const char *filename = GetParameter("RNTO");
			String<MaxFilenameLength> location;
			if (haveFileToMove
				&& MassStorage::CombineName(location.GetRef(), currentDirectory.c_str(), filename)
				&& GetPlatform().GetMassStorage()->Rename(filenameBeingProcessed.c_str(), location.c_str()))
			{
				outBuf->copy("250 Rename successful.\r\n");
			}
			else
			{
				outBuf->copy("500 Could not rename file or directory\r\n");
			}
			haveFileToMove = false;
			Commit(ResponderState::reading);
		}
		// no op
		else if (StringEqualsIgnoreCase(clientMessage, "NOOP"))
		{
			outBuf->copy("200 NOOP okay.\r\n");
			Commit(ResponderState::reading);
		}
		// end connection
		else if (StringEqualsIgnoreCase(clientMessage, "QUIT"))
		{
			haveFileToMove = false;
			filenameBeingProcessed.Clear();
			outBuf->copy("221 Goodbye.\r\n");
			Commit();
		}
		// unknown
		else
		{
			outBuf->copy("500 Unknown command.\r\n");
			Commit(ResponderState::reading);
		}
		break;

	case ResponderState::pasvPortOpened:
		// list directory entries
		if (StringStartsWith(clientMessage, "LIST"))
		{
			// send announcement via ftp main port
			outBuf->copy("150 Here comes the directory listing.\r\n");
			Commit(ResponderState::sendingPasvData);

			// build directory listing, dataBuf is sent later in the Spin loop
			MassStorage * const massStorage = GetPlatform().GetMassStorage();
			FileInfo fileInfo;
			if (massStorage->FindFirst(currentDirectory.c_str(), fileInfo))
			{
				do {
					// Example for a typical UNIX-like file list:
					// "drwxr-xr-x    2 ftp      ftp             0 Apr 11 2013 bin\r\n"
					const char dirChar = (fileInfo.isDirectory) ? 'd' : '-';
					const struct tm * const timeInfo = gmtime(&fileInfo.lastModified);
					dataBuf->catf("%crw-rw-rw- 1 ftp ftp %13lu %s %02d %04d %s\r\n",
							dirChar, fileInfo.size, massStorage->GetMonthName(timeInfo->tm_mon + 1),
							timeInfo->tm_mday, timeInfo->tm_year + 1900, fileInfo.fileName.c_str());
				} while (massStorage->FindNext(fileInfo));
			}
		}
		// switch transfer mode (sends response, but doesn't have any effects)
		else if (StringStartsWith(clientMessage, "TYPE"))
		{
			const char *type = GetParameter("TYPE");
			if (StringEqualsIgnoreCase(type, "I"))
			{
				outBuf->copy("200 Switching to Binary mode.\r\n");
			}
			else if (StringEqualsIgnoreCase(type, "A"))
			{
				outBuf->copy("200 Switching to ASCII mode.\r\n");
			}
			else
			{
				outBuf->copy("500 Unknown command.\r\n");
			}
			Commit(ResponderState::pasvPortOpened);
		}
		// upload a file
		else if (StringStartsWith(clientMessage, "STOR"))
		{
			// Variable filenameBeingProcessed is used for both uploading and for renaming files, so clear it here and clear haveFileToMove
			haveFileToMove = false;
			filenameBeingProcessed.Clear();

			const char * const filename = GetParameter("STOR");
			FileStore * const file = StartUpload(currentDirectory.c_str(), filename, OpenMode::write);
			if (file != nullptr)
			{
				outBuf->copy("150 OK to send data.\r\n");
				Commit(ResponderState::uploading);
			}
			else
			{
				outBuf->copy("550 Failed to open file.\r\n");
				Commit(ResponderState::reading);
			}
		}
		// download a file
		else if (StringStartsWith(clientMessage, "RETR"))
		{
			const char * const filename = GetParameter("RETR");
			fileBeingSent = GetPlatform().OpenFile(currentDirectory.c_str(), filename, OpenMode::read);
			if (fileBeingSent != nullptr)
			{
				outBuf->printf("150 Opening data connection for %s (%lu bytes).\r\n", filename, fileBeingSent->Length());
				Commit(ResponderState::sendingPasvData);
			}
			else
			{
				outBuf->copy("550 Failed to open file.\r\n");
				Commit(ResponderState::reading);
			}
		}
		// abort current operation
		else if (StringEqualsIgnoreCase(clientMessage, "ABOR"))
		{
			CloseDataPort();

			outBuf->copy("226 ABOR successful.\r\n");
			Commit(ResponderState::reading);
		}
		// end connection
		else if (StringEqualsIgnoreCase(clientMessage, "QUIT"))
		{
			CloseDataPort();

			outBuf->copy("221 Goodbye.\r\n");
			Commit();
		}
		// unknown command
		else
		{
			outBuf->copy("500 Unknown command.\r\n");
			Commit(ResponderState::pasvPortOpened);
		}
		break;

	case ResponderState::uploading:
	case ResponderState::sendingPasvData:
		// abort current transfer
		if (StringEqualsIgnoreCase(clientMessage, "ABOR"))
		{
			CancelUpload();
			CloseDataPort();

			outBuf->copy("226 ABOR successful.\r\n");
			Commit(ResponderState::reading);
		}
		// in theory we could add support for QUIT here but it's not worth the effort
		// unknown command
		else
		{
			outBuf->copy("500 Unknown command.\r\n");
			Commit(responderState);
		}
		break;

	default:
		// do nothing
		break;
	}
}

const char *FtpResponder::GetParameter(const char *after) const
{
	const size_t commandLength = strlen(after);
	if (commandLength >= ftpMessageLength)
	{
		return "";
	}

	const char *result = clientMessage + strlen(after) + 1;
	while ((*result == '\t' || *result == ' ') && *result != 0)
	{
		++result;
	}
	return result;
}

void FtpResponder::ChangeDirectory(const char *newDirectory)
{
	String<MaxFilenameLength> combinedPath;
	if (newDirectory[0] != 0)
	{
		// Prepare the new directory path
		if (newDirectory[0] == '/')		// Absolute path
		{
			combinedPath.copy(newDirectory);
		}
		else if (StringEqualsIgnoreCase(newDirectory, "."))
		{
			combinedPath.copy(currentDirectory.c_str());
		}
		else if (StringEqualsIgnoreCase(newDirectory, ".."))	// Go up
		{
			// Check if we're already at the root directory
			if (StringEqualsIgnoreCase(currentDirectory.c_str(), "/"))
			{
				outBuf->copy("550 Failed to change directory.\r\n");
				Commit(responderState);
				return;
			}

			// No - find the parent directory
			combinedPath.copy(currentDirectory.c_str());
			for(int i = combinedPath.strlen() - 2; i >= 0; i--)
			{
				if (combinedPath[i] == '/')
				{
					combinedPath[i + 1] = 0;
					break;
				}
			}
		}
		else									// Go to child directory
		{
			combinedPath.copy(currentDirectory.c_str());
			if (!combinedPath.EndsWith('/') && combinedPath.strlen() > 1)
			{
				combinedPath.cat('/');
			}
			combinedPath.cat(newDirectory);
		}

		// Make sure the new path does not end with a slash, else FatFs won't be able to see the directory
		if (combinedPath.EndsWith('/') && combinedPath.strlen() > 1)
		{
			combinedPath[combinedPath.strlen() - 1] = 0;
		}

		// Verify the final path and change it if possible
		if (GetPlatform().GetMassStorage()->DirectoryExists(combinedPath.GetRef()))
		{
			currentDirectory.copy(combinedPath.c_str());
			outBuf->copy("250 Directory successfully changed.\r\n");
			Commit(responderState);
		}
		else
		{
			outBuf->copy("550 Failed to change directory.\r\n");
			Commit(responderState);
		}
	}
	else
	{
		outBuf->copy("550 Failed to change directory.\r\n");
		Commit(responderState);
	}
}

void FtpResponder::CloseDataPort()
{
	if (reprap.Debug(moduleWebserver))
	{
		debugPrintf("FTP data port is being closed\n");
	}

	if (dataSocket != nullptr)
	{
		dataSocket->Close();								// close it gracefully
		dataSocket = nullptr;
	}
	else if (skt != nullptr)
	{
		skt->GetInterface()->TerminateDataPort();			// in case it has been partially set up
	}

	OutputBuffer::ReleaseAll(dataBuf);

	if (fileBeingSent != nullptr)
	{
		fileBeingSent->Close();
		fileBeingSent = nullptr;
	}
}

/*static*/ void FtpResponder::InitStatic()
{
	// Nothing needed here
}

// This is called when we are shutting down the network or just this protocol. It may be called even if this protocol isn't enabled.
/*static*/ void FtpResponder::Disable()
{
	// Nothing needed here
}

#endif

// End
