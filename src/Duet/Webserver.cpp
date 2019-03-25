/****************************************************************************************************

 RepRapFirmware - Webserver

 This class serves a single-page web applications to the attached network.  This page forms the user's
 interface with the RepRap machine.  This software interprests returned values from the page and uses it
 to generate G Codes, which it sends to the RepRap.  It also collects values from the RepRap like
 temperature and uses those to construct the web page.

 The page itself - reprap.htm - uses Jquery.js to perform AJAX.  See:

 http://jquery.com/

 -----------------------------------------------------------------------------------------------------

 Version 0.2

 10 May 2013

 Adrian Bowyer
 RepRap Professional Ltd
 http://reprappro.com

 Licence: GPL

 -----------------------------------------------------------------------------------------------------

 The supported requests are GET requests for files (for which the root is the www directory on the
 SD card), and the following. These all start with "/rr_". Ordinary files used for the web interface
 must not have names starting "/rr_" or they will not be found. Times should be generally specified
 in the format YYYY-MM-DDTHH:MM:SS so the firmware can parse them.

 rr_connect?password=xxx&time=yyy
             Sent by the web interface software to establish an initial connection, indicating that
 	 	 	 any state variables relating to the web interface (e.g. file upload in progress) should
 	 	 	 be reset. This only happens if the password could be verified.

 rr_fileinfo Returns file information about the file being printed.

 rr_fileinfo?name=xxx
 	 	 	 Returns file information about a file on the SD card or a JSON-encapsulated response
 	 	 	 with err = 1 if the passed filename was invalid.

 rr_status	 New-style status response, in which temperatures, axis positions and extruder positions
 	 	 	 are returned in separate variables. Another difference is that extruder positions are
 	 	 	 returned as absolute positions instead of relative to the previous gcode. A client
 	 	 	 may also request different status responses by specifying the "type" keyword, followed
 	 	 	 by a custom status response type. Also see "M105 S1".

 rr_filelist?dir=xxx
 	 	 	 Returns a JSON-formatted list of all the files in xxx including the type and size in the
			 following format: "files":[{"type":'f/d',"name":"xxx",size:yyy},...]

 rr_files?dir=xxx&flagDirs={1/0} [DEPRECATED]
 	 	 	 Returns a listing of the filenames in the /gcode directory of the SD card. 'dir' is a
 	 	 	 directory path relative to the root of the SD card. If the 'dir' variable is not present,
 	 	 	 it defaults to the /gcode directory. If flagDirs is set to 1, all directories will be
			 prefixed by an asterisk.

 rr_reply    Returns the last-known G-code reply as plain text (not encapsulated as JSON).

 rr_configfile [DEPRECATED]
			 Sends the config file as plain text (not encapsulated as JSON either).

 rr_download?name=xxx
			 Download a specified file from the SD card

 rr_upload?name=xxx&time=yyy
 	 	 	 Upload a specified file using a POST request. The payload of this request has to be
 	 	 	 the file content. Only one file may be uploaded at once. When the upload has finished,
 	 	 	 a JSON response with the variable "err" will be returned, which will be 0 if the job
 	 	 	 has finished without problems, it will be set to 1 otherwise.

 rr_delete?name=xxx
			 Delete file xxx. Returns err (zero if successful).

 rr_mkdir?dir=xxx
			 Create a new directory xxx. Return err (zero if successful).

 rr_move?old=xxx&new=yyy
			 Rename an old file xxx to yyy. May also be used to move a file to another directory.

 ****************************************************************************************************/

#include "RepRapFirmware.h"
#include "Webserver.h"
#include "NetworkTransaction.h"
#include "Platform.h"
#include "Network.h"
#include "RepRap.h"
#include "GCodes/GCodes.h"
#include "PrintMonitor.h"

//***************************************************************************************************

const char* const overflowResponse = "overflow";
const char* const badEscapeResponse = "bad escape";

//**************************** Generic Webserver implementation ******************************

// Constructor and initialisation
Webserver::Webserver(Platform *p, Network *n) : platform(p), network(n), webserverActive(false)
{
	httpInterpreter = new HttpInterpreter(p, this, n);
	ftpInterpreter = new FtpInterpreter(p, this, n);
	telnetInterpreter = new TelnetInterpreter(p, this, n);
}

void Webserver::Init()
{
	// initialise the webserver class
	longWait = millis();
	webserverActive = true;
	readingConnection = NoConnection;

	// initialise all protocol handlers
	httpInterpreter->ResetState();
	ftpInterpreter->ResetState();
	telnetInterpreter->ResetState();
}

// Deal with input/output from/to the client (if any)
void Webserver::Spin()
{
	if (webserverActive)
	{
		// Check if we can actually send something back to the client
		if (OutputBuffer::GetBytesLeft(nullptr) == 0)
		{
			return;
		}

		// We must ensure that we have exclusive access to LWIP
		if (!network->Lock())
		{
			return;
		}

		// Allow each ProtocolInterpreter to do something
		httpInterpreter->Spin();
		ftpInterpreter->Spin();
		telnetInterpreter->Spin();

		// See if we have new data to process
		currentTransaction = network->GetTransaction(readingConnection);
		if (currentTransaction != nullptr)
		{
			// Take care of different protocol types here
			ProtocolInterpreter *interpreter;
			const uint16_t localPort = currentTransaction->GetLocalPort();
			if (localPort == Network::GetHttpPort())
			{
				interpreter = httpInterpreter;
			}
			else if (localPort == Network::GetTelnetPort())
			{
				interpreter = telnetInterpreter;
			}
			else
			{
				interpreter = ftpInterpreter;
			}

			// See if we have to print some debug info
			if (reprap.Debug(moduleWebserver))
			{
				const char *type;
				switch (currentTransaction->GetStatus())
				{
					case TransactionStatus::released: type = "released"; break;
					case TransactionStatus::connected: type = "connected"; break;
					case TransactionStatus::receiving: type = "receiving"; break;
					case TransactionStatus::sending: type = "sending"; break;
					case TransactionStatus::disconnected: type = "disconnected"; break;
					case TransactionStatus::deferred: type = "deferred"; break;
					case TransactionStatus::acquired: type = "acquired"; break;
					default: type = "unknown"; break;
				}
				platform->MessageF(UsbMessage, "Incoming transaction: Type %s at local port %d (remote port %d)\n",
						type, localPort, currentTransaction->GetRemotePort());
			}

			// For protocols other than HTTP it is important to send a HELO message
			TransactionStatus status = currentTransaction->GetStatus();
			if (status == TransactionStatus::connected)
			{
				interpreter->ConnectionEstablished();
			}
			// Graceful disconnects are handled here, because prior NetworkTransactions might still contain valid
			// data. That's why it's a bad idea to close these connections immediately in the Network class.
			else if (status == TransactionStatus::disconnected)
			{
				// This will call the disconnect events and effectively close the connection
				currentTransaction->Discard();
			}
			// Check for fast uploads via this connection
			else if (interpreter->DoingFastUpload())
			{
				interpreter->DoFastUpload();
			}
			// Process other messages (if we can)
			else if (interpreter->CanParseData())
			{
				readingConnection = currentTransaction->GetConnection();
				for(size_t i = 0; i < TCP_MSS / 3; i++)
				{
					char c;
					if (currentTransaction->Read(c))
					{
						// Each ProtocolInterpreter must take care of the current NetworkTransaction by
						// calling either Commit(), Discard() or Defer()
						if (interpreter->CharFromClient(c))
						{
							readingConnection = NoConnection;
							break;
						}
					}
					else
					{
						// We ran out of data before finding a complete request. This happens when the incoming
						// message length exceeds the TCP MSS. Notify the current ProtocolInterpreter about this,
						// which will remove the current transaction too
						interpreter->NoMoreDataAvailable();
						readingConnection = NoConnection;
						break;
					}
				}
			}
		}
		else if (readingConnection != NoConnection)
		{
			// We failed to find a transaction for a reading connection.
			// This should never happen, but if it does, terminate this connection instantly
			platform->Message(UsbMessage, "Error: Transaction for reading connection not found\n");
			Network::Terminate(readingConnection);
		}
		network->Unlock();		// unlock LWIP again
	}
}

void Webserver::Exit()
{
	httpInterpreter->CancelUpload();
	ftpInterpreter->CancelUpload();
	//telnetInterpreter->CancelUpload();		// Telnet doesn't support fast file uploads

	platform->Message(UsbMessage, "Webserver class exited.\n");
	webserverActive = false;
}

void Webserver::Diagnostics(MessageType mtype)
{
	platform->Message(mtype, "=== Webserver ===\n");
	httpInterpreter->Diagnostics(mtype);
	ftpInterpreter->Diagnostics(mtype);
	telnetInterpreter->Diagnostics(mtype);
}

void Webserver::HandleGCodeReply(const WebSource source, OutputBuffer *reply)
{
	switch (source)
	{
		case WebSource::HTTP:
			httpInterpreter->HandleGCodeReply(reply);
			break;

		case WebSource::Telnet:
			telnetInterpreter->HandleGCodeReply(reply);
			break;
	}
}

void Webserver::HandleGCodeReply(const WebSource source, const char *reply)
{
	switch (source)
	{
		case WebSource::HTTP:
			httpInterpreter->HandleGCodeReply(reply);
			break;

		case WebSource::Telnet:
			telnetInterpreter->HandleGCodeReply(reply);
			break;
	}
}

// Handle immediate disconnects here (cs will be freed after this call)
// May be called by ISR, but not while LwIP is NOT locked
void Webserver::ConnectionLost(Connection conn)
{
	// Inform protocol handlers that this connection has been lost
	const uint16_t localPort = Network::GetLocalPort(conn);
	ProtocolInterpreter *interpreter;
	if (localPort == Network::GetHttpPort())
	{
		interpreter = httpInterpreter;
	}
	else if (localPort == Network::GetFtpPort() || localPort == network->GetDataPort())
	{
		interpreter = ftpInterpreter;
	}
	else if (localPort == Network::GetTelnetPort())
	{
		interpreter = telnetInterpreter;
	}
	else
	{
		platform->MessageF(GenericMessage, "Error: Webserver should handle disconnect event at local port %d, but no handler was found!\n", localPort);
		return;
	}

	// Print some debug information and notify the protocol interpreter
	if (reprap.Debug(moduleWebserver))
	{
		platform->MessageF(UsbMessage, "ConnectionLost called for local port %d (remote port %d)\n", localPort, Network::GetRemotePort(conn));
	}
	interpreter->ConnectionLost(conn);

	// Don't process any more data from this connection if has gone down
	if (readingConnection == conn)
	{
		readingConnection = NoConnection;
	}
}


//********************************************************************************************
//
//********************** Generic Procotol Interpreter implementation *************************
//
//********************************************************************************************

ProtocolInterpreter::ProtocolInterpreter(Platform *p, Webserver *ws, Network *n)
	: platform(p), webserver(ws), network(n)
{
	uploadState = notUploading;
	filenameBeingUploaded[0] = 0;
}

void ProtocolInterpreter::Spin()
{
	// Check if anything went wrong while writing upload data, and delete the file again if that is the case
	if (uploadState == uploadError)
	{
		if (fileBeingUploaded.IsLive())
		{
			fileBeingUploaded.Close();
		}
		if (filenameBeingUploaded[0] != 0)
		{
			platform->Delete(FS_PREFIX, filenameBeingUploaded);
		}

		uploadState = notUploading;
		filenameBeingUploaded[0] = 0;
	}
}

void ProtocolInterpreter::ConnectionEstablished()
{
	// Don't care about incoming connections by default
	webserver->currentTransaction->Discard();
}

void ProtocolInterpreter::NoMoreDataAvailable()
{
	// Request is not complete yet, but don't care. Interpreters that do not explicitly
	// overwrite this method don't support more than one connected client anyway
	webserver->currentTransaction->Discard();
}

// Start writing to a new file
bool ProtocolInterpreter::StartUpload(FileStore *file, const char *fileName)
{
	if (file != nullptr)
	{
		fileBeingUploaded.Set(file);
		SafeStrncpy(filenameBeingUploaded, fileName, ARRAY_SIZE(filenameBeingUploaded));
		filenameBeingUploaded[ARRAY_UPB(filenameBeingUploaded)] = 0;

		uploadState = uploadOK;
		return true;
	}

	platform->Message(GenericMessage, "Error: Could not open file while starting upload!\n");
	return false;
}

void ProtocolInterpreter::CancelUpload()
{
	if (uploadState == uploadOK)
	{
		// Do the file handling next time when Spin is called
		uploadState = uploadError;
	}
}

void ProtocolInterpreter::DoFastUpload()
{
	NetworkTransaction *transaction = webserver->currentTransaction;

	const char *buffer;
	size_t len;
	if (transaction->ReadBuffer(buffer, len))
	{
		// See if we can output a debug message
		if (reprap.Debug(moduleWebserver))
		{
			platform->MessageF(UsbMessage, "Writing %u bytes of upload data\n", len);
		}

		// Writing data usually takes a while, so keep LwIP running while this is being done
		network->Unlock();
		if (!fileBeingUploaded.Write(buffer, len))
		{
			platform->Message(GenericMessage, "Error: Could not write upload data!\n");
			CancelUpload();

			while (!network->Lock());
			transaction->Commit(false);
			return;
		}
		while (!network->Lock());
	}

	if (uploadState != uploadOK || !transaction->HasMoreDataToRead())
	{
		transaction->Discard();
	}
}

bool ProtocolInterpreter::FinishUpload(uint32_t fileLength)
{
	// Flush remaining data for FSO
	if (uploadState == uploadOK && !fileBeingUploaded.Flush())
	{
		uploadState = uploadError;
		platform->Message(GenericMessage, "Error: Could not flush remaining data while finishing upload!\n");
	}

	// Check the file length is as expected
	if (uploadState == uploadOK && fileLength != 0 && fileBeingUploaded.Length() != fileLength)
	{
		uploadState = uploadError;
		platform->MessageF(GenericMessage, "Error: Uploaded file size is different (%" PRIu32 " vs. expected %" PRIu32 " bytes)!\n", fileBeingUploaded.Length(), fileLength);
	}

	// Close the file
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
	}

	// Delete the file again if an error has occurred
	if (uploadState == uploadError && filenameBeingUploaded[0] != 0)
	{
		platform->Delete(FS_PREFIX, filenameBeingUploaded);
	}

	// Clean up again
	bool success = (uploadState == uploadOK);
	uploadState = notUploading;
	filenameBeingUploaded[0] = 0;
	return success;
}


//********************************************************************************************
//
// *********************** HTTP interpreter for the Webserver class **************************
//
//********************************************************************************************



Webserver::HttpInterpreter::HttpInterpreter(Platform *p, Webserver *ws, Network *n)
	: ProtocolInterpreter(p, ws, n), state(doingCommandWord), numSessions(0), clientsServed(0)
{
	gcodeReply = new OutputStack();
	deferredRequestConnection = NoConnection;
	seq = 0;
}

void Webserver::HttpInterpreter::Diagnostics(MessageType mt)
{
	platform->MessageF(mt, "HTTP sessions: %d of %d\n", numSessions, maxHttpSessions);
}

void Webserver::HttpInterpreter::Spin()
{
	// Deal with aborted uploads
	ProtocolInterpreter::Spin();

	// Verify HTTP sessions
	const uint32_t now = millis();
	for(int i = (int)numSessions - 1; i >= 0; i--)
	{
		if (sessions[i].isPostUploading)
		{
			// Check for cancelled POST uploads
			if (uploadState != uploadOK)
			{
				sessions[i].isPostUploading = false;
				sessions[i].lastQueryTime = millis();
			}
		}
		else if ((now - sessions[i].lastQueryTime) > httpSessionTimeout)
		{
			// Check for timed out sessions
			for(size_t k = i + 1; k < numSessions; k++)
			{
				memcpy(&sessions[k - 1], &sessions[k], sizeof(HttpSession));
			}
			numSessions--;
			clientsServed++;	// assume the disconnected client hasn't fetched the G-Code reply yet
		}
	}

	// If we cannot send the G-Code reply to anyone, we may free up some run-time space by dumping it
	if (numSessions == 0 || clientsServed >= numSessions)
	{
		while (!gcodeReply->IsEmpty())
		{
			OutputBuffer *buf = gcodeReply->Pop();
			OutputBuffer::ReleaseAll(buf);
		}
		clientsServed = 0;
	}

}

// File Uploads

bool Webserver::HttpInterpreter::DoingFastUpload() const
{
	uint32_t remoteIP = webserver->currentTransaction->GetRemoteIP();
	uint16_t remotePort = webserver->currentTransaction->GetRemotePort();
	for(size_t i = 0; i < numSessions; i++)
	{
		if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
		{
			// There is only one session per IP address...
			return (sessions[i].postPort == remotePort);
		}
	}
	return false;
}

// Write some data on the SD card
void Webserver::HttpInterpreter::DoFastUpload()
{
	NetworkTransaction * const transaction = webserver->currentTransaction;
	const char *buffer;
	size_t len;
	if (transaction->ReadBuffer(buffer, len))
	{
		network->Unlock();
		const bool success = fileBeingUploaded.Write(buffer, len);
		if (!success)
		{
			platform->Message(GenericMessage, "Error: Could not write upload data!\n");
			CancelUpload();

			while (!network->Lock()) { }
			SendJsonResponse("upload");
			return;
		}
		uploadedBytes += len;
		while (!network->Lock()) { }
	}

	// See if the upload has finished
	if (uploadState == uploadOK && uploadedBytes >= postFileLength)
	{
		// Reset POST upload state for this client
		const uint32_t remoteIP = transaction->GetRemoteIP();
		for(size_t i = 0; i < numSessions; i++)
		{
			if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
			{
				sessions[i].isPostUploading = false;
				sessions[i].lastQueryTime = millis();
				break;
			}
		}

		// Grab a copy of the filename and finish this upload
		char filename[MaxFilenameLength];
		SafeStrncpy(filename, filenameBeingUploaded, MaxFilenameLength);
		FinishUpload(postFileLength);

		// Update the file timestamp if it was specified before
		if (fileLastModified != 0)
		{
			(void)platform->GetMassStorage()->SetLastModifiedTime(filename, fileLastModified);
		}

		// Eventually send the JSON response
		SendJsonResponse("upload");
	}
	else if (uploadState != uploadOK || !transaction->HasMoreDataToRead())
	{
		// We cannot read any more, discard the transaction again
		transaction->Discard();
	}
}

// Output to the client

// Start sending a file or a JSON response.
void Webserver::HttpInterpreter::SendFile(const char* nameOfFileToSend, bool isWebFile)
{
	NetworkTransaction *transaction = webserver->currentTransaction;
	FileStore *fileToSend = nullptr;
	bool zip = false;

	if (isWebFile)
	{
		if (nameOfFileToSend[0] == '/')
		{
			++nameOfFileToSend;						// all web files are relative to the /www folder, so remove the leading '/'
		}

		if (nameOfFileToSend[0] == 0)
		{
			nameOfFileToSend = INDEX_PAGE_FILE;
		}

		for (;;)
		{
			// Try to open a gzipped version of the file first
			if (!StringEndsWithIgnoreCase(nameOfFileToSend, ".gz") && strlen(nameOfFileToSend) + 3 <= MaxFilenameLength)
			{
				char nameBuf[MaxFilenameLength + 1];
				strcpy(nameBuf, nameOfFileToSend);
				strcat(nameBuf, ".gz");
				fileToSend = platform->OpenFile(platform->GetWebDir(), nameBuf, OpenMode::read);
				if (fileToSend != nullptr)
				{
					zip = true;
					break;
				}
			}

			// That failed, try to open the normal version of the file
			fileToSend = platform->OpenFile(platform->GetWebDir(), nameOfFileToSend, OpenMode::read);
			if (fileToSend != nullptr)
			{
				break;
			}

			if (StringEqualsIgnoreCase(nameOfFileToSend, INDEX_PAGE_FILE))
			{
				nameOfFileToSend = OLD_INDEX_PAGE_FILE;			// the index file wasn't found, so try the old one
			}
			else if (!strchr(nameOfFileToSend, '.'))			// if we were asked to return a file without a '.' in the name, return the index page
			{
				nameOfFileToSend = INDEX_PAGE_FILE;
			}
			else
			{
				break;
			}
		}

		// If we still couldn't find the file and it was an HTML file, return the 404 error page
		if (fileToSend == nullptr && (StringEndsWithIgnoreCase(nameOfFileToSend, ".html") || StringEndsWithIgnoreCase(nameOfFileToSend, ".htm")))
		{
			nameOfFileToSend = FOUR04_PAGE_FILE;
			fileToSend = platform->OpenFile(platform->GetWebDir(), nameOfFileToSend, OpenMode::read);
		}
	}
	else
	{
		fileToSend = platform->OpenFile(FS_PREFIX, nameOfFileToSend, OpenMode::read);
	}

	if (fileToSend == nullptr)
	{
		RejectMessage("not found", 404);
		return;
	}

	transaction->SetFileToWrite(fileToSend);
	transaction->Write("HTTP/1.1 200 OK\r\n");

	// Don't cache files served by rr_download
	if (!isWebFile)
	{
		transaction->Write("Cache-Control: no-cache, no-store, must-revalidate\r\n");
		transaction->Write("Pragma: no-cache\r\n");
		transaction->Write("Expires: 0\r\n");
		transaction->Write("Access-Control-Allow-Origin: *\r\n");
	}

	const char* contentType;
	if (StringEndsWithIgnoreCase(nameOfFileToSend, ".png"))
	{
		contentType = "image/png";
	}
	else if (StringEndsWithIgnoreCase(nameOfFileToSend, ".ico"))
	{
		contentType = "image/x-icon";
	}
	else if (StringEndsWithIgnoreCase(nameOfFileToSend, ".js"))
	{
		contentType = "application/javascript";
	}
	else if (StringEndsWithIgnoreCase(nameOfFileToSend, ".css"))
	{
		contentType = "text/css";
	}
	else if (StringEndsWithIgnoreCase(nameOfFileToSend, ".htm") || StringEndsWithIgnoreCase(nameOfFileToSend, ".html"))
	{
		contentType = "text/html";
	}
	else if (StringEndsWithIgnoreCase(nameOfFileToSend, ".zip"))
	{
		contentType = "application/zip";
		zip = true;
	}
	else if (StringEndsWithIgnoreCase(nameOfFileToSend, ".g") || StringEndsWithIgnoreCase(nameOfFileToSend, ".gc") || StringEndsWithIgnoreCase(nameOfFileToSend, ".gcode"))
	{
		contentType = "text/plain";
	}
	else
	{
		contentType = "application/octet-stream";
	}
	transaction->Printf("Content-Type: %s\r\n", contentType);

	if (zip)
	{
		transaction->Write("Content-Encoding: gzip\r\n");
	}

	transaction->Printf("Content-Length: %lu\r\n", fileToSend->Length());
	transaction->Write("Connection: close\r\n\r\n");
	transaction->Commit(false);
}

void Webserver::HttpInterpreter::SendGCodeReply()
{
	// Do we need to keep the G-Code reply for other clients?
	bool clearReply = false;
	if (!gcodeReply->IsEmpty())
	{
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
			platform->MessageF(UsbMessage, "Sending G-Code reply to client %d of %d (length %u)\n", clientsServed, numSessions, gcodeReply->DataLength());
		}
	}

	// Send the whole G-Code reply as plain text to the client
	NetworkTransaction *transaction = webserver->currentTransaction;
	transaction->Write("HTTP/1.1 200 OK\r\n");
	transaction->Write("Cache-Control: no-cache, no-store, must-revalidate\r\n");
	transaction->Write("Pragma: no-cache\r\n");
	transaction->Write("Expires: 0\r\n");
	transaction->Write("Access-Control-Allow-Origin: *\r\n");
	transaction->Write("Content-Type: text/plain\r\n");
	transaction->Printf("Content-Length: %u\r\n", gcodeReply->DataLength());
	transaction->Write("Connection: close\r\n\r\n");
	transaction->Write(gcodeReply);
	transaction->Commit(false);

	// Possibly clean up the G-code reply once again
	if (clearReply)
	{
		gcodeReply->Clear();
	}
}

void Webserver::HttpInterpreter::SendJsonResponse(const char* command)
{
	// Try to authorize the user automatically to retain compatibility with the old web interface
	if (!IsAuthenticated() && reprap.NoPasswordSet())
	{
		Authenticate();
	}

	// Update the authentication status and try handle "text/plain" requests here
	if (IsAuthenticated())
	{
		UpdateAuthentication();

		if (StringEqualsIgnoreCase(command, "reply"))			// rr_reply
		{
			SendGCodeReply();
			return;
		}

		if (StringEqualsIgnoreCase(command, "configfile"))	// rr_configfile [DEPRECATED]
		{
			String<MaxFilenameLength> fileName;
			MassStorage::CombineName(fileName.GetRef(), DEFAULT_SYS_DIR, platform->GetConfigFile());
			SendFile(fileName.c_str(), false);
			return;
		}

		if (StringEqualsIgnoreCase(command, "download"))
		{
			const char* const filename = GetKeyValue("name");
			if (filename != nullptr)
			{
				SendFile(filename, false);
				return;
			}
		}
	}

	// Try to process a request for JSON responses
	OutputBuffer *jsonResponse;
	if (!OutputBuffer::Allocate(jsonResponse))
	{
		// Reset the connection immediately if we cannot write any data. Should never happen
		Network::Terminate(webserver->currentTransaction->GetConnection());
		return;
	}

	bool mayKeepOpen;
	GetJsonResponse(command, jsonResponse, mayKeepOpen);

	// Check special cases of deferred requests (rr_fileinfo) and rejected messages
	NetworkTransaction *transaction = webserver->currentTransaction;
	if (transaction->GetStatus() == TransactionStatus::deferred || transaction->GetStatus() == TransactionStatus::sending)
	{
		OutputBuffer::Release(jsonResponse);
		return;
	}

	// Send the JSON response
	bool keepOpen = false;
	if (mayKeepOpen)
	{
		// Check that the browser wants to persist the connection too
		for (size_t i = 0; i < numHeaderKeys; ++i)
		{
			if (StringEqualsIgnoreCase(headers[i].key, "Connection"))
			{
				// Comment out the following line to disable persistent connections
				keepOpen = StringEqualsIgnoreCase(headers[i].value, "keep-alive");
				break;
			}
		}
	}

	transaction->Write("HTTP/1.1 200 OK\r\n");
	transaction->Write("Cache-Control: no-cache, no-store, must-revalidate\r\n");
	transaction->Write("Pragma: no-cache\r\n");
	transaction->Write("Expires: 0\r\n");
	transaction->Write("Access-Control-Allow-Origin: *\r\n");
	transaction->Write("Content-Type: application/json\r\n");
	transaction->Printf("Content-Length: %u\r\n", (jsonResponse != nullptr) ? jsonResponse->Length() : 0);
	transaction->Printf("Connection: %s\r\n\r\n", keepOpen ? "keep-alive" : "close");
	transaction->Write(jsonResponse);

	transaction->Commit(keepOpen);
}

//----------------------------------------------------------------------------------------------------

// Input from the client

// Get the Json response for this command.
// 'value' is null-terminated, but we also pass its length in case it contains embedded nulls, which matters when uploading files.
void Webserver::HttpInterpreter::GetJsonResponse(const char* request, OutputBuffer *&response, bool& keepOpen)
{
	keepOpen = false;	// assume we don't want to persist the connection

	if (StringEqualsIgnoreCase(request, "connect") && GetKeyValue("password") != nullptr)
	{
		if (IsAuthenticated() || reprap.CheckPassword(GetKeyValue("password")))
		{
			// Password OK
			if (Authenticate())
			{
				// See if we can update the current RTC date and time
				const char* const timeString = GetKeyValue("time");
				if (timeString != nullptr && !platform->IsDateTimeSet())
				{
					struct tm timeInfo;
					memset(&timeInfo, 0, sizeof(timeInfo));
					if (strptime(timeString, "%Y-%m-%dT%H:%M:%S", &timeInfo) != nullptr)
					{
						time_t newTime = mktime(&timeInfo);
						platform->SetDateTime(newTime);
					}
				}

				// Client has been logged in
				response->printf("{\"err\":0,\"sessionTimeout\":%" PRIu32 ",\"boardType\":\"%s\"}", httpSessionTimeout, platform->GetBoardString());
			}
			else
			{
				// No more HTTP sessions available
				response->copy("{\"err\":2}");
			}
		}
		else
		{
			// Wrong password
			response->copy("{\"err\":1}");
		}
	}
	else if (!IsAuthenticated())
	{
		RejectMessage("Not authorized", 401);
	}
	else if (StringEqualsIgnoreCase(request, "disconnect"))
	{
		response->printf("{\"err\":%d}", RemoveAuthentication() ? 0 : 1);
	}
	else if (StringEqualsIgnoreCase(request, "status"))
	{
		int type = 0;
		if (GetKeyValue("type") != nullptr)
		{
			// New-style JSON status responses
			type = atoi(GetKeyValue("type"));
			if (type < 1 || type > 3)
			{
				type = 1;
			}

			OutputBuffer::Release(response);
			response = reprap.GetStatusResponse(type, ResponseSource::HTTP);
		}
		else
		{
			// Deprecated
			OutputBuffer::Release(response);
			response = reprap.GetLegacyStatusResponse(1, 0);
		}
	}
	else if (StringEqualsIgnoreCase(request, "gcode") && GetKeyValue("gcode") != nullptr)
	{
		NetworkGCodeInput * const httpInput = reprap.GetGCodes().GetHTTPInput();
		httpInput->Put(HttpMessage, GetKeyValue("gcode"));
		response->printf("{\"buff\":%u}", httpInput->BufferSpaceLeft());
	}
	else if (StringEqualsIgnoreCase(request, "upload"))
	{
		response->printf("{\"err\":%d}", (uploadedBytes == postFileLength) ? 0 : 1);
	}
	else if (StringEqualsIgnoreCase(request, "delete") && GetKeyValue("name") != nullptr)
	{
		const bool ok = platform->Delete(FS_PREFIX, GetKeyValue("name"));
		response->printf("{\"err\":%d}", (ok) ? 0 : 1);
	}
	else if (StringEqualsIgnoreCase(request, "filelist") && GetKeyValue("dir") != nullptr)
	{
		OutputBuffer::Release(response);
		const char* const firstVal = GetKeyValue("first");
		const unsigned int startAt = (firstVal == nullptr) ? 0 : (unsigned int)SafeStrtol(firstVal);
		response = reprap.GetFilelistResponse(GetKeyValue("dir"), startAt);		// this may return nullptr
	}
	else if (StringEqualsIgnoreCase(request, "files"))
	{
		OutputBuffer::Release(response);
		const char* dir = GetKeyValue("dir");
		if (dir == nullptr)
		{
			dir = platform->GetGCodeDir();
		}
		const char* const firstVal = GetKeyValue("first");
		const unsigned int startAt = (firstVal == nullptr) ? 0 : SafeStrtol(firstVal);
		const char* const flagDirsVal = GetKeyValue("flagDirs");
		const bool flagDirs = flagDirsVal != nullptr && atoi(flagDirsVal) == 1;
		response = reprap.GetFilesResponse(dir, startAt, flagDirs);				// this may return nullptr
	}
	else if (StringEqualsIgnoreCase(request, "fileinfo"))
	{
		if (deferredRequestConnection != NoConnection)
		{
			// Don't allow multiple deferred requests to be processed at once
			webserver->currentTransaction->Defer(DeferralMode::ResetData);
		}
		else
		{
			const char* const nameVal = GetKeyValue("name");
			if (nameVal != nullptr)
			{
				// Regular rr_fileinfo?name=xxx call
				SafeStrncpy(filenameBeingProcessed, nameVal, ARRAY_SIZE(filenameBeingProcessed));
				filenameBeingProcessed[ARRAY_UPB(filenameBeingProcessed)] = 0;
			}
			else
			{
				// Simple rr_fileinfo call to get info about the file being printed
				filenameBeingProcessed[0] = 0;
			}

			deferredRequestConnection = webserver->currentTransaction->GetConnection();
			ProcessDeferredRequest();
		}
	}
	else if (StringEqualsIgnoreCase(request, "move"))
	{
		const char* const oldVal = GetKeyValue("old");
		const char* const newVal = GetKeyValue("new");
		bool success = false;
		if (oldVal != nullptr && newVal != nullptr)
		{
			MassStorage * const ms = platform->GetMassStorage();
			if (StringEqualsIgnoreCase(GetKeyValue("deleteexisting"), "yes") && ms->FileExists(oldVal) && ms->FileExists(newVal))
			{
				ms->Delete(newVal);
			}
			success = ms->Rename(oldVal, newVal);
		}
		response->printf("{\"err\":%d}", (success) ? 0 : 1);
	}
	else if (StringEqualsIgnoreCase(request, "mkdir"))
	{
		const char* const dirVal = GetKeyValue("dir");
		bool success = false;
		if (dirVal != nullptr)
		{
			success = platform->GetMassStorage()->MakeDirectory(dirVal);
		}
		response->printf("{\"err\":%d}", (success) ? 0 : 1);
	}
	else if (StringEqualsIgnoreCase(request, "config"))
	{
		OutputBuffer::Release(response);
		response = reprap.GetConfigResponse();
	}
	else
	{
		RejectMessage("Unknown request", 500);
	}
}

const char* Webserver::HttpInterpreter::GetKeyValue(const char *key) const
{
	for (size_t i = 0; i < numQualKeys; ++i)
	{
		if (StringEqualsIgnoreCase(qualifiers[i].key, key))
		{
			return qualifiers[i].value;
		}
	}
	return nullptr;
}

void Webserver::HttpInterpreter::ResetState()
{
	clientPointer = 0;
	state = doingCommandWord;
	numCommandWords = 0;
	numQualKeys = 0;
	numHeaderKeys = 0;
	commandWords[0] = clientMessage;
}

void Webserver::HttpInterpreter::NoMoreDataAvailable()
{
	RejectMessage("Incomplete or too long HTTP request", 500);
}

// May be called from ISR!
void Webserver::HttpInterpreter::ConnectionLost(Connection conn)
{
	// Make sure deferred requests are cancelled
	if (deferredRequestConnection == conn)
	{
		deferredRequestConnection = NoConnection;
	}

	// If we couldn't read an entire request from a connection, reset our state here again
	if (webserver->readingConnection == conn)
	{
		ResetState();
	}

	// Deal with aborted POST uploads. Note that we also check the remote port here,
	// because the client *might* have two instances of the web interface running.
	if (uploadState == uploadOK)
	{
		const uint32_t remoteIP = Network::GetRemoteIP(conn);
		const uint16_t remotePort = Network::GetRemotePort(conn);
		for(size_t i = 0; i < numSessions; i++)
		{
			if (sessions[i].ip == remoteIP && sessions[i].isPostUploading && sessions[i].postPort == remotePort)
			{
				if (reprap.Debug(moduleWebserver))
				{
					platform->MessageF(UsbMessage, "POST upload for '%s' has been cancelled!\n", filenameBeingUploaded);
				}
				sessions[i].isPostUploading = false;
				CancelUpload();
				break;
			}
		}
	}
}

bool Webserver::HttpInterpreter::CanParseData()
{
	// We want to send a response, but we need memory for that. Check if we have to truncate the G-Code reply
	while (OutputBuffer::GetBytesLeft(nullptr) < minHttpResponseSize)
	{
		if (gcodeReply->IsEmpty())
		{
			// We cannot truncate any G-Code reply and don't have enough free space, try again later
			return false;
		}

		if (OutputBuffer::Truncate(gcodeReply->GetFirstItem(), minHttpResponseSize) == 0)
		{
			// Truncating didn't work out, but see if we can free up a few more bytes by releasing the first reply item
			OutputBuffer *buf = gcodeReply->Pop();
			OutputBuffer::ReleaseAll(buf);
		}
	}

	// Are we still processing a deferred request?
	if (deferredRequestConnection == webserver->currentTransaction->GetConnection())
	{
		if (Network::IsConnected(deferredRequestConnection))
		{
			// Process more of this request. If it doesn't finish this time, it will be appended to the list
			// of ready transactions again, which will ensure it can be processed later again
			ProcessDeferredRequest();
		}
		else
		{
			// Don't bother with this request if the connection has been closed.
			// We expect a "disconnected" transaction to report this later, so don't clean up anything here
			webserver->currentTransaction->Discard();
		}
		return false;
	}

	return true;
}

// Process a character from the client
// Rewritten as a state machine by dc42 to increase capability and speed, and reduce RAM requirement.
// On entry:
//  There is space for at least 1 character in clientMessage.
// On return:
//	If we return false:
//		We want more characters. There is space for at least 1 character in clientMessage.
//	If we return true:
//		We have processed the message and sent the reply. No more characters may be read from this message.
// Whenever this calls ProcessMessage:
//	The first line has been split up into words. Variables numCommandWords and commandWords give the number of words we found
//  and the pointers to each word. The second word is treated specially. It is assumed to be a filename followed by an optional
//  qualifier comprising key/value pairs. Both may include %xx escapes, and the qualifier may include + to mean space. We store
//  a pointer to the filename without qualifier in commandWords[1]. We store the qualifier key/value pointers in array 'qualifiers'
//  and the number of them in numQualKeys.
//  The remaining lines have been parsed as header name/value pairs. Pointers to them are stored in array 'headers' and the number
//  of them in numHeaders.
// If one of our arrays is about to overflow, or the message is not in a format we expect, then we call RejectMessage with an
// appropriate error code and string.
bool Webserver::HttpInterpreter::CharFromClient(char c)
{
	switch(state)
	{
		case doingCommandWord:
			switch(c)
			{
				case '\n':
					clientMessage[clientPointer++] = 0;
					++numCommandWords;
					numHeaderKeys = 0;
					headers[0].key = clientMessage + clientPointer;
					state = doingHeaderKey;
					break;
				case '\r':
					break;
				case ' ':
				case '\t':
					clientMessage[clientPointer++] = 0;
					if (numCommandWords < maxCommandWords)
					{
						++numCommandWords;
						commandWords[numCommandWords] = clientMessage + clientPointer;
						if (numCommandWords == 1)
						{
							state = doingFilename;
						}
					}
					else
					{
						return RejectMessage("too many command words");
					}
					break;
				default:
					clientMessage[clientPointer++] = c;
					break;
			}
			break;

		case doingFilename:
			switch(c)
			{
				case '\n':
					clientMessage[clientPointer++] = 0;
					++numCommandWords;
					numQualKeys = 0;
					numHeaderKeys = 0;
					headers[0].key = clientMessage + clientPointer;
					state = doingHeaderKey;
					break;
				case '?':
					clientMessage[clientPointer++] = 0;
					++numCommandWords;
					numQualKeys = 0;
					qualifiers[0].key = clientMessage + clientPointer;
					state = doingQualifierKey;
					break;
				case '%':
					state = doingFilenameEsc1;
					break;
				case '\r':
					break;
				case ' ':
				case '\t':
					clientMessage[clientPointer++] = 0;
					if (numCommandWords < maxCommandWords)
					{
						++numCommandWords;
						commandWords[numCommandWords] = clientMessage + clientPointer;
						state = doingCommandWord;
					}
					else
					{
						return RejectMessage("too many command words");
					}
					break;
				default:
					clientMessage[clientPointer++] = c;
					break;
			}
			break;

		case doingQualifierKey:
			switch(c)
			{
				case '=':
					clientMessage[clientPointer++] = 0;
					qualifiers[numQualKeys].value = clientMessage + clientPointer;
					++numQualKeys;
					state = doingQualifierValue;
					break;
				case '\n':	// key with no value
				case ' ':
				case '\t':
				case '\r':
				case '%':	// none of our keys needs escaping, so treat an escape within a key as an error
				case '&':	// key with no value
					return RejectMessage("bad qualifier key");
				default:
					clientMessage[clientPointer++] = c;
					break;
			}
			break;

		case doingQualifierValue:
			switch(c)
			{
				case '\n':
					clientMessage[clientPointer++] = 0;
					qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
					numHeaderKeys = 0;
					headers[0].key = clientMessage + clientPointer;
					state = doingHeaderKey;
					break;
				case ' ':
				case '\t':
					clientMessage[clientPointer++] = 0;
					qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
					commandWords[numCommandWords] = clientMessage + clientPointer;
					state = doingCommandWord;
					break;
				case '\r':
					break;
				case '%':
					state = doingQualifierValueEsc1;
					break;
				case '&':
					// Another variable is coming
					clientMessage[clientPointer++] = 0;
					qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
					if (numQualKeys < maxQualKeys)
					{
						state = doingQualifierKey;
					}
					else
					{
						return RejectMessage("too many keys in qualifier");
					}
					break;
				case '+':
					clientMessage[clientPointer++] = ' ';
					break;
				default:
					clientMessage[clientPointer++] = c;
					break;
			}
			break;

		case doingFilenameEsc1:
		case doingQualifierValueEsc1:
			if (c >= '0' && c <= '9')
			{
				decodeChar = (c - '0') << 4;
				state = (HttpState)(state + 1);
			}
			else if (c >= 'A' && c <= 'F')
			{
				decodeChar = (c - ('A' - 10)) << 4;
				state = (HttpState)(state + 1);
			}
			else
			{
				return RejectMessage(badEscapeResponse);
			}
			break;

		case doingFilenameEsc2:
		case doingQualifierValueEsc2:
			if (c >= '0' && c <= '9')
			{
				clientMessage[clientPointer++] = decodeChar | (c - '0');
				state = (HttpState)(state - 2);
			}
			else if (c >= 'A' && c <= 'F')
			{
				clientMessage[clientPointer++] = decodeChar | (c - ('A' - 10));
				state = (HttpState)(state - 2);
			}
			else
			{
				return RejectMessage(badEscapeResponse);
			}
			break;

		case doingHeaderKey:
			switch(c)
			{
				case '\n':
					if (clientMessage + clientPointer == headers[numHeaderKeys].key)	// if the key hasn't started yet, then this is the blank line at the end
					{
						if (ProcessMessage())
						{
							return true;
						}
					}
					else
					{
						return RejectMessage("unexpected newline");
					}
					break;
				case '\r':
					break;
				case ':':
					if (numHeaderKeys == maxHeaders - 1)
					{
						return RejectMessage("too many header key-value pairs");
					}
					clientMessage[clientPointer++] = 0;
					headers[numHeaderKeys].value = clientMessage + clientPointer;
					++numHeaderKeys;
					state = expectingHeaderValue;
					break;
				default:
					clientMessage[clientPointer++] = c;
					break;
			}
			break;

		case expectingHeaderValue:
			if (c == ' ' || c == '\t')
			{
				break;		// ignore spaces between header key and value
			}
			state = doingHeaderValue;
			// no break

		case doingHeaderValue:
			if (c == '\n')
			{
				state = doingHeaderContinuation;
			}
			else if (c != '\r')
			{
				clientMessage[clientPointer++] = c;
			}
			break;

		case doingHeaderContinuation:
			switch(c)
			{
				case ' ':
				case '\t':
					// It's a continuation of the previous value
					clientMessage[clientPointer++] = c;
					state = doingHeaderValue;
					break;
				case '\n':
					// It's the blank line
					clientMessage[clientPointer] = 0;
					if (ProcessMessage())
					{
						return true;
					}
					// no break
				case '\r':
					break;
				default:
					// It's a new key
					if (clientPointer + 3 <= ARRAY_SIZE(clientMessage))
					{
						clientMessage[clientPointer++] = 0;
						headers[numHeaderKeys].key = clientMessage + clientPointer;
						clientMessage[clientPointer++] = c;
						state = doingHeaderKey;
					}
					else
					{
						return RejectMessage(overflowResponse);
					}
					break;
			}
			break;

		default:
			break;
	}

	if (clientPointer == ARRAY_SIZE(clientMessage))
	{
		return RejectMessage(overflowResponse);
	}
	return false;
}

// Process the message received so far. We have reached the end of the headers.
// Return true if the message is complete, false if we want to continue receiving data (i.e. postdata)
bool Webserver::HttpInterpreter::ProcessMessage()
{
	if (reprap.Debug(moduleWebserver))
	{
		platform->Message(UsbMessage, "HTTP req, command words {");
		for (size_t i = 0; i < numCommandWords; ++i)
		{
			platform->MessageF(UsbMessage, " %s", commandWords[i]);
		}
		platform->Message(UsbMessage, " }, parameters {");

		for (size_t i = 0; i < numQualKeys; ++i)
		{
			platform->MessageF(UsbMessage, " %s=%s", qualifiers[i].key, qualifiers[i].value);
		}
		platform->Message(UsbMessage, " }\n");
	}

	if (numCommandWords < 2)
	{
		return RejectMessage("too few command words");
	}

	if (StringEqualsIgnoreCase(commandWords[0], "GET"))
	{
		if (StringStartsWith(commandWords[1], KO_START))
		{
			SendJsonResponse(commandWords[1] + KO_FIRST);
		}
		else if (commandWords[1][0] == '/' && StringStartsWith(commandWords[1] + 1, KO_START))
		{
			SendJsonResponse(commandWords[1] + 1 + KO_FIRST);
		}
		else
		{
			SendFile(commandWords[1], true);
		}

		ResetState();
		return true;
	}

	if (StringEqualsIgnoreCase(commandWords[0], "OPTIONS"))
	{
		NetworkTransaction *transaction = webserver->currentTransaction;

		transaction->Write("HTTP/1.1 200 OK\r\n");
		transaction->Write("Allow: OPTIONS, GET, POST\r\n");
		transaction->Write("Cache-Control: no-cache, no-store, must-revalidate\r\n");
		transaction->Write("Pragma: no-cache\r\n");
		transaction->Write("Expires: 0\r\n");
		transaction->Write("Access-Control-Allow-Origin: *\r\n");
		transaction->Write("Access-Control-Allow-Headers: Content-Type\r\n");
		transaction->Write("Content-Length: 0\r\n");
		transaction->Write("\r\n");
		transaction->Commit(false);

		ResetState();
		return true;
	}

	if (IsAuthenticated() && StringEqualsIgnoreCase(commandWords[0], "POST"))
	{
		const bool isUploadRequest = (StringEqualsIgnoreCase(commandWords[1], KO_START "upload"))
								  || (commandWords[1][0] == '/' && StringEqualsIgnoreCase(commandWords[1] + 1, KO_START "upload"));
		if (isUploadRequest)
		{
			const char* const filename = GetKeyValue("name");
			if (filename != nullptr)
			{
				// We cannot upload more than one file at once
				if (IsUploading())
				{
					return RejectMessage("cannot upload more than one file at once");
				}

				// See how many bytes we expect to read
				bool contentLengthFound = false;
				for (size_t i = 0; i < numHeaderKeys; i++)
				{
					if (StringEqualsIgnoreCase(headers[i].key, "Content-Length"))
					{
						postFileLength = atoi(headers[i].value);
						contentLengthFound = true;
						break;
					}
				}

				// Start POST file upload
				if (!contentLengthFound)
				{
					return RejectMessage("invalid POST upload request");
				}

				// Start a new file upload
				FileStore *file = platform->OpenFile(FS_PREFIX, filename, OpenMode::write);
				if (!StartUpload(file, filename))
				{
					return RejectMessage("could not start file upload");
				}

				// Try to get the last modified file date and time
				const char* const lastModifiedString = GetKeyValue("time");
				if (lastModifiedString != nullptr)
				{
					struct tm timeInfo;
					memset(&timeInfo, 0, sizeof(timeInfo));
					if (strptime(lastModifiedString, "%Y-%m-%dT%H:%M:%S", &timeInfo) != nullptr)
					{
						fileLastModified  = mktime(&timeInfo);
					}
					else
					{
						fileLastModified = 0;
					}
				}
				else
				{
					fileLastModified = 0;
				}

				if (reprap.Debug(moduleWebserver))
				{
					platform->MessageF(UsbMessage, "Start uploading file %s length %lu\n", filename, postFileLength);
				}
				uploadedBytes = 0;

				// Keep track of the connection that is now uploading
				uint32_t remoteIP = webserver->currentTransaction->GetRemoteIP();
				uint16_t remotePort = webserver->currentTransaction->GetRemotePort();
				for(size_t i = 0; i < numSessions; i++)
				{
					if (sessions[i].ip == remoteIP)
					{
						sessions[i].postPort = remotePort;
						sessions[i].isPostUploading = true;
						break;
					}
				}

				ResetState();
				return true;
			}
		}
		return RejectMessage("only rr_upload is supported for POST requests");
	}
	else
	{
		return RejectMessage("Unknown message type or not authenticated");
	}
}

// Reject the current message. Always returns true to indicate that we should stop reading the message.
bool Webserver::HttpInterpreter::RejectMessage(const char* response, unsigned int code)
{
	platform->MessageF(UsbMessage, "Webserver: rejecting message with: %s\n", response);

	NetworkTransaction *transaction = webserver->currentTransaction;
	transaction->Printf("HTTP/1.1 %u %s\nConnection: close\r\n\r\n", code, response);
	transaction->Commit(false);

	ResetState();

	return true;
}

// Authenticate current IP and return true on success
bool Webserver::HttpInterpreter::Authenticate()
{
	if (numSessions < maxHttpSessions)
	{
		sessions[numSessions].ip = webserver->currentTransaction->GetRemoteIP();
		sessions[numSessions].lastQueryTime = millis();
		sessions[numSessions].isPostUploading = false;
		numSessions++;
		return true;
	}
	return false;
}

bool Webserver::HttpInterpreter::IsAuthenticated() const
{
	const uint32_t remoteIP = webserver->currentTransaction->GetRemoteIP();
	for(size_t i = 0; i < numSessions; i++)
	{
		if (sessions[i].ip == remoteIP)
		{
			return true;
		}
	}
	return false;
}

void Webserver::HttpInterpreter::UpdateAuthentication()
{
	const uint32_t remoteIP = webserver->currentTransaction->GetRemoteIP();
	for(size_t i = 0; i < numSessions; i++)
	{
		if (sessions[i].ip == remoteIP)
		{
			sessions[i].lastQueryTime = millis();
			break;
		}
	}
}

bool Webserver::HttpInterpreter::RemoveAuthentication()
{
	const uint32_t remoteIP = webserver->currentTransaction->GetRemoteIP();
	for(int i=(int)numSessions - 1; i>=0; i--)
	{
		if (sessions[i].ip == remoteIP)
		{
			if (sessions[i].isPostUploading)
			{
				// Don't allow sessions with active POST uploads to be removed
				return false;
			}

			for (size_t k = i + 1; k < numSessions; ++k)
			{
				memcpy(&sessions[k - 1], &sessions[k], sizeof(HttpSession));
			}
			numSessions--;
			return true;
		}
	}
	return false;
}

// Handle a G Code reply from the GCodes class
void Webserver::HttpInterpreter::HandleGCodeReply(OutputBuffer *reply)
{
	if (reply != nullptr)
	{
		if (numSessions > 0)
		{
			// FIXME: This might cause G-code responses to be sent twice to fast HTTP clients, but
			// I (chrishamm) cannot think of a nicer way to deal with slow clients at the moment...
			gcodeReply->Push(reply);
			clientsServed = 0;
			seq++;
		}
		else
		{
			// Don't use buffers that may never get released...
			OutputBuffer::ReleaseAll(reply);
		}
	}
}

void Webserver::HttpInterpreter::HandleGCodeReply(const char *reply)
{
	if (numSessions > 0)
	{
		OutputBuffer *buffer = gcodeReply->GetLastItem();
		if (buffer == nullptr || buffer->IsReferenced())
		{
			if (!OutputBuffer::Allocate(buffer))
			{
				// No more space available, stop here
				return;
			}
			gcodeReply->Push(buffer);
		}

		buffer->cat(reply);
		clientsServed = 0;
		seq++;
	}
}

// Called to process a deferred request and takes care of the current Webserver transaction
void Webserver::HttpInterpreter::ProcessDeferredRequest()
{
	OutputBuffer *jsonResponse = nullptr;
	const Connection lastDeferredConnection = deferredRequestConnection;

	// At the moment only file info requests are deferred.
	// Parsing the file may take a while, so keep LwIP running while we're waiting
	network->Unlock();
	bool gotFileInfo = reprap.GetFileInfoResponse(filenameBeingProcessed, jsonResponse, false);
	while (!network->Lock());

	// Because LwIP was unlocked before, there is a chance that the ConnectionLost() call has already
	// stopped the file parsing. Check this special case here
	if (lastDeferredConnection == deferredRequestConnection)
	{
		NetworkTransaction *transaction = webserver->currentTransaction;
		if (gotFileInfo)
		{
			deferredRequestConnection = NoConnection;

			// Got it - send the response now
			transaction->Write("HTTP/1.1 200 OK\r\n");
			transaction->Write("Cache-Control: no-cache, no-store, must-revalidate\r\n");
			transaction->Write("Pragma: no-cache\r\n");
			transaction->Write("Expires: 0\r\n");
			transaction->Write("Access-Control-Allow-Origin: *\r\n");
			transaction->Write("Content-Type: application/json\r\n");
			transaction->Printf("Content-Length: %u\r\n", (jsonResponse != nullptr) ? jsonResponse->Length() : 0);
			transaction->Printf("Connection: close\r\n\r\n");
			transaction->Write(jsonResponse);

			transaction->Commit(false);
		}
		else
		{
			// File hasn't been fully parsed yet, try again later
			transaction->Defer(DeferralMode::DiscardData);
		}
	}
	else
	{
		// Clean up again if we cannot send the response at all
		OutputBuffer::ReleaseAll(jsonResponse);
	}
}

//********************************************************************************************
//
//************************* FTP interpreter for the Webserver class **************************
//
//********************************************************************************************

Webserver::FtpInterpreter::FtpInterpreter(Platform *p, Webserver *ws, Network *n)
	: ProtocolInterpreter(p, ws, n), state(authenticating), clientPointer(0)
{
	connectedClients = 0;
	strcpy(currentDir, "/");
}

void Webserver::FtpInterpreter::Diagnostics(MessageType mt)
{
	platform->MessageF(mt, "FTP connections: %d, state %d\n", connectedClients, state);
}

void Webserver::FtpInterpreter::ConnectionEstablished()
{
	connectedClients++;
	if (reprap.Debug(moduleWebserver))
	{
		platform->Message(UsbMessage, "Webserver: FTP connection established!\n");
	}

	// Is this a new connection on the data port?
	NetworkTransaction *transaction = webserver->currentTransaction;
	if (transaction->GetLocalPort() != Network::GetFtpPort())
	{
		if (state == waitingForPasvPort)
		{
			// Yes - save it for the main request
			network->SaveDataConnection();
			state = pasvPortConnected;
			transaction->Discard();
		}
		else
		{
			// Should never get here...
			transaction->Commit(false);
		}
		return;
	}

	// A client is trying to connect to the main FTP port
	switch (state)
	{
		case idle:
		case authenticated:		// added by DC because without it, we can't transfer any files with FileZilla
			// We can safely deal with one connection on the main FTP port
			state = authenticating;
			SendReply(220, "RepRapFirmware FTP server", true);
			break;

		default:
			// But don't allow multiple ones, this could mess things up
			SendReply(421, "Only one client can be connected at a time.", false);
			return;
	}
}

// May be called from ISR!
void Webserver::FtpInterpreter::ConnectionLost(Connection conn)
{
	connectedClients--;

	if (Network::GetLocalPort(conn) != Network::GetFtpPort())
	{
		// Did everything work out? Usually this is only called for uploads
		if (network->AcquireFTPTransaction())
		{
			webserver->currentTransaction = network->GetTransaction();
			if (state == doingPasvIO)
			{
				if (uploadState != uploadError && !Network::IsTerminated(conn))
				{
					SendReply(226, "Transfer complete.");
					FinishUpload(0);
				}
				else
				{
					SendReply(526, "Transfer failed!");
				}
			}
			else
			{
				SendReply(550, "Lost data connection!");
			}
		}

		// Close the data port and reset our state again
		network->CloseDataPort();
		CancelUpload();
		state = authenticated;
	}

	if (connectedClients == 0)
	{
		// Last one gone now...
		ResetState();
	}
}

bool Webserver::FtpInterpreter::CharFromClient(char c)
{
	if (clientPointer == ARRAY_UPB(clientMessage))
	{
		clientPointer = 0;
		platform->Message(UsbMessage, "Webserver: Buffer overflow in FTP server!\n");
		return true;
	}

	switch (c)
	{
		case 0:
			break;

		case '\r':
		case '\n':
			clientMessage[clientPointer++] = 0;

			if (reprap.Debug(moduleWebserver))
			{
				platform->MessageF(UsbMessage, "FtpInterpreter::ProcessLine called with state %d:\n%s\n", state, clientMessage);
			}

			if (clientPointer > 1) // only process a new line if we actually received data
			{
				ProcessLine();
				clientPointer = 0;
				return true;
			}

			if (reprap.Debug(moduleWebserver))
			{
				platform->Message(UsbMessage, "FtpInterpreter::ProcessLine call finished.\n");
			}

			clientPointer = 0;
			break;

		default:
			clientMessage[clientPointer++] = c;
			break;
	}

	return false;
}

void Webserver::FtpInterpreter::ResetState()
{
	clientPointer = 0;
	strcpy(currentDir, "/");

	network->CloseDataPort();
	CancelUpload();

	state = idle;
}

bool Webserver::FtpInterpreter::DoingFastUpload() const
{
	return (IsUploading() && webserver->currentTransaction->GetLocalPort() == network->GetDataPort());
}

// return true if an error has occurred, false otherwise
void Webserver::FtpInterpreter::ProcessLine()
{
	switch (state)
	{
		case idle:
		case authenticating:
			// don't check the user name
			if (StringStartsWith(clientMessage, "USER"))
			{
				SendReply(331, "Please specify the password.");
			}
			// but check the password
			else if (StringStartsWith(clientMessage, "PASS"))
			{
				char pass[RepRapPasswordLength];
				int pass_length = 0;
				bool reading_pass = false;
				for(size_t i = 4; i < clientPointer && i < RepRapPasswordLength + 3; i++)
				{
					reading_pass |= (clientMessage[i] != ' ' && clientMessage[i] != '\t');
					if (reading_pass)
					{
						pass[pass_length++] = clientMessage[i];
					}
				}
				pass[pass_length] = 0;

				if (reprap.CheckPassword(pass))
				{
					state = authenticated;
					SendReply(230, "Login successful.");
				}
				else
				{
					SendReply(530, "Login incorrect.", false);
				}
			}
			// if it's different, send response 500 to indicate we don't know the code (might be AUTH or so)
			else
			{
				SendReply(500, "Unknown login command.");
			}

			break;

		case authenticated:
			// get system type
			if (StringEqualsIgnoreCase(clientMessage, "SYST"))
			{
				SendReply(215, "UNIX Type: L8");
			}
			// get features
			else if (StringEqualsIgnoreCase(clientMessage, "FEAT"))
			{
				SendFeatures();
			}
			// get current dir
			else if (StringEqualsIgnoreCase(clientMessage, "PWD"))
			{
				NetworkTransaction *transaction = webserver->currentTransaction;
				transaction->Printf("257 \"%s\"\r\n", currentDir);
				transaction->Commit(true);
			}
			// set current dir
			else if (StringStartsWith(clientMessage, "CWD"))
			{
				ReadFilename(3);
				ChangeDirectory(filename.c_str());
			}
			// change to parent of current directory
			else if (StringEqualsIgnoreCase(clientMessage, "CDUP"))
			{
				ChangeDirectory("..");
			}
			// switch transfer mode (sends response, but doesn't have any effects)
			else if (StringStartsWith(clientMessage, "TYPE"))
			{
				for(size_t i = 4; i < clientPointer; i++)
				{
					if (clientMessage[i] == 'I')
					{
						SendReply(200, "Switching to Binary mode.");
						return;
					}

					if (clientMessage[i] == 'A')
					{
						SendReply(200, "Switching to ASCII mode.");
						return;
					}
				}

				SendReply(500, "Unknown command.");
			}
			// enter passive mode mode
			else if (StringEqualsIgnoreCase(clientMessage, "PASV"))
			{
				/* get local IP address */
				const uint8_t * const ip_address = network->GetIPAddress();

				/* open random port > 1023 */
				uint16_t pasv_port = random(1024, 65535);
				network->OpenDataPort(pasv_port);
				portOpenTime = millis();
				state = waitingForPasvPort;

				/* send FTP response */
				NetworkTransaction *transaction = webserver->currentTransaction;
				transaction->Printf("227 Entering Passive Mode (%d,%d,%d,%d,%d,%d)\r\n",
						ip_address[0], ip_address[1], ip_address[2], ip_address[3],
						pasv_port / 256, pasv_port % 256);
				transaction->Commit(true);
			}
			// PASV commands are not supported in this state
			else if (StringEqualsIgnoreCase(clientMessage, "LIST") || StringStartsWith(clientMessage, "RETR") || StringStartsWith(clientMessage, "STOR"))
			{
				SendReply(425, "Use PASV first.");
			}
			// delete file
			else if (StringStartsWith(clientMessage, "DELE"))
			{
				ReadFilename(4);
				if (platform->Delete(currentDir, filename.c_str()))
				{
					SendReply(250, "Delete operation successful.");
				}
				else
				{
					SendReply(550, "Delete operation failed.");
				}
			}
			// delete directory
			else if (StringStartsWith(clientMessage, "RMD"))
			{
				ReadFilename(3);
				if (platform->Delete(currentDir, filename.c_str()))
				{
					SendReply(250, "Remove directory operation successful.");
				}
				else
				{
					SendReply(550, "Remove directory operation failed.");
				}
			}
			// make new directory
			else if (StringStartsWith(clientMessage, "MKD"))
			{
				ReadFilename(3);
				String<MaxFilenameLength> location;
				if (filename[0] == '/')
				{
					location.copy(filename.c_str());
				}
				else
				{
					MassStorage::CombineName(location.GetRef(), currentDir, filename.c_str());
				}

				if (platform->GetMassStorage()->MakeDirectory(location.c_str()))
				{
					NetworkTransaction *transaction = webserver->currentTransaction;
					transaction->Printf("257 \"%s\" created\r\n", location);
					transaction->Commit(true);
				}
				else
				{
					SendReply(550, "Create directory operation failed.");
				}
			}
			// rename file or directory
			else if (StringStartsWith(clientMessage, "RNFR"))
			{
				ReadFilename(4);
				if (filename[0] != '/')
				{
					String<MaxFilenameLength> temp;
					MassStorage::CombineName(temp.GetRef(), currentDir, filename.c_str());
					filename.copy(temp.c_str());
				}

				if (platform->GetMassStorage()->FileExists(filename.c_str()))
				{
					SendReply(350, "Ready to RNTO.");
				}
				else
				{
					SendReply(550, "Invalid file or directory.");
				}
			}
			else if (StringStartsWith(clientMessage, "RNTO"))
			{
				// Copy origin path to temp oldFilename and read new path
				String<MaxFilenameLength> oldFilename;
				oldFilename.copy(filename.c_str());
				ReadFilename(4);

				String<MaxFilenameLength> newFilename;
				MassStorage::CombineName(newFilename.GetRef(), currentDir, filename.c_str());
				if (platform->GetMassStorage()->Rename(oldFilename.c_str(), newFilename.c_str()))
				{
					SendReply(250, "Rename successful.");
				}
				else
				{
					SendReply(500, "Could not rename file or directory.");
				}
			}
			// no op
			else if (StringEqualsIgnoreCase(clientMessage, "NOOP"))
			{
				SendReply(200, "NOOP okay.");
			}
			// end connection
			else if (StringEqualsIgnoreCase(clientMessage, "QUIT"))
			{
				SendReply(221, "Goodbye.", false);
				ResetState();
			}
			// unknown
			else
			{
				SendReply(500, "Unknown command.");
			}
			break;

		case waitingForPasvPort:
			if (millis() - portOpenTime > ftpPasvPortTimeout)
			{
				SendReply(425, "Failed to establish connection.");

				network->CloseDataPort();
				state = authenticated;
			}
			else
			{
				webserver->currentTransaction->Defer(DeferralMode::ResetData);
			}
			break;

		case pasvPortConnected:
			// save current connection state so we can send '226 Transfer complete.' when ConnectionLost() is called
			network->SaveFTPConnection();

			// list directory entries
			if (StringStartsWith(clientMessage, "LIST"))
			{
				if (network->AcquireDataTransaction())
				{
					// send announcement via ftp main port
					SendReply(150, "Here comes the directory listing.");

					// send directory listing via data port
					NetworkTransaction *dataTransaction = network->GetTransaction();

					FileInfo fileInfo;
					if (platform->GetMassStorage()->FindFirst(currentDir, fileInfo))
					{
						do {
							// Example for a typical UNIX-like file list:
							// "drwxr-xr-x    2 ftp      ftp             0 Apr 11 2013 bin\r\n"
							const char dirChar = (fileInfo.isDirectory) ? 'd' : '-';
							const struct tm * const timeInfo = gmtime(&fileInfo.lastModified);
							dataTransaction->Printf("%crw-rw-rw- 1 ftp ftp %13lu %s %02d %04d %s\r\n",
									dirChar, fileInfo.size, platform->GetMassStorage()->GetMonthName(timeInfo->tm_mon + 1),
									timeInfo->tm_mday, timeInfo->tm_year + 1900, fileInfo.fileName);
						} while (platform->GetMassStorage()->FindNext(fileInfo));
					}

					dataTransaction->Commit(false);
					state = doingPasvIO;
				}
				else
				{
					SendReply(500, "Unknown error.");
					network->CloseDataPort();
					state = authenticated;
				}
			}
			// switch transfer mode (sends response, but doesn't have any effects)
			else if (StringStartsWith(clientMessage, "TYPE"))
			{
				for (size_t i = 4; i < clientPointer; i++)
				{
					if (clientMessage[i] == 'I')
					{
						SendReply(200, "Switching to Binary mode.");
						break;
					}

					if (clientMessage[i] == 'A')
					{
						SendReply(200, "Switching to ASCII mode.");
						break;
					}
				}

				SendReply(500, "Unknown command.");
			}
			// upload a file
			else if (StringStartsWith(clientMessage, "STOR"))
			{
				ReadFilename(4);

				FileStore *file = platform->OpenFile(currentDir, filename.c_str(), OpenMode::write);
				if (StartUpload(file, filename.c_str()))
				{
					SendReply(150, "OK to send data.");
					state = doingPasvIO;
				}
				else
				{
					SendReply(550, "Failed to open file.");
					network->CloseDataPort();
					state = authenticated;
				}
			}
			// download a file
			else if (StringStartsWith(clientMessage, "RETR"))
			{
				ReadFilename(4);

				FileStore *file = platform->OpenFile(currentDir, filename.c_str(), OpenMode::read);
				if (file == nullptr)
				{
					SendReply(550, "Failed to open file.");
				}
				else
				{
					if (network->AcquireDataTransaction())
					{
						// send announcement via main ftp port
						NetworkTransaction *transaction = webserver->currentTransaction;
						transaction->Printf("150 Opening data connection for %s (%lu bytes).\r\n", filename, file->Length());
						transaction->Commit(true);

						// send the file via data port
						NetworkTransaction *dataTransaction = network->GetTransaction();
						dataTransaction->SetFileToWrite(file);
						dataTransaction->Commit(false);
						state = doingPasvIO;
					}
					else
					{
						file->Close();
						SendReply(500, "Unknown error.");
						network->CloseDataPort();
						state = authenticated;
					}
				}
			}
			// unknown command
			else
			{
				SendReply(500, "Unknown command.");
				network->CloseDataPort();
				state = authenticated;
			}
			break;

		case doingPasvIO:
			// abort current transfer
			if (StringEqualsIgnoreCase(clientMessage, "ABOR"))
			{
				if (IsUploading())
				{
					CancelUpload();
					SendReply(226, "ABOR successful.");
				}
				else
				{
					network->CloseDataPort();
					SendReply(226, "ABOR successful.");
				}
			}
			// unknown command
			else
			{
				SendReply(500, "Unknown command.");
				network->CloseDataPort();
				state = authenticated;
			}
			break;
	}
}

void Webserver::FtpInterpreter::SendReply(int code, const char *message, bool keepConnection)
{
	NetworkTransaction *transaction = webserver->currentTransaction;
	transaction->Printf("%d %s\r\n", code, message);
	transaction->Commit(keepConnection);
}

void Webserver::FtpInterpreter::SendFeatures()
{
	NetworkTransaction *transaction = webserver->currentTransaction;
	transaction->Write("211-Features:\r\n");
	transaction->Write("PASV\r\n");		// support PASV mode
	transaction->Write("211 End\r\n");
	transaction->Commit(true);
}

void Webserver::FtpInterpreter::ReadFilename(uint16_t start)
{
	int filenameLength = 0;
	bool readingPath = false;
	for(int i = start; i < (int)clientPointer && filenameLength < (int)(MaxFilenameLength - 1); i++)
	{
		switch (clientMessage[i])
		{
			// ignore quotes
			case '"':
			case '\'':
				break;

			// skip whitespaces unless the actual filename is being read
			case ' ':
			case '\t':
				if (readingPath)
				{
					filename[filenameLength++] = clientMessage[i];
				}
				break;

			// read path name
			default:
				readingPath = true;
				filename[filenameLength++] = clientMessage[i];
				break;
		}
	}
	filename[filenameLength] = 0;
}

void Webserver::FtpInterpreter::ChangeDirectory(const char *newDirectory)
{
	char combinedPath[MaxFilenameLength];

	if (newDirectory[0] != 0)
	{
		/* Prepare the new directory path */
		if (newDirectory[0] == '/') // absolute path
		{
			SafeStrncpy(combinedPath, newDirectory, MaxFilenameLength);
			combinedPath[MaxFilenameLength - 1] = 0;
		}
		else if (StringEqualsIgnoreCase(newDirectory, "."))
		{
			SafeStrncpy(combinedPath, currentDir, ARRAY_SIZE(combinedPath));
		}
		else if (StringEqualsIgnoreCase(newDirectory, "..")) // go up
		{
			if (StringEqualsIgnoreCase(currentDir, "/"))
			{
				// we're already at the root, so we can't go up any more
				SendReply(550, "Failed to change directory.");
				return;
			}
			else
			{
				SafeStrncpy(combinedPath, currentDir, MaxFilenameLength);
				for(int i=strlen(combinedPath) -2; i>=0; i--)
				{
					if (combinedPath[i] == '/')
					{
						combinedPath[i +1] = 0;
						break;
					}
				}
			}
		}
		else // go to child directory
		{
			SafeStrncpy(combinedPath, currentDir, MaxFilenameLength);
			if (strlen(currentDir) > 1)
			{
				SafeStrncat(combinedPath, "/", MaxFilenameLength);
			}
			SafeStrncat(combinedPath, newDirectory, MaxFilenameLength);
		}

		/* Make sure the new path does not end with a '/', because FatFs won't see the directory otherwise */
		if (StringEndsWithIgnoreCase(combinedPath, "/") && strlen(combinedPath) > 1)
		{
			combinedPath[strlen(combinedPath) -1] = 0;
		}

		/* Verify path and change it */
		if (platform->GetMassStorage()->DirectoryExists(combinedPath))
		{
			SafeStrncpy(currentDir, combinedPath, MaxFilenameLength);
			SendReply(250, "Directory successfully changed.");
		}
		else
		{
			SendReply(550, "Failed to change directory.");
		}
	}
	else
	{
		SendReply(550, "Failed to change directory.");
	}
}


//********************************************************************************************
//
//*********************** Telnet interpreter for the Webserver class *************************
//
//********************************************************************************************

Webserver::TelnetInterpreter::TelnetInterpreter(Platform *p, Webserver *ws, Network *n)
	: ProtocolInterpreter(p, ws, n), connectedClients(0), processNextLine(false), gcodeReply(nullptr)
{
	ResetState();
}

void Webserver::TelnetInterpreter::Diagnostics(MessageType mt)
{
	platform->MessageF(mt, "Telnet connections: %d, state %d\n", connectedClients, state);
}

void Webserver::TelnetInterpreter::ConnectionEstablished()
{
	connectedClients++;
	NetworkTransaction *transaction = network->GetTransaction();

	// Only one client may be connected via Telnet at once, so check this first
	if (state != idle)
	{
		transaction->Write("Sorry, only one client may be connected via Telnet at once.\r\n");
		transaction->Commit(false);
		return;
	}
	state = justConnected;
	connectTime = millis();

	// Check whether we need a password to log in
	if (reprap.NoPasswordSet())
	{
		// Don't send a login prompt if no password is set, so we don't mess up Pronterface
		transaction->Discard();
	}
	else
	{
		transaction->Write("RepRapFirmware Telnet interface\r\n\r\n");
		transaction->Write("Please enter your password:\r\n");
		transaction->Write("> ");
		transaction->Commit(true);
	}
}

// May be called from ISR!
void Webserver::TelnetInterpreter::ConnectionLost(Connection conn)
{
	connectedClients--;
	if (connectedClients == 0)
	{
		ResetState();

		// Don't save up output buffers if they can't be sent
		OutputBuffer::ReleaseAll(gcodeReply);
		gcodeReply = nullptr;
	}
}

bool Webserver::TelnetInterpreter::CanParseData()
{
	// Is this an acquired transaction using which we can send the G-code reply?
	TransactionStatus status = webserver->currentTransaction->GetStatus();
	if (status == TransactionStatus::acquired)
	{
		SendGCodeReply();
		return false;
	}

	// Is this connection still live? Check that for deferred requests
	if (status == TransactionStatus::deferred && !webserver->currentTransaction->IsConnected())
	{
		webserver->currentTransaction->Discard();
		return false;
	}

	// In order to support TCP streaming mode, check if we can store any more data at this time
	RegularGCodeInput * const telnetInput = reprap.GetGCodes().GetTelnetInput();
	if (telnetInput->BufferSpaceLeft() < clientPointer + 1)
	{
		webserver->currentTransaction->Defer(DeferralMode::DeferOnly);
		return false;
	}

	// If that works and if the next line hasn't been processed yet, do it now
	if (processNextLine)
	{
		return !ProcessLine();
	}

	// Otherwise just parse the next request
	return true;
}

bool Webserver::TelnetInterpreter::CharFromClient(char c)
{
	// If this is likely to be a Telnet setup message (with some garbage in it), dump the first
	// received packet and move on to the next state
	if (state == justConnected)
	{
		if (reprap.NoPasswordSet())
		{
			state = authenticated;
			network->SaveTelnetConnection();
		}
		else
		{
			state = authenticating;
		}

		if (millis() - connectTime < telnetSetupDuration)
		{
			network->GetTransaction()->Discard();
			return true;
		}
	}

	// Otherwise try to read one line at a time
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
				// This line is complete, do we have enough space left to store it?
				clientMessage[clientPointer] = 0;
				RegularGCodeInput * const telnetInput = reprap.GetGCodes().GetTelnetInput();
				if (telnetInput->BufferSpaceLeft() < clientPointer + 1)
				{
					// No - defer this transaction, so we can process more of it next time
					webserver->currentTransaction->Defer(DeferralMode::DeferOnly);
					processNextLine = true;
					return true;
				}

				// Yes - try to process it
				return ProcessLine();
			}
			break;

		default:
			clientMessage[clientPointer++] = c;

			// Make sure we don't overflow the line buffer
			if (clientPointer == ARRAY_UPB(clientMessage))
			{
				clientPointer = 0;
				platform->Message(UsbMessage, "Webserver: Buffer overflow in Telnet server!\n");
				return true;
			}
			break;
	}

	return false;
}

void Webserver::TelnetInterpreter::ResetState()
{
	state = idle;
	connectTime = 0;
	clientPointer = 0;
}

// Usually we should not try to send any data here, because that would purge the packet's
// payload and mess with TCP streaming mode if Pronterface is used. However, under special
// circumstances this must happen and in this case this method must always return true.
bool Webserver::TelnetInterpreter::ProcessLine()
{
	processNextLine = false;
	clientPointer = 0;

	NetworkTransaction *transaction = network->GetTransaction();
	switch (state)
	{
		case idle:
		case justConnected:
			// Should never get here...
			// no break

		case authenticating:
			if (reprap.CheckPassword(clientMessage))
			{
				network->SaveTelnetConnection();
				state = authenticated;

				transaction->Write("Log in successful!\r\n");
				transaction->Commit(true);
			}
			else
			{
				transaction->Write("Invalid password.\r\n> ");
				transaction->Commit(true);
			}
			return true;

		case authenticated:
			// Special commands for Telnet
			if (StringEqualsIgnoreCase(clientMessage, "exit") || StringEqualsIgnoreCase(clientMessage, "quit"))
			{
				transaction->Write("Goodbye.\r\n");
				transaction->Commit(false);
				return true;
			}

			// All other codes are stored for the GCodes class
			NetworkGCodeInput * const telnetInput = reprap.GetGCodes().GetTelnetInput();
			telnetInput->Put(TelnetMessage, clientMessage);
			break;
	}
	return false;
}

// Handle a G-Code reply from the GCodes class; replace \n with \r\n
void Webserver::TelnetInterpreter::HandleGCodeReply(OutputBuffer *reply)
{
	if (reply != nullptr && state >= authenticated)
	{
		if (!network->AcquireTelnetTransaction())
		{
			// We must be able to send the response to the client on the next Spin call
			return;
		}

		// We need a valid OutputBuffer to start the conversion from NL to CRNL
		if (gcodeReply == nullptr)
		{
			OutputBuffer *buffer;
			if (!OutputBuffer::Allocate(buffer))
			{
				OutputBuffer::Truncate(reply, OUTPUT_BUFFER_SIZE);
				if (!OutputBuffer::Allocate(buffer))
				{
					// If we're really short on memory, release the G-Code reply instantly
					OutputBuffer::ReleaseAll(reply);
					return;
				}
			}
			gcodeReply = buffer;
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
	}
	else
	{
		// Don't store buffers that may never get released...
		OutputBuffer::ReleaseAll(reply);
	}
}

void Webserver::TelnetInterpreter::HandleGCodeReply(const char *reply)
{
	if (reply != nullptr && state >= authenticated)
	{
		if (!network->AcquireTelnetTransaction())
		{
			// We must be able to send the response to the client on the next Spin call
			return;
		}

		// We need a valid OutputBuffer to start the conversion from NL to CRNL
		if (gcodeReply == nullptr)
		{
			OutputBuffer *buffer;
			if (!OutputBuffer::Allocate(buffer))
			{
				// No more space available to store this reply, stop here
				return;
			}
			gcodeReply = buffer;
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
		};
	}
}

void Webserver::TelnetInterpreter::SendGCodeReply()
{
	NetworkTransaction *transaction = webserver->currentTransaction;

	if (gcodeReply == nullptr)
	{
		transaction->Discard();
	}
	else
	{
		transaction->Write(gcodeReply);
		transaction->Commit(true);
	}

	gcodeReply = nullptr;
}

// End
