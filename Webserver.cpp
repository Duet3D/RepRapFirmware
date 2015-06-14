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
 must not have names starting "/rr_" or they will not be found.

 rr_connect?password=xxx
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

 rr_files?dir=xxx
 	 	 	 Returns a listing of the filenames in the /gcode directory of the SD card. 'dir' is a
 	 	 	 directory path relative to the root of the SD card. If the 'dir' variable is not present,
 	 	 	 it defaults to the /gcode directory.

 rr_reply    Returns the last-known G-code reply as plain text (not encapsulated as JSON).

 rr_upload?name=xxx
 	 	 	 Upload a specified file using a POST request. The payload of this request has to be
 	 	 	 the file content. Only one file may be uploaded at once. When the upload has finished,
 	 	 	 a JSON response with the variable "err" will be returned, which will be 0 if the job
 	 	 	 has finished without problems, it will be set to 1 otherwise.

 rr_upload_begin?name=xxx
 	 	 	 Indicates that we wish to upload the specified file. xxx is the filename relative
 	 	 	 to the root of the SD card. The directory component of the filename must already
 	 	 	 exist. Returns variables ubuff (= max upload data we can accept in the next message)
 	 	 	 and err (= 0 if the file was created successfully, nonzero if there was an error).

rr_upload_data?data=xxx
 	 	 	 Provides a data block for the file upload. Returns the samwe variables as rr_upload_begin,
 	 	 	 except that err is only zero if the file was successfully created and there has not been
 	 	 	 a file write error yet. This response is returned before attempting to write this data block.

rr_upload_end
 	 	 	 Indicates that we have finished sending upload data. The server closes the file and reports
 	 	 	 the overall status in err. It may also return ubuff again.

rr_upload_cancel
 	 	 	 Indicates that the user wishes to cancel the current upload. Returns err and ubuff.

rr_delete?name=xxx
			 Delete file xxx. Returns err (zero if successful).

rr_mkdir?dir=xxx
			 Create a new directory xxx. Return err (zero if successful).

rr_move?old=xxx&new=yyy
			 Rename an old file xxx to yyy. May also be used to move a file to another directory.

 ****************************************************************************************************/

#include "RepRapFirmware.h"

//***************************************************************************************************

static const char* overflowResponse = "overflow";
static const char* badEscapeResponse = "bad escape";

const float pasvPortTimeout = 10.0;	 					// seconds to wait for the FTP data port


//********************************************************************************************
//
//**************************** Generic Webserver implementation ******************************
//
//********************************************************************************************



// Constructor and initialisation
Webserver::Webserver(Platform* p, Network *n) : platform(p), network(n),
		webserverActive(false), readingConnection(NULL)
{
	httpInterpreter = new HttpInterpreter(p, this, n);
	ftpInterpreter = new FtpInterpreter(p, this, n);
	telnetInterpreter = new TelnetInterpreter(p, this, n);
}

void Webserver::Init()
{
	// initialise the webserver class
	gcodeReadIndex = gcodeWriteIndex = 0;
	lastTime = platform->Time();
	longWait = lastTime;
	webserverActive = true;

	// initialise all protocol handlers
	httpInterpreter->ResetState();
	httpInterpreter->ResetSessions();
	ftpInterpreter->ResetState();
	telnetInterpreter->ResetState();
}


// Deal with input/output from/to the client (if any)
void Webserver::Spin()
{
	// Before we process an incoming Request, we must ensure that the webserver
	// is active and that all upload buffers are empty.

	if (webserverActive && httpInterpreter->FlushUploadData() && ftpInterpreter->FlushUploadData())
	{
		// Check if we can purge any HTTP sessions

		httpInterpreter->CheckSessions();

		// We must ensure that we have exclusive access to LWIP

		if (!network->Lock())
		{
			platform->ClassReport(longWait);
			return;
		}

		// See if we have new data to process

		NetworkTransaction *req = network->GetTransaction(readingConnection);
		if (req != NULL)
		{
			// Process incoming request
			if (!req->LostConnection())
			{
				// Take care of different protocol types here
				ProtocolInterpreter *interpreter;
				uint16_t localPort = req->GetLocalPort();
				switch (localPort)
				{
					case ftpPort: 		/* FTP */
						interpreter = ftpInterpreter;
						break;

					case telnetPort: 	/* Telnet */
						interpreter = telnetInterpreter;
						break;

					default:			/* HTTP and FTP data */
						if (localPort == network->GetHttpPort())
						{
							interpreter = httpInterpreter;
						}
						else
						{
							interpreter = ftpInterpreter;
						}
						break;
				}

				// For protocols other than HTTP it is important to send a HELO message
				TransactionStatus status = req->GetStatus();
				if (status == connected)
				{
					interpreter->ConnectionEstablished();

					// Close this request unless ConnectionEstablished() has already used it for sending
					if (req == network->GetTransaction())
					{
						network->CloseTransaction();
					}
				}
				// Graceful disconnects are handled here, because prior NetworkTransactions might still contain valid
				// data. That's why it's a bad idea to close these connections immediately in the Network class.
				else if (status == disconnected)
				{
					// CloseRequest() will call the disconnect events and close the connection
					network->CloseTransaction();
				}
				// Check for fast uploads
				else if (interpreter->DoingFastUpload())
				{
					if (!interpreter->DoFastUpload())
					{
						// Ensure this connection won't block everything if anything goes wrong
						readingConnection = NULL;
					}
				}
				// Check if we need to send data to a Telnet client
				else if (interpreter == telnetInterpreter && telnetInterpreter->HasRemainingData())
				{
					network->SendAndClose(NULL, true);
					telnetInterpreter->RemainingDataSent();
				}
				// Process other messages
				else
				{
					char c;
					for (uint16_t i=0; i<500; i++)
					{
						if (req->Read(c))
						{
							// Each ProtocolInterpreter must take care of the current NetworkTransaction and remove
							// it from the ready transactions by either calling SendAndClose() or CloseRequest().
							if (interpreter->CharFromClient(c))
							{
								readingConnection = NULL;
								break;
							}
						}
						else
						{
							// We ran out of data before finding a complete request.
							// This happens when the incoming message length exceeds the TCP MSS.
							// Check if we need to process another packet on the same connection.
							readingConnection = (interpreter->NeedMoreData()) ? req->GetConnection() : NULL;
							network->CloseTransaction();
							break;
						}
					}
				}
			}
			else
			{
				platform->Message(HOST_MESSAGE, "Webserver: Skipping zombie request with status %d\n", req->GetStatus());
				network->CloseTransaction();
			}
		}

		network->Unlock();
		platform->ClassReport(longWait);
	}
}

void Webserver::Exit()
{
	httpInterpreter->CancelUpload();
	ftpInterpreter->CancelUpload();

	platform->Message(BOTH_MESSAGE, "Webserver class exited.\n");
	webserverActive = false;
}

void Webserver::Diagnostics()
{
	platform->AppendMessage(BOTH_MESSAGE, "Webserver Diagnostics:\n");
}

// Process a null-terminated gcode
// We intercept four G/M Codes so we can deal with file manipulation and emergencies.  That
// way things don't get out of sync, and - as a file name can contain
// a valid G code (!) - confusion is avoided.
void Webserver::ProcessGcode(const char* gc)
{
	if (StringStartsWith(gc, "M112") && !isdigit(gc[4]))	// emergency stop
	{
		reprap.EmergencyStop();
		gcodeReadIndex = gcodeWriteIndex;		// clear the buffer
		reprap.GetGCodes()->Reset();
	}
	else if (StringStartsWith(gc, "M503") && !isdigit(gc[4]))	// echo config.g file
	{
		FileStore *configFile = platform->GetFileStore(platform->GetSysDir(), platform->GetConfigFile(), false);
		if (configFile == NULL)
		{
			ResponseToWebInterface("Configuration file not found", true);
		}
		else
		{
			reprap.MessageToGCodeReply("");

			char c;
			bool readingWhitespace = false;
			while (configFile->Read(c))
			{
				if (!readingWhitespace || (c != ' ' && c != '\t'))
				{
					reprap.AppendCharToStatusResponse(c);
				}
				readingWhitespace = (c == ' ' || c == '\t');
			}
			configFile->Close();

			telnetInterpreter->HandleGcodeReply(reprap.GetGcodeReply().Pointer());
		}
	}
	else
	{
		StoreGcodeData(gc, strlen(gc) + 1);
	}
}

// Feeding G Codes to the GCodes class
bool Webserver::GCodeAvailable()
{
	return gcodeReadIndex != gcodeWriteIndex;
}

char Webserver::ReadGCode()
{
	char c;
	if (gcodeReadIndex == gcodeWriteIndex)
	{
		c = 0;
	}
	else
	{
		c = gcodeBuffer[gcodeReadIndex];
		gcodeReadIndex = (gcodeReadIndex + 1u) % gcodeBufferLength;
	}
	return c;
}

// Process a received string of gcodes
void Webserver::LoadGcodeBuffer(const char* gc)
{
	char gcodeTempBuf[GcodeLength];
	uint16_t gtp = 0;
	bool inComment = false;
	for (;;)
	{
		char c = *gc++;
		if (c == 0)
		{
			gcodeTempBuf[gtp] = 0;
			ProcessGcode(gcodeTempBuf);
			return;
		}

		if (c == '\n')
		{
			gcodeTempBuf[gtp] = 0;
			ProcessGcode(gcodeTempBuf);
			gtp = 0;
			inComment = false;
		}
		else
		{
			if (c == ';')
			{
				inComment = true;
			}

			if (gtp == ARRAY_UPB(gcodeTempBuf))
			{
				// gcode is too long, we haven't room for another character and a null
				if (c != ' ' && !inComment)
				{
					platform->Message(BOTH_ERROR_MESSAGE, "Webserver: GCode local buffer overflow.\n");
					return;
				}
				// else we're either in a comment or the current character is a space.
				// If we're in a comment, we'll silently truncate it.
				// If the current character is a space, we'll wait until we see a non-comment character before reporting an error,
				// in case the next character is end-of-line or the start of a comment.
			}
			else
			{
				gcodeTempBuf[gtp++] = c;
			}
		}
	}
}

// Process a received string of gcodes
void Webserver::StoreGcodeData(const char* data, size_t len)
{
	if (len > GetGcodeBufferSpace())
	{
		platform->Message(BOTH_ERROR_MESSAGE, "GCode buffer overflow in Webserver!\n");
	}
	else
	{
		size_t remaining = gcodeBufferLength - gcodeWriteIndex;
		if (len <= remaining)
		{
			memcpy(gcodeBuffer + gcodeWriteIndex, data, len);
		}
		else
		{
			memcpy(gcodeBuffer + gcodeWriteIndex, data, remaining);
			memcpy(gcodeBuffer, data + remaining, len - remaining);
		}
		gcodeWriteIndex = (gcodeWriteIndex + len) % gcodeBufferLength;
	}
}

// Handle immediate disconnects here (cs will be freed after this call)
void Webserver::ConnectionLost(const ConnectionState *cs)
{
	// See which connection caused this event
	uint32_t remoteIP = cs->GetRemoteIP();
	uint16_t remotePort = cs->GetRemotePort();
	uint16_t localPort = cs->GetLocalPort();

	// Inform protocol handlers that this connection has been lost
	ProtocolInterpreter *interpreter;
	switch (localPort)
	{
		case ftpPort: 	/* FTP */
			interpreter = ftpInterpreter;
			break;

		case telnetPort: /* Telnet */
			interpreter = telnetInterpreter;
			break;

		default: 		/* HTTP and FTP data */
			if (localPort == network->GetHttpPort())
			{
				interpreter = httpInterpreter;
				break;
			}
			else if (localPort == network->GetDataPort())
			{
				interpreter = ftpInterpreter;
				break;
			}

			platform->Message(BOTH_ERROR_MESSAGE, "Webserver: Connection closed at local port %d, but no handler found!\n", localPort);
			return;
	}
	if (reprap.Debug(moduleWebserver))
	{
		platform->Message(HOST_MESSAGE, "Webserver: ConnectionLost called with port %d\n", localPort);
	}
	interpreter->ConnectionLost(remoteIP, remotePort, localPort);

	// If our reading connection is lost, it will be no longer important which connection is read from first.
	if (cs == readingConnection)
	{
		readingConnection = NULL;
	}
}

void Webserver::ResponseToWebInterface(const char *s, bool error)
{
	if (!webserverActive)
	{
		return;
	}

	if (strlen(s) == 0 && !error)
	{
		reprap.MessageToGCodeReply(s);
		telnetInterpreter->HandleGcodeReply("ok\r\n");
	}
	else
	{
		if (error)
		{
			reprap.MessageToGCodeReply("Error: ");
			reprap.AppendMessageToGCodeReply(s);

			telnetInterpreter->HandleGcodeReply("Error: ");
			telnetInterpreter->HandleGcodeReply(s);
		}
		else
		{
			reprap.MessageToGCodeReply(s);
			telnetInterpreter->HandleGcodeReply(s);
		}
	}
}

void Webserver::AppendResponseToWebInterface(const char *s)
{
	if (!webserverActive)
	{
		return;
	}

	reprap.AppendMessageToGCodeReply(s);
	telnetInterpreter->HandleGcodeReply(s);
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
	uploadPointer = NULL;
	uploadLength = 0;
	filenameBeingUploaded[0] = 0;
}

// Start writing to a new file
bool ProtocolInterpreter::StartUpload(FileStore *file)
{
	CancelUpload();

	if (file != NULL)
	{
		fileBeingUploaded.Set(file);
		uploadState = uploadOK;
		return true;
	}

	uploadState = uploadError;
	platform->Message(HOST_MESSAGE, "Could not open file while starting upload!\n");
	return false;
}

// Process a received buffer of upload data
bool ProtocolInterpreter::StoreUploadData(const char* data, unsigned int len)
{
	if (uploadState == uploadOK)
	{
		uploadPointer = data;
		uploadLength = len;
		return true;
	}
	return false;
}

// Try to flush upload buffer and return true if all data has been flushed
bool ProtocolInterpreter::FlushUploadData()
{
	if (uploadState == uploadOK && uploadLength != 0)
	{
		// Write some uploaded data to file (never write more than 256 bytes at once)
		unsigned int len = min<unsigned int>(uploadLength, 256);
		if (!fileBeingUploaded.Write(uploadPointer, len))
		{
			platform->Message(HOST_MESSAGE, "Could not flush upload data!\n");
			uploadState = uploadError;
		}

		uploadPointer += len;
		uploadLength -= len;

		return (uploadLength == 0);
	}

	return true;
}

void ProtocolInterpreter::CancelUpload()
{
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();		// cancel any pending file upload
		if (strlen(filenameBeingUploaded) != 0)
		{
			platform->GetMassStorage()->Delete("0:/", filenameBeingUploaded);
		}
	}
	filenameBeingUploaded[0] = 0;
	uploadPointer = NULL;
	uploadLength = 0;
	uploadState = notUploading;
}

bool ProtocolInterpreter::DoFastUpload()
{
	NetworkTransaction *req = network->GetTransaction();
	if (IsUploading())
	{
		char *buffer;
		unsigned int len;
		if (req->ReadBuffer(buffer, len))
		{
			StoreUploadData(buffer, len);
		}
		else
		{
			network->CloseTransaction();
		}
	}
	else if (req->DataLength() > 0)
	{
		platform->Message(HOST_MESSAGE, "Webserver: Closing invalid data connection\n");
		network->SendAndClose(NULL);
		return false;
	}

	return true;
}

void ProtocolInterpreter::FinishUpload(uint32_t fileLength)
{
	// Write the remaining data
	if (uploadState == uploadOK)
	{
		while (uploadLength > 0)
		{
			unsigned int len = min<unsigned int>(uploadLength, 256);
			if (!fileBeingUploaded.Write(uploadPointer, len))
			{
				uploadState = uploadError;
				platform->Message(HOST_MESSAGE, "Could not write remaining data while finishing upload!\n");
				break;
			}

			uploadLength -= len;
			uploadPointer += len;
		}
	}

	uploadPointer = NULL;
	uploadLength = 0;

	if (uploadState == uploadOK && !fileBeingUploaded.Flush())
	{
		uploadState = uploadError;
		platform->Message(HOST_MESSAGE, "Could not flush remaining data while finishing upload!\n");
	}

	// Check the file length is as expected
	if (uploadState == uploadOK && fileLength != 0 && fileBeingUploaded.Length() != fileLength)
	{
		uploadState = uploadError;
		platform->Message(HOST_MESSAGE, "Uploaded file size is different (%u vs. expected %u Bytes)!\n", fileBeingUploaded.Length(), fileLength);
	}

	// Close the file
	if (!fileBeingUploaded.Close())
	{
		uploadState = uploadError;
		platform->Message(HOST_MESSAGE, "Could not close the upload file while finishing upload!\n");
	}

	// Delete file if an error has occurred
	if (uploadState == uploadError && strlen(filenameBeingUploaded) != 0)
	{
		platform->GetMassStorage()->Delete("0:/", filenameBeingUploaded);
	}
	filenameBeingUploaded[0] = 0;
}



//********************************************************************************************
//
// *********************** HTTP interpreter for the Webserver class **************************
//
//********************************************************************************************

Webserver::HttpInterpreter::HttpInterpreter(Platform *p, Webserver *ws, Network *n)
	: ProtocolInterpreter(p, ws, n), state(doingCommandWord)
{
	uploadingTextData = false;
	numContinuationBytes = 0;
}

// File Uploads

bool Webserver::HttpInterpreter::DoFastUpload()
{
	bool success = ProtocolInterpreter::DoFastUpload();
	uploadedBytes += uploadLength;

	// See if we can finish it this time
	if (uploadedBytes >= postFileLength)
	{
		// Reset POST upload state for this client
		uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
		for(size_t i=0; i<numActiveSessions; i++)
		{
			if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
			{
				sessions[i].isPostUploading = false;
				break;
			}
		}

		// We're done, flush the remaining upload data and send the JSON response
		FinishUpload(postFileLength);
		SendJsonResponse("upload");
		uploadState = notUploading;
	}

	return success;
}

bool Webserver::HttpInterpreter::DoingFastUpload() const
{
	if (state == doingPost)
	{
		// Always finish the current request before checking for fast POST uploads
		return false;
	}

	uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
	uint16_t remotePort = network->GetTransaction()->GetRemotePort();
	for(size_t i=0; i<numActiveSessions; i++)
	{
		if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
		{
			return (remotePort == sessions[i].postPort);
		}
	}
	return false;
}

bool Webserver::HttpInterpreter::StartUpload(FileStore *file)
{
	numContinuationBytes = 0;
	return ProtocolInterpreter::StartUpload(file);
}

bool Webserver::HttpInterpreter::StoreUploadData(const char* data, unsigned int len)
{
	if (uploadState == uploadOK)
	{
		uploadPointer = data;
		uploadLength = len;

		// Count the number of UTF8 continuation bytes. We may need it to adjust the expected file length.
		if (uploadingTextData)
		{
			while (len != 0)
			{
				if ((*data & 0xC0) == 0x80)
				{
					++numContinuationBytes;
				}
				++data;
				--len;
			}
		}

		return true;
	}
	return false;
}

void Webserver::HttpInterpreter::CancelUpload()
{
	CancelUpload(network->GetTransaction()->GetRemoteIP());
}

void Webserver::HttpInterpreter::CancelUpload(uint32_t remoteIP)
{
	for(size_t i=0; i<numActiveSessions; i++)
	{
		if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
		{
			sessions[i].isPostUploading = false;
			sessions[i].lastQueryTime = platform->Time();
			break;
		}
	}

	ProtocolInterpreter::CancelUpload();
}

// Output to the client

// Start sending a file or a JSON response.
void Webserver::HttpInterpreter::SendFile(const char* nameOfFileToSend)
{
	if (StringEquals(nameOfFileToSend, "/"))
	{
		nameOfFileToSend = INDEX_PAGE;
	}
	FileStore *fileToSend = platform->GetFileStore(platform->GetWebDir(), nameOfFileToSend, false);
	if (fileToSend == NULL)
	{
		nameOfFileToSend = FOUR04_FILE;
		fileToSend = platform->GetFileStore(platform->GetWebDir(), nameOfFileToSend, false);
		if (fileToSend == NULL)
		{
			RejectMessage("not found", 404);
			return;
		}
	}

	NetworkTransaction *req = network->GetTransaction();
	req->Write("HTTP/1.1 200 OK\n");

	const char* contentType;
	bool zip = false;
	if (StringEndsWith(nameOfFileToSend, ".png"))
	{
		contentType = "image/png";
	}
	else if (StringEndsWith(nameOfFileToSend, ".ico"))
	{
		contentType = "image/x-icon";
	}
	else if (StringEndsWith(nameOfFileToSend, ".js"))
	{
		contentType = "application/javascript";
	}
	else if (StringEndsWith(nameOfFileToSend, ".css"))
	{
		contentType = "text/css";
	}
	else if (StringEndsWith(nameOfFileToSend, ".htm") || StringEndsWith(nameOfFileToSend, ".html"))
	{
		contentType = "text/html";
	}
	else if (StringEndsWith(nameOfFileToSend, ".zip"))
	{
		contentType = "application/zip";
		zip = true;
	}
	else
	{
		contentType = "application/octet-stream";
	}
	req->Printf("Content-Type: %s\n", contentType);

	if (zip && fileToSend != NULL)
	{
		req->Write("Content-Encoding: gzip\n");
		req->Printf("Content-Length: %lu", fileToSend->Length());
	}

	req->Write("Connection: close\n\n");
	network->SendAndClose(fileToSend);
}

void Webserver::HttpInterpreter::SendGCodeReply()
{
	NetworkTransaction *req = network->GetTransaction();
	req->Write("HTTP/1.1 200 OK\n");
	req->Write("Content-Type: text/plain\n");
	req->Printf("Content-Length: %u\n", reprap.GetGcodeReply().strlen());
	req->Write("Connection: close\n\n");
	req->Write(reprap.GetGcodeReply());
	network->SendAndClose(NULL);
}

void Webserver::HttpInterpreter::SendJsonResponse(const char* command)
{
	// rr_reply is treated differently, because it (currently) responds as "text/plain"
	if (IsAuthenticated() && StringEquals(command, "reply"))
	{
		SendGCodeReply();
		return;
	}

	// See if we can find a suitable JSON response
	NetworkTransaction *req = network->GetTransaction();
	bool keepOpen = false;
	bool mayKeepOpen;
	bool found;
	char jsonResponseBuffer[jsonReplyLength];
	StringRef jsonResponse(jsonResponseBuffer, ARRAY_SIZE(jsonResponseBuffer));
	if (numQualKeys == 0)
	{
		found = GetJsonResponse(command, jsonResponse, "", "", 0, mayKeepOpen);
	}
	else
	{
		found = GetJsonResponse(command, jsonResponse, qualifiers[0].key, qualifiers[0].value, qualifiers[1].key - qualifiers[0].value - 1, mayKeepOpen);
	}

	if (found)
	{
		jsonResponseBuffer[ARRAY_UPB(jsonResponseBuffer)] = 0;
		if (reprap.Debug(moduleWebserver))
		{
			platform->Message(HOST_MESSAGE, "JSON response: %s queued\n", jsonResponseBuffer);
		}
	}
	else
	{
		jsonResponseBuffer[0] = 0;
		platform->Message(HOST_MESSAGE, "KnockOut request: %s not recognised\n", command);
	}

	if (mayKeepOpen)
	{
		// Check that the browser wants to persist the connection too
		for (size_t i = 0; i < numHeaderKeys; ++i)
		{
			if (StringEquals(headers[i].key, "Connection"))
			{
// Comment out the following line to disable persistent connections
				keepOpen = StringEquals(headers[i].value, "keep-alive");
				break;
			}
		}
	}
	req->Write("HTTP/1.1 200 OK\n");
	req->Write("Content-Type: application/json\n");
	req->Printf("Content-Length: %u\n", jsonResponse.strlen());
	req->Printf("Connection: %s\n\n", keepOpen ? "keep-alive" : "close");
	req->Write(jsonResponse);

	network->SendAndClose(NULL, keepOpen);
}

//----------------------------------------------------------------------------------------------------

// Input from the client

// Get the Json response for this command.
// 'value' is null-terminated, but we also pass its length in case it contains embedded nulls, which matters when uploading files.
bool Webserver::HttpInterpreter::GetJsonResponse(const char* request, StringRef& response, const char* key, const char* value, size_t valueLength, bool& keepOpen)
{
	keepOpen = false;	// assume we don't want to persist the connection
	bool found = true;	// assume success

	if (!IsAuthenticated() && reprap.NoPasswordSet())
	{
		// Try to allocate a new HTTP session even if this is no connect request,
		// because we may need to know if a POST upload is in progress.
		Authenticate();
	}

	if (StringEquals(request, "connect") && StringEquals(key, "password"))
	{
		if (IsAuthenticated())
		{
			// This IP is already authenticated, no need to check the password again
			response.copy("{\"err\":0}");
		}
		else if (reprap.CheckPassword(value))
		{
			if (Authenticate())
			{
				// This is only possible if we have at least one HTTP session left
				response.copy("{\"err\":0}");
			}
			else
			{
				// Otherwise report an error
				response.copy("{\"err\":2}");
			}
		}
		else
		{
			// Wrong password
			response.copy("{\"err\":1}");
		}
	}
	else if (!IsAuthenticated())
	{
		// Don't respond if this IP is not authenticated
		found = false;
	}
	else
	{
		UpdateAuthentication();

		if (StringEquals(request, "disconnect"))
		{
			RemoveAuthentication();
			response.copy("{\"err\":0}");
		}
		else if (StringEquals(request, "status"))
		{
			int type = 0;
			if (StringEquals(key, "type"))
			{
				// New-style JSON status responses
				type = atoi(value);
				if (type < 1 || type > 3)
				{
					type = 1;
				}

				reprap.GetStatusResponse(response, type, -1, true);
			}
			else
			{
				// Deprecated
				reprap.GetLegacyStatusResponse(response, 1, 0);
			}
		}
		else if (StringEquals(request, "gcode") && StringEquals(key, "gcode"))
		{
			webserver->LoadGcodeBuffer(value);
			response.printf("{\"buff\":%u}", webserver->GetGcodeBufferSpace());
		}
		else if (StringEquals(request, "upload"))
		{
			response.printf("{\"err\":%d}", (uploadState == uploadOK && uploadedBytes == postFileLength) ? 0 : 1);
		}
		else if (StringEquals(request, "upload_begin") && StringEquals(key, "name"))
		{
			FileStore *file = platform->GetFileStore("0:/", value, true);
			if (StartUpload(file))
			{
				strncpy(filenameBeingUploaded, value, ARRAY_SIZE(filenameBeingUploaded));
				filenameBeingUploaded[ARRAY_UPB(filenameBeingUploaded)] = 0;
				uploadingTextData = (numQualKeys < 2 || !StringEquals(qualifiers[1].key, "type") ||
										!StringEquals(qualifiers[1].value, "binary"));
			}

			GetJsonUploadResponse(response);
		}
		else if (StringEquals(request, "upload_data") && StringEquals(key, "data"))
		{
			StoreUploadData(value, valueLength);

			GetJsonUploadResponse(response);
			keepOpen = true;
		}
		else if (StringEquals(request, "upload_end") && StringEquals(key, "size"))
		{
			uint32_t fileLength = strtoul(value, NULL, 10);
			FinishUpload(fileLength);

			GetJsonUploadResponse(response);
			uploadState = notUploading;
		}
		else if (StringEquals(request, "upload_cancel"))
		{
			CancelUpload();
			response.copy("{\"err\":0}");
		}
		else if (StringEquals(request, "delete") && StringEquals(key, "name"))
		{
			bool ok = platform->GetMassStorage()->Delete("0:/", value);
			response.printf("{\"err\":%d}", (ok) ? 0 : 1);
		}
		else if (StringEquals(request, "files"))
		{
			// TODO: get rid of GetFilesResponse and write directly to NetworkTransaction!
			const char* dir = (StringEquals(key, "dir")) ? value : platform->GetGCodeDir();
			reprap.GetFilesResponse(response, dir, false);
		}
		else if (StringEquals(request, "fileinfo"))
		{
			reprap.GetPrintMonitor()->GetFileInfoResponse(response, (StringEquals(key, "name")) ? value : NULL);
		}
		else if (StringEquals(request, "move"))
		{
			if (numQualKeys >= 2)
			{
				if (StringEquals(key, "old") && StringEquals(qualifiers[1].key, "new"))
				{
					response.printf("{\"err\":%d}", platform->GetMassStorage()->Rename(value, qualifiers[1].value) ? 1 : 0);
				}
				else
				{
					response.printf("{\"err\":1}");
				}
			}
			else
			{
				response.printf("{\"err\":1}");
			}
		}
		else if (StringEquals(request, "name"))
		{
			reprap.GetNameResponse(response);
		}
		else if (StringEquals(request, "connect"))
		{
			CancelUpload();
			reprap.GetStatusResponse(response, 1, -1, true);
		}
		else if (StringEquals(request, "mkdir") && StringEquals(key, "dir"))
		{
			bool ok = (platform->GetMassStorage()->MakeDirectory(value));
			response.printf("{\"err\":%d}", (ok) ? 0 : 1);
		}
		else if (StringEquals(request, "config"))
		{
			reprap.GetConfigResponse(response);
		}
		else
		{
			found = false;
		}
	}

	return found;
}

void Webserver::HttpInterpreter::GetJsonUploadResponse(StringRef& response)
{
	response.printf("{\"ubuff\":%u,\"err\":%d}", webUploadBufferSize, (uploadState == uploadOK) ? 0 : 1);
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

bool Webserver::HttpInterpreter::NeedMoreData()
{
	if (state == doingPost)
	{
		StoreUploadData(clientMessage + (clientPointer - uploadedBytes), uploadedBytes);
		ResetState();
		return false;
	}
	else if (!IsAuthenticated() && (numCommandWords == 0 || !StringEquals(commandWords[0], "connect")))
	{
		// It makes very little sense to allow unknown connections to block our HTTP reader, however
		// connect requests may take up more than one TCP_MSS if a long cookie value is passed.
		ResetState();
		return false;
	}
	return true;
}

void Webserver::HttpInterpreter::ResetSessions()
{
	numActiveSessions = 0;
}

void Webserver::HttpInterpreter::ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort)
{
	// Deal with aborted POST uploads. Note that we also check the remote port here,
	// because the client *might* have two instances of the web interface running.
	if (uploadState == uploadOK)
	{
		for(size_t i=0; i<numActiveSessions; i++)
		{
			if (sessions[i].ip == remoteIP && sessions[i].isPostUploading && sessions[i].postPort == remotePort)
			{
				if (reprap.Debug(moduleWebserver))
				{
					platform->Message(HOST_MESSAGE, "POST upload for '%s' has been cancelled!\n", filenameBeingUploaded);
				}
				CancelUpload(remoteIP);
				break;
			}
		}
	}
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
			++numCommandWords;
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
			clientMessage[clientPointer++] = decodeChar | c - ('A' - 10);
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
					ResetState();
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
				ResetState();
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

	case doingPost:
		clientMessage[clientPointer++] = c;
		uploadedBytes++;

		if (uploadedBytes == postFileLength)
		{
			StoreUploadData(clientMessage + (clientPointer - uploadedBytes), uploadedBytes);
			FinishUpload(postFileLength);

			SendJsonResponse("upload");

			// Reset state
			uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
			for(unsigned int i=0; i<numActiveSessions; i++)
			{
				if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
				{
					sessions[i].isPostUploading = false;
					sessions[i].lastQueryTime = platform->Time();
					break;
				}
			}
			uploadState = notUploading;
			ResetState();

			return true;
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
    	platform->Message(HOST_MESSAGE, "HTTP requests with %d command words:", numCommandWords);
    	for (unsigned int i = 0; i < numCommandWords; ++i)
    	{
    		platform->AppendMessage(HOST_MESSAGE, " %s", commandWords[i]);
    	}
    	platform->AppendMessage(HOST_MESSAGE, "\n");
    }

    if (numCommandWords < 2)
	{
		return RejectMessage("too few command words");
	}

	if (StringEquals(commandWords[0], "GET"))
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
			SendFile(commandWords[1]);
		}
		return true;
	}
	else if (StringEquals(commandWords[0], "POST"))
	{
		bool isUploadRequest = (StringEquals(commandWords[1], KO_START "upload"));
		isUploadRequest |= (commandWords[1][0] == '/' && StringEquals(commandWords[1] + 1, KO_START "upload"));
		if (isUploadRequest)
		{
			if (numQualKeys > 0 && StringEquals(qualifiers[0].key, "name"))
			{
				// We cannot upload more than one file at once
				if (uploadState == uploadOK)
				{
					return RejectMessage("cannot upload more than one file at once");
				}

				// See how many bytes we expect to read
				bool contentLengthFound = false;
				for(size_t i=0; i<numHeaderKeys; i++)
				{
					if (StringEquals(headers[i].key, "Content-Length"))
					{
						postFileLength = atoi(headers[i].value);
						contentLengthFound = true;
						break;
					}
				}

				// Start POST file upload
				if (contentLengthFound)
				{
					FileStore *file = platform->GetFileStore("0:/", qualifiers[0].value, true);
					if (StartUpload(file))
					{
						// Start new file upload
						uploadingTextData = false;
						uploadedBytes = numContinuationBytes = 0;

						strncpy(filenameBeingUploaded, qualifiers[0].value, ARRAY_SIZE(filenameBeingUploaded));
						filenameBeingUploaded[ARRAY_UPB(filenameBeingUploaded)] = 0;

						// Set POST variables only if we actually need to store data
						if (postFileLength > 0)
						{
							uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
							uint16_t remotePort = network->GetTransaction()->GetRemotePort();
							for(size_t i=0; i<numActiveSessions; i++)
							{
								if (sessions[i].ip == remoteIP)
								{
									sessions[i].postPort = remotePort;
									sessions[i].isPostUploading = true;
									break;
								}
							}
							state = doingPost;

							return false;
						}

						// User has uploaded an empty file - we should have finished here already
						FinishUpload(0);
						SendJsonResponse("upload");
						return true;
					}
					return RejectMessage("could not start file upload");
				}
			}
			return RejectMessage("invalid POST upload request");
		}
		return RejectMessage("only rr_upload is supported for POST requests");
	}
	else
	{
		return RejectMessage("Unknown message type");
	}
}

// Reject the current message. Always returns true to indicate that we should stop reading the message.
bool Webserver::HttpInterpreter::RejectMessage(const char* response, unsigned int code)
{
	platform->Message(HOST_MESSAGE, "Webserver: rejecting message with: %s\n", response);

	NetworkTransaction *req = network->GetTransaction();
	req->Printf("HTTP/1.1 %u %s\nConnection: close\n\n", code, response);
	network->SendAndClose(NULL);

	ResetState();

	return true;
}

// Authenticate current IP and return true on success
bool Webserver::HttpInterpreter::Authenticate()
{
	if (numActiveSessions < maxSessions)
	{
		sessions[numActiveSessions].isPostUploading = false;
		sessions[numActiveSessions].ip = network->GetTransaction()->GetRemoteIP();
		sessions[numActiveSessions].lastQueryTime = platform->Time();
		numActiveSessions++;
		return true;
	}
	return false;
}

bool Webserver::HttpInterpreter::IsAuthenticated() const
{
	uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
	for(size_t i=0; i<numActiveSessions; i++)
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
	uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
	for(size_t i=0; i<numActiveSessions; i++)
	{
		if (sessions[i].ip == remoteIP)
		{
			sessions[i].lastQueryTime = platform->Time();
			break;
		}
	}
}

void Webserver::HttpInterpreter::RemoveAuthentication()
{
	uint32_t remoteIP = network->GetTransaction()->GetRemoteIP();
	for(size_t i=numActiveSessions - 1; i>=0; i--)
	{
		if (sessions[i].ip == remoteIP)
		{
			for(int k=numActiveSessions - 1; k > i; k--)
			{
				memcpy(&sessions[k - 1], &sessions[k], sizeof(HttpSession));
			}
			numActiveSessions--;
			break;
		}
	}
}


void Webserver::HttpInterpreter::CheckSessions()
{
	const float time = platform->Time();
	for(int i=numActiveSessions - 1; i>=0; i--)
	{
		if (!sessions[i].isPostUploading && (time - sessions[i].lastQueryTime) > httpSessionTimeout)
		{
			for(int k=numActiveSessions - 1; k > i; k--)
			{
				memcpy(&sessions[k - 1], &sessions[k], sizeof(HttpSession));
			}
			numActiveSessions--;
		}
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
	strcpy(currentDir, "/");
}

void Webserver::FtpInterpreter::ConnectionEstablished()
{
	if (reprap.Debug(moduleWebserver))
	{
		platform->Message(HOST_MESSAGE, "Webserver: FTP connection established!\n");
	}

	NetworkTransaction *req = network->GetTransaction();

	switch (state)
	{
		case waitingForPasvPort:
			if (req->GetLocalPort() == ftpPort)
			{
				network->SendAndClose(NULL);
				return;
			}

			network->SaveDataConnection();
			state = pasvPortConnected;

			break;

		default:
			// I (zpl) wanted to allow only one active FTP session, but some FTP programs
			// like FileZilla open a second connection for transfers for some reason.
			if (req->GetLocalPort() == ftpPort)
			{
				req->Write("220 RepRapPro Ormerod\r\n");
				network->SendAndClose(NULL, true);

				ResetState();
			}

			break;
	}
}

void Webserver::FtpInterpreter::ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort)
{
	if (localPort != ftpPort)
	{
		// Close the data port
		network->CloseDataPort();

		// Send response
		if (network->AcquireFTPTransaction())
		{
			if (state == doingPasvIO)
			{
				if (uploadState != uploadError)
				{
					SendReply(226, "Transfer complete.");
				}
				else
				{
					SendReply(526, "Upload failed!");
				}
			}
			else
			{
				SendReply(550, "Lost data connection!");
			}
		}

		// Do file handling
		if (IsUploading())
		{
			FinishUpload(0U);
			uploadState = notUploading;
		}
		state = authenticated;
	}
}

bool Webserver::FtpInterpreter::CharFromClient(char c)
{
	if (clientPointer == ARRAY_UPB(clientMessage))
	{
		clientPointer = 0;
		platform->Message(HOST_MESSAGE, "Webserver: Buffer overflow in FTP server!\n");
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
				platform->Message(HOST_MESSAGE, "FtpInterpreter::ProcessLine called with state %d:\n%s\n", state, clientMessage);
			}

			if (clientPointer > 1) // only process a new line if we actually received data
			{
				ProcessLine();
				clientPointer = 0;
				return true;
			}

			if (reprap.Debug(moduleWebserver))
			{
				platform->Message(HOST_MESSAGE, "FtpInterpreter::ProcessLine call finished.\n");
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

	state = authenticating;
}

bool Webserver::FtpInterpreter::DoingFastUpload() const
{
	return (IsUploading() && network->GetTransaction()->GetLocalPort() == network->GetDataPort());
}

// return true if an error has occurred, false otherwise
void Webserver::FtpInterpreter::ProcessLine()
{
	switch (state)
	{
		case authenticating:
			// don't check the user name
			if (StringStartsWith(clientMessage, "USER"))
			{
				SendReply(331, "Please specify the password.");
			}
			// but check the password
			else if (StringStartsWith(clientMessage, "PASS"))
			{
				char pass[MaxPasswordLength + 1];
				size_t pass_length = 0;
				bool reading_pass = false;
				for(size_t i = 4; i < clientPointer && i < MaxPasswordLength + 4; i++)
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
			if (StringEquals(clientMessage, "SYST"))
			{
				SendReply(215, "UNIX Type: L8");
			}
			// get features
			else if (StringEquals(clientMessage, "FEAT"))
			{
				SendFeatures();
			}
			// get current dir
			else if (StringEquals(clientMessage, "PWD"))
			{
				snprintf(ftpResponse, ftpResponseLength, "\"%s\"", currentDir);
				SendReply(257, ftpResponse);
			}
			// set current dir
			else if (StringStartsWith(clientMessage, "CWD"))
			{
				ReadFilename(3);
				ChangeDirectory(filename);
			}
			// change to parent of current directory
			else if (StringEquals(clientMessage, "CDUP"))
			{
				ChangeDirectory("..");
			}
			// switch transfer mode (sends response, but doesn't have any effects)
			else if (StringStartsWith(clientMessage, "TYPE"))
			{
				for(int i=4; i<clientPointer; i++)
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
			else if (StringEquals(clientMessage, "PASV"))
			{
				/* get local IP address */
				const byte *ip_address = network->IPAddress();

				/* open random port > 1023 */
				rand();
				uint16_t pasv_port = random(1024, 65535);
				network->OpenDataPort(pasv_port);
				portOpenTime = platform->Time();
				state = waitingForPasvPort;

				/* send FTP response */
				snprintf(ftpResponse, ftpResponseLength, "Entering Passive Mode (%d,%d,%d,%d,%d,%d)",
						*ip_address++, *ip_address++, *ip_address++, *ip_address++,
						pasv_port / 256, pasv_port % 256);
				SendReply(227, ftpResponse);
			}
			// PASV commands are not supported in this state
			else if (StringEquals(clientMessage, "LIST") || StringStartsWith(clientMessage, "RETR") || StringStartsWith(clientMessage, "STOR"))
			{
				SendReply(425, "Use PASV first.");
			}
			// delete file
			else if (StringStartsWith(clientMessage, "DELE"))
			{
				ReadFilename(4);

				bool ok;
				if (filename[0] == '/')
				{
					ok = platform->GetMassStorage()->Delete(NULL, filename);
				}
				else
				{
					ok = platform->GetMassStorage()->Delete(currentDir, filename);
				}

				if (ok)
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

				bool ok;
				if (filename[0] == '/')
				{
					ok = platform->GetMassStorage()->Delete(NULL, filename);
				}
				else
				{
					ok = platform->GetMassStorage()->Delete(currentDir, filename);
				}

				if (ok)
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
				const char *location = (filename[0] == '/')
										? filename
											: platform->GetMassStorage()->CombineName(currentDir, filename);

				if (platform->GetMassStorage()->MakeDirectory(location))
				{
					snprintf(ftpResponse, ftpResponseLength, "\"%s\" created", location);
					SendReply(257, ftpResponse);
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
					const char *temp = platform->GetMassStorage()->CombineName(currentDir, filename);
					strncpy(filename, temp, MaxFilenameLength);
					filename[MaxFilenameLength - 1] = 0;
				}

				if (platform->GetMassStorage()->FileExists(filename))
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
				char oldFilename[MaxFilenameLength];
				strncpy(oldFilename, filename, MaxFilenameLength);
				oldFilename[MaxFilenameLength - 1] = 0;
				ReadFilename(4);

				// See where this file needs to be moved to
				if (filename[0] == '/')
				{
					if (platform->GetMassStorage()->Rename(oldFilename, filename))
					{
						SendReply(250, "Rename successful.");
					}
					else
					{
						SendReply(550, "Could not rename file or directory.");
					}
				}
				else
				{
					const char *newFilename = platform->GetMassStorage()->CombineName(currentDir, filename);
					if (platform->GetMassStorage()->Rename(oldFilename, newFilename))
					{
						SendReply(250, "Rename successful.");
					}
					else
					{
						SendReply(500, "Could not rename file or directory.");
					}
				}
			}
			// no op
			else if (StringEquals(clientMessage, "NOOP"))
			{
				SendReply(200, "NOOP okay.");
			}
			// end connection
			else if (StringEquals(clientMessage, "QUIT"))
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
			if (!reprap.Debug(moduleWebserver) && platform->Time() - portOpenTime > pasvPortTimeout)
			{
				SendReply(425, "Failed to establish connection.");

				network->CloseDataPort();
				state = authenticated;
			}
			else
			{
				network->WaitForDataConection();
			}

			break;

		case pasvPortConnected:
			// save current connection state so we can send '226 Transfer complete.' when ConnectionLost() is called
			network->SaveFTPConnection();

			// list directory entries
			if (StringEquals(clientMessage, "LIST"))
			{
				// send response via main port
				strncpy(ftpResponse, "150 Here comes the directory listing.\r\n", ftpResponseLength);
				NetworkTransaction *ftp_req = network->GetTransaction();
				ftp_req->Write(ftpResponse);
				network->SendAndClose(NULL, true);

				// send file list via data port
				if (network->AcquireDataTransaction())
				{
					FileInfo file_info;
					if (platform->GetMassStorage()->FindFirst(currentDir, file_info))
					{
						NetworkTransaction *data_req = network->GetTransaction();
						char line[300];

						do {
							// Example for a typical UNIX-like file list:
							// "drwxr-xr-x    2 ftp      ftp             0 Apr 11 2013 bin\r\n"
							char dirChar = (file_info.isDirectory) ? 'd' : '-';
							snprintf(line, ARRAY_SIZE(line), "%crw-rw-rw- 1 ftp ftp %13d %s %02d %04d %s\r\n",
									dirChar, file_info.size, platform->GetMassStorage()->GetMonthName(file_info.month),
									file_info.day, file_info.year, file_info.fileName);

							// Fortunately we don't need to bother with output buffer chunks any more...
							data_req->Write(line);
						} while (platform->GetMassStorage()->FindNext(file_info));
					}

					network->SendAndClose(NULL);
					state = doingPasvIO;
				}
				else
				{
					SendReply(500, "Unknown error.");
					network->CloseDataPort();
					state = authenticated;
				}
			}
			// upload a file
			else if (StringStartsWith(clientMessage, "STOR"))
			{
				FileStore *file;

				ReadFilename(4);
				if (filename[0] == '/')
				{
					file = platform->GetFileStore(NULL, filename, true);
				}
				else
				{
					file = platform->GetFileStore(currentDir, filename, true);
				}

				if (StartUpload(file))
				{
					strncpy(filenameBeingUploaded, filename, ARRAY_SIZE(filenameBeingUploaded));
					filenameBeingUploaded[ARRAY_UPB(filenameBeingUploaded)] = 0;

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
				FileStore *fs;

				ReadFilename(4);
				if (filename[0] == '/')
				{
					fs = platform->GetFileStore(NULL, filename, false);
				}
				else
				{
					fs = platform->GetFileStore(currentDir, filename, false);
				}

				if (fs == NULL)
				{
					SendReply(550, "Failed to open file.");
				}
				else
				{
					snprintf(ftpResponse, ftpResponseLength, "Opening data connection for %s (%lu bytes).", filename, fs->Length());
					SendReply(150, ftpResponse);

					if (network->AcquireDataTransaction())
					{
						// send the file via data port
						network->SendAndClose(fs, false);
						state = doingPasvIO;
					}
					else
					{
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
			if (StringEquals(clientMessage, "ABOR"))
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
	NetworkTransaction *req = network->GetTransaction();
	req->Printf("%d %s\r\n", code, message);
	network->SendAndClose(NULL, keepConnection);
}

void Webserver::FtpInterpreter::SendFeatures()
{
	NetworkTransaction *req = network->GetTransaction();
	req->Write("211-Features:\r\n");
	req->Write("PASV\r\n");		// support PASV mode
	req->Write("211 End\r\n");
	network->SendAndClose(NULL, true);
}

void Webserver::FtpInterpreter::ReadFilename(int start)
{
	int filenameLength = 0;
	bool readingPath = false;
	for(int i=start; i<clientPointer && filenameLength < MaxFilenameLength - 1; i++)
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
			strncpy(combinedPath, newDirectory, MaxFilenameLength);
			combinedPath[MaxFilenameLength - 1] = 0;
		}
		else // relative path
		{
			if (StringEquals(newDirectory, "..")) // go up
			{
				if (StringEquals(currentDir, "/"))
				{
					// we're already at the root, so we can't go up any more
					SendReply(550, "Failed to change directory.");
					return;
				}
				else
				{
					strncpy(combinedPath, currentDir, MaxFilenameLength);
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
				strncpy(combinedPath, currentDir, MaxFilenameLength);
				if (strlen(currentDir) > 1)
				{
					strncat(combinedPath, "/", MaxFilenameLength - strlen(combinedPath) - 1);
				}
				strncat(combinedPath, newDirectory, MaxFilenameLength - strlen(combinedPath) - 1);
			}
		}

		/* Make sure the new path does not end with a '/', because FatFs won't see the directory otherwise */
		if (StringEndsWith(combinedPath, "/") && strlen(combinedPath) > 1)
		{
			combinedPath[strlen(combinedPath) -1] = 0;
		}

		/* Verify path and change it */
		if (platform->GetMassStorage()->PathExists(combinedPath))
		{
			strncpy(currentDir, combinedPath, MaxFilenameLength);
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
	: ProtocolInterpreter(p, ws, n)
{
	ResetState();
}

void Webserver::TelnetInterpreter::ConnectionEstablished()
{
	NetworkTransaction *req = network->GetTransaction();
	req->Write("RepRapPro Ormerod Telnet Interface\r\n\r\n");
	req->Write("Please enter your password:\r\n");
	req->Write("> ");
	network->SendAndClose(NULL, true);
}

void Webserver::TelnetInterpreter::ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort)
{
	ResetState();
}

bool Webserver::TelnetInterpreter::CharFromClient(char c)
{
	if (clientPointer == ARRAY_UPB(clientMessage))
	{
		clientPointer = 0;
		platform->Message(HOST_MESSAGE, "Webserver: Buffer overflow in Telnet server!\n");
		return true;
	}

	switch (c)
	{
		case 0:
			break;

		case '\r':
		case '\n':
			if (clientPointer != 0)
			{
				clientMessage[clientPointer] = 0;
				ProcessLine();
				clientPointer = 0;
			}
			return true;

		case '#':
			// On Linux, the last character of the HELO message sent by Telnet is '#',
			// so clear the buffer if the user hasn't logged in yet.
			if (state == authenticating)
			{
				clientPointer = 0;
				break;
			}
			// no break

		default:
			clientMessage[clientPointer++] = c;
			break;
	}

	return false;
}

void Webserver::TelnetInterpreter::ResetState()
{
	state = authenticating;
	clientPointer = 0;
	sendPending = false;
}

void Webserver::TelnetInterpreter::ProcessLine()
{
	NetworkTransaction *req = network->GetTransaction();

	switch (state)
	{
		case authenticating:
			if (reprap.CheckPassword(clientMessage))
			{
				network->SaveTelnetConnection();
				state = authenticated;

				req->Write("Log in successful!\r\n");
				network->SendAndClose(NULL, true);
			}
			else
			{
				req->Write("Invalid password.\r\n> ");
				network->SendAndClose(NULL, true);
			}
			break;

		case authenticated:
			// Special commands for Telnet
			if (StringEquals(clientMessage, "exit") || StringEquals(clientMessage, "quit"))
			{
				req->Write("Goodbye.\r\n");
				network->SendAndClose(NULL);
			}
			// All other commands are processed by the Webserver
			else
			{
				webserver->ProcessGcode(clientMessage);
				if (sendPending)
				{
					sendPending = false;
					network->SendAndClose(NULL, true);
				}
				else
				{
					network->CloseTransaction();
				}
			}
			break;
	}
}

void Webserver::TelnetInterpreter::HandleGcodeReply(const char *reply)
{
	if (state >= authenticated && network->AcquireTelnetTransaction())
	{
		NetworkTransaction *req = network->GetTransaction();

		// Whenever a new line is read, we also need to send a carriage return
		while (*reply != 0)
		{
			if (*reply == '\n')
			{
				req->Write('\r');
			}
			req->Write(*reply);
			reply++;
		}

		// We must not send the message here, because then we'd have to deal with LWIP internals
		sendPending = true;
	}
}

// End
