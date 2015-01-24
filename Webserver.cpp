/****************************************************************************************************

 RepRapFirmware - Webserver

 This class serves a single-page web applications to the attached network.  This page forms the user's
 interface with the RepRap machine.  This software interprests returned values from the page and uses it
 to Generate G Codes, which it sends to the RepRap.  It also collects values from the RepRap like
 temperature and uses those to construct the web page.

 The page itself - reprap.htm - uses Knockout.js and Jquery.js.  See:

 http://knockoutjs.com/

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

 rr_connect	 Sent by the web interface software to establish an initial connection, indicating that
 	 	 	 any state variables relating to the web interface (e.g. file upload in progress) should
 	 	 	 be reset. Returns the same response as rr_status.

 rr_poll	 Returns the old-style status response. Not recommended because all the position,
 	 	 	 extruder position and temperature variables are returned in a single array, which means
 	 	 	 that the web interface has to know in advance how many heaters and extruders there are.
 	 	 	 Provided only for backwards compatibility with older web interface software. Likely to
 	 	 	 be removed in a future version.

 rr_status	 New-style status response, in which temperatures, axis positions and extruder positions
 	 	 	 are returned in separate variables. Another difference is that extruder positions are
 	 	 	 returned as absolute positions instead of relative to the previous gcode.

 rr_files?dir=xxx
 	 	 	 Returns a listing of the filenames in the /gcode directory of the SD card. 'dir' is a
 	 	 	 directory path relative to the root of the SD card. If the 'dir' variable is not present,
 	 	 	 it defaults to the /gcode directory.

 rr_axes	 Returns the axis lengths.

 rr_name	 Returns the machine name in variable myname.

 rr_password?password=xxx
 	 	 	 Returns variable "password" having value "right" if xxx is the correct password and
 	 	 	 "wrong" otherwise.

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
Webserver::Webserver(Platform* p) : platform(p), webserverActive(false), readingConnection(NULL),
		gcodeReply(gcodeReplyBuffer, ARRAY_SIZE(gcodeReplyBuffer))
{
	httpInterpreter = new HttpInterpreter(p, this);
	ftpInterpreter = new FtpInterpreter(p, this);
	telnetInterpreter = new TelnetInterpreter(p, this);
}

void Webserver::Init()
{
	// initialise the webserver class
	SetPassword(DEFAULT_PASSWORD);
	SetName(DEFAULT_NAME);

	gcodeReadIndex = gcodeWriteIndex = 0;
	lastTime = platform->Time();
	longWait = lastTime;
	gcodeReply[0] = 0;
	webserverActive = true;

	// initialise all protocol handlers
	httpInterpreter->ResetState();
	ftpInterpreter->ResetState();
	telnetInterpreter->ResetState();

	// Reinitialise the message file
	//platform->GetMassStorage()->Delete(platform->GetWebDir(), MESSAGE_FILE);
}


// Deal with input/output from/to the client (if any)
void Webserver::Spin()
{
	// Before we process an incoming Request, we must ensure that the webserver
	// is active and that all upload buffers are empty.
	if (webserverActive && httpInterpreter->FlushUploadData() && ftpInterpreter->FlushUploadData())
	{
		Network *net = reprap.GetNetwork();
		NetworkTransaction *req = net->GetTransaction(readingConnection);
		if (req != NULL)
		{
			// Process incoming request
			if (req->IsReady())
			{
				// I (zpl) have added support for two more protocols (FTP and Telnet),
				// so we must take care of the protocol type here.
				ProtocolInterpreter *interpreter;
				uint16_t local_port = req->GetLocalPort();
				bool is_data_port = false;
				switch (local_port)
				{
					case 80: 	/* HTTP */
						interpreter = httpInterpreter;
						break;

					case 21: 	/* FTP */
						interpreter = ftpInterpreter;
						break;

					case 23: 	/* Telnet */
						interpreter = telnetInterpreter;
						break;

					default:	/* FTP data */
						interpreter = ftpInterpreter;
						is_data_port = true;
						break;
				}

				TransactionStatus status = req->GetStatus();

				// For protocols other than HTTP it is important to send a HELO message
				if (status == connected)
				{
					interpreter->ConnectionEstablished();

					// Close this request unless ConnectionEstablished() has already used it for sending
					if (req == net->GetTransaction(readingConnection))
					{
						net->CloseTransaction();
					}
				}
				// Graceful disconnects are handled here, because prior NetworkTransactions might still contain valid
				// data. That's why it's a bad idea to close these connections immediately in the Network class.
				else if (status == disconnected)
				{
					// CloseRequest() will call the disconnect events and close the connection
					net->CloseTransaction();
				}
				// Fast upload for FTP connections
				else if (is_data_port)
				{
					if (interpreter->IsUploading())
					{
						char *buffer;
						unsigned int len;
						if (req->ReadBuffer(buffer, len))
						{
							interpreter->StoreUploadData(buffer, len);
						}
						else
						{
							net->CloseTransaction();
						}
					}
					else
					{
						platform->Message(HOST_MESSAGE, "Webserver: Closing invalid data connection\n");
						readingConnection = NULL;
						net->SendAndClose(NULL);
					}
				}
				// Process other messages
				else
				{
					char c;
					for (int i=0; i<500; i++)
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
							// This happens when the incoming message length exceeds the TCP MSS. We need to process another packet on the same connection.
							readingConnection = req->GetConnection();
							net->SendAndClose(NULL, true);
							break;
						}
					}
				}
			}
			else if (req->LostConnection())
			{
				platform->Message(HOST_MESSAGE, "Webserver: Skipping zombie request\n");
				net->CloseTransaction();
			}
		}

		platform->ClassReport("Webserver", longWait, moduleWebserver);
	}
}

void Webserver::Exit()
{
	httpInterpreter->CancelUpload();

	platform->Message(HOST_MESSAGE, "Webserver class exited.\n");
	webserverActive = false;
}

void Webserver::Diagnostics()
{
	platform->AppendMessage(BOTH_MESSAGE, "Webserver Diagnostics:\n");
}

void Webserver::SetPassword(const char* pw)
{
	// Users sometimes put a tab character between the password and the comment, so allow for this
	CopyParameterText(pw, password, ARRAY_SIZE(password));
}

void Webserver::SetName(const char* nm)
{
	// Users sometimes put a tab character between the machine name and the comment, so allow for this
	CopyParameterText(nm, myName, ARRAY_SIZE(myName));
}

// Copy some parameter text, stopping at the first control character or when the destination buffer is full, and removing trailing spaces
void Webserver::CopyParameterText(const char* src, char *dst, size_t length)
{
	size_t i;
	for (i = 0; i + 1 < length && src[i] >= ' '; ++i)
	{
		dst[i] = src[i];
	}
	// Remove any trailing spaces
	while (i > 0 && dst[i - 1] == ' ')
	{
		--i;
	}
	dst[i] = 0;
}

const char *Webserver::GetName() const
{
	return myName;
}

bool Webserver::CheckPassword(const char *pw) const
{
	return StringEquals(pw, password);
}

// Get the actual amount of gcode buffer space we have
unsigned int Webserver::GetGcodeBufferSpace() const
{
	return (gcodeReadIndex - gcodeWriteIndex - 1u) % gcodeBufferLength;
}

// Process a null-terminated gcode
// We intercept four G/M Codes so we can deal with file manipulation and emergencies.  That
// way things don't get out of sync, and - as a file name can contain
// a valid G code (!) - confusion is avoided.
void Webserver::ProcessGcode(const char* gc)
{
	if (StringStartsWith(gc, "M30 "))		// delete SD card file
	{
		reprap.GetGCodes()->DeleteFile(&gc[4]);
	}
	else if (StringStartsWith(gc, "M23 "))	// select SD card file to print next
	{
		reprap.StartingFilePrint(&gc[4]);
		reprap.GetGCodes()->QueueFileToPrint(&gc[4]);
	}
	else if (StringStartsWith(gc, "M112") && !isdigit(gc[4]))	// emergency stop
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
			MessageStringToWebInterface("Configuration file not found", true);
		}
		else
		{
			char c;
			size_t i = 0;
			bool reading_whitespace = false;
			while (i < gcodeReply.Length() - 1 && configFile->Read(c))
			{
				if (!reading_whitespace || (c != ' ' && c != '\t'))
				{
					gcodeReply[i++] = c;
				}
				reading_whitespace = (c == ' ' || c == '\t');
			}
			configFile->Close();
			gcodeReply[i] = 0;

			++seq;
			telnetInterpreter->HandleGcodeReply(gcodeReply.Pointer());
		}
	}
	else if (StringStartsWith(gc, "M25") && !isDigit(gc[3]))	// pause SD card print
	{
		reprap.GetGCodes()->PauseSDPrint();
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
	char gcodeTempBuf[GCODE_LENGTH];
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
		platform->Message(HOST_MESSAGE, "Webserver: GCode buffer overflow.\n");
		MessageStringToWebInterface("Webserver: GCode buffer overflow", true);
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
	// Inform protocol handlers that this connection has been lost
	uint16_t local_port = cs->GetLocalPort();
	ProtocolInterpreter *interpreter;
	switch (local_port)
	{
		case 80: /* HTTP */
			interpreter = httpInterpreter;
			break;

		case 21: /* FTP */
			interpreter = ftpInterpreter;
			break;

		case 23: /* Telnet */
			interpreter = telnetInterpreter;
			break;

		default: /* FTP data */
			interpreter = ftpInterpreter;
			break;
	}

	if (interpreter->DebugEnabled())
	{
		debugPrintf("Webserver: ConnectionLost called with port %d\n", local_port);
	}

	interpreter->ConnectionLost(local_port);

	// If our reading connection is lost, it will be no longer important which connection is read from first.
	if (cs == readingConnection)
	{
		readingConnection = NULL;
	}
}

void Webserver::MessageStringToWebInterface(const char *s, bool error)
{
	if (!webserverActive)
	{
		return;
	}

	if (strlen(s) == 0 && !error)
	{
		gcodeReply.copy("ok");
		telnetInterpreter->HandleGcodeReply("ok");
	}
	else
	{
		if (error)
		{
			gcodeReply.printf("Error: %s", s);
			telnetInterpreter->HandleGcodeReply("Error: ", true);
			telnetInterpreter->HandleGcodeReply(s);
		}
		else
		{
			gcodeReply.copy(s);
			telnetInterpreter->HandleGcodeReply(s);
		}
	}

	++seq;
}

void Webserver::AppendReplyToWebInterface(const char *s, bool error)
{
	if (!webserverActive)
	{
		return;
	}

	gcodeReply.cat(s);
	++seq;
	telnetInterpreter->HandleGcodeReply(s, true);
}


//********************************************************************************************
//
//********************** Generic Procotol Interpreter implementation *************************
//
//********************************************************************************************

ProtocolInterpreter::ProtocolInterpreter(Platform *p, Webserver *ws) : platform(p), webserver(ws)
{
	uploadState = notUploading;
	uploadPointer = NULL;
	uploadLength = 0;
	numContinuationBytes = 0;
	filenameBeingUploaded[0] = 0;
}

// Start writing to a new file
bool ProtocolInterpreter::StartUpload(FileStore *file)
{
	CancelUpload();
	numContinuationBytes = 0;

	if (file != NULL)
	{
		fileBeingUploaded.Set(file);
		uploadState = uploadOK;
		return true;
	}
	else
	{
		uploadState = uploadError;
		platform->Message(HOST_MESSAGE, "Could not open file while starting upload!\n");
		return false;
	}
}

// Process a received buffer of upload data
bool ProtocolInterpreter::StoreUploadData(const char* data, unsigned int len)
{
	if (uploadState == uploadOK)
	{
		uploadPointer = data;
		uploadLength = len;

		// Count the number of UTF8 continuation bytes. We will need it to adjust the expected file length.
		while (len != 0)
		{
			if ((*data & 0xC0) == 0x80)
			{
				++numContinuationBytes;
			}
			++data;
			--len;
		}
		return true;
	}
	return false;
}

// Try to flush upload buffer and return true if all data has been flushed
bool ProtocolInterpreter::FlushUploadData()
{
	if (uploadState == uploadOK && uploadLength != 0)
	{
		// Write some uploaded data to file, if possible half a sector (256 bytes) at a time
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

void ProtocolInterpreter::FinishUpload(uint32_t file_length)
{
	// Write the remaining data
	if (uploadState == uploadOK && uploadLength != 0)
	{
		if (!fileBeingUploaded.Write(uploadPointer, uploadLength))
		{
			uploadState = uploadError;
			platform->Message(HOST_MESSAGE, "Could not write remaining data while finishing upload!\n");
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
	if (uploadState == uploadOK && file_length != 0 && fileBeingUploaded.Length() != file_length + numContinuationBytes)
	{
		uploadState = uploadError;
		platform->Message(HOST_MESSAGE, "Uploaded file size is different!\n");
	}

	// Close the file
	if (!fileBeingUploaded.Close())
	{
		uploadState = uploadError;
		platform->Message(HOST_MESSAGE, "Could not close the upload file while finishing upload!\n");
	}

	// Delete file if an error has occured
	if (uploadState == uploadError && strlen(filenameBeingUploaded) != 0)
	{
		platform->GetMassStorage()->Delete("0:/", filenameBeingUploaded);
	}
	filenameBeingUploaded[0] = 0;
}

// This is overridden in class HttpInterpreter
bool ProtocolInterpreter::DebugEnabled() const
{
	return reprap.Debug(moduleWebserver);
}

//********************************************************************************************
//
// *********************** HTTP interpreter for the Webserver class **************************
//
//********************************************************************************************



Webserver::HttpInterpreter::HttpInterpreter(Platform *p, Webserver *ws)
	: ProtocolInterpreter(p, ws), state(doingCommandWord)
{
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

	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();
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
	net->SendAndClose(fileToSend);
}

void Webserver::HttpInterpreter::SendJsonResponse(const char* command)
{
	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();
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
	net->SendAndClose(NULL, keepOpen);
}

//----------------------------------------------------------------------------------------------------

// Input from the client

// Get the Json response for this command.
// 'value' is null-terminated, but we also pass its length in case it contains embedded nulls, which matter when uploading files.
bool Webserver::HttpInterpreter::GetJsonResponse(const char* request, StringRef& response, const char* key, const char* value, size_t valueLength, bool& keepOpen)
{
	bool found = true;	// assume success
	keepOpen = false;	// assume we don't want to persist the connection

	if (StringEquals(request, "status"))	// new style status request
	{
		reprap.GetStatusResponse(response, 1);
	}
	else if (StringEquals(request, "poll"))		// old style status request
	{
		reprap.GetStatusResponse(response, 0);
	}
	else if (StringEquals(request, "gcode") && StringEquals(key, "gcode"))
	{
		webserver->LoadGcodeBuffer(value);
		response.printf("{\"buff\":%u}", webserver->GetGcodeBufferSpace());
	}
	else if (StringEquals(request, "upload_begin") && StringEquals(key, "name"))
	{
		FileStore *file = platform->GetFileStore("0:/", value, true);
		StartUpload(file);
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
		uint32_t file_length = strtoul(value, NULL, 10);
		FinishUpload(file_length);

		GetJsonUploadResponse(response);
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
		const char* dir = (StringEquals(key, "dir")) ? value : platform->GetGCodeDir();
		reprap.GetFilesResponse(response, dir);
	}
	else if (StringEquals(request, "fileinfo"))
	{
		reprap.GetFileInfoResponse(response, (StringEquals(key, "name")) ? value : NULL);
	}
	else if (StringEquals(request, "name"))
	{
		reprap.GetNameResponse(response);
	}
	else if (StringEquals(request, "password") && StringEquals(key, "password"))
	{
		gotPassword = webserver->CheckPassword(value);
		response.printf("{\"password\":\"%s\"}", (gotPassword) ? "right" : "wrong");
	}
	else if (StringEquals(request, "axes"))
	{
		response.copy("{\"axes\":");
		char ch = '[';
		for (int8_t drive = 0; drive < AXES; drive++)
		{
			response.catf("%c%.1f", ch, platform->AxisTotalLength(drive));
			ch = ',';
		}
		response.cat("]}");
	}
	else if (StringEquals(request, "connect"))
	{
		CancelUpload();
		reprap.GetStatusResponse(response, 1);
	}
	else
	{
		found = false;
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
    if(reprap.Debug(moduleWebserver))
    {
    	platform->Message(HOST_MESSAGE, "HTTP request:");
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
		// We don't support POST yet
		return RejectMessage("POST not supported");
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

	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();
	req->Printf("HTTP/1.1 %u %s\nConnection: close\n\n", code, response);
	net->SendAndClose(NULL);

	ResetState();

	return true;
}


//********************************************************************************************
//
//************************* FTP interpreter for the Webserver class **************************
//
//********************************************************************************************

Webserver::FtpInterpreter::FtpInterpreter(Platform *p, Webserver *ws)
	: ProtocolInterpreter(p, ws), state(authenticating), clientPointer(0)
{
	strcpy(currentDir, "/");
}

void Webserver::FtpInterpreter::ConnectionEstablished()
{
	if (DebugEnabled())
	{
		platform->Message(DEBUG_MESSAGE, "Webserver: FTP connection established!\n");
	}

	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();

	switch (state)
	{
		case waitingForPasvPort:
			if (req->GetLocalPort() == 21)
			{
				net->SendAndClose(NULL);
				return;
			}

			net->SaveDataConnection();
			state = pasvPortConnected;

			break;

		default:
			// I (zpl) wanted to allow only one active FTP session, but some FTP programs
			// like FileZilla open a second connection for transfers for some reason.
			if (req->GetLocalPort() == 21)
			{
				req->Write("220 RepRapPro Ormerod\r\n");
				net->SendAndClose(NULL, true);

				ResetState();
			}

			break;
	}
}

void Webserver::FtpInterpreter::ConnectionLost(uint16_t local_port)
{
	if (local_port != 21)
	{
		// Close the data port
		Network *net = reprap.GetNetwork();
		net->CloseDataPort();

		// Send response
		if (net->AcquireFTPTransaction())
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

			if (DebugEnabled())
			{
				platform->Message(DEBUG_MESSAGE, "FtpInterpreter::ProcessLine called with state %d:\n%s\n", state, clientMessage);
			}

			if (clientPointer > 1) // only process a new line if we actually received data
			{
				ProcessLine();
				clientPointer = 0;
				return true;
			}

			if (DebugEnabled())
			{
				platform->Message(DEBUG_MESSAGE, "FtpInterpreter::ProcessLine call finished.\n");
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

	Network *net = reprap.GetNetwork();
	net->CloseDataPort();

	CancelUpload();

	state = authenticating;
}

// return true if an error has occurred, false otherwise
void Webserver::FtpInterpreter::ProcessLine()
{
	Network *net = reprap.GetNetwork();

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
				char pass[SHORT_STRING_LENGTH];
				int pass_length = 0;
				bool reading_pass = false;
				for(int i=4; i<clientPointer && i<SHORT_STRING_LENGTH +3; i++)
				{
					reading_pass |= (clientMessage[i] != ' ' && clientMessage[i] != '\t');
					if (reading_pass)
					{
						pass[pass_length++] = clientMessage[i];
					}
				}
				pass[pass_length] = 0;

				if (webserver->CheckPassword(pass))
				{
					state = authenticated;
					SendReply(230, "Login successful.");
				}
				else
				{
					SendReply(530, "Login incorrect.", false);
				}
			}
			// if this is different, disconnect immediately
			else
			{
				SendReply(500, "Unknown login command.", false);
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
				const byte *ip_address = reprap.GetPlatform()->IPAddress();

				/* open random port > 1024 */
				rand();
				uint16_t pasv_port = random(1024, 65535);
				net->OpenDataPort(pasv_port);
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
				SendReply(350, "Ready to RNTO.");
			}
			else if (StringStartsWith(clientMessage, "RNTO"))
			{
				char buffer[MaxFilenameLength];
				strncpy(buffer, filename, MaxFilenameLength);
				ReadFilename(4);

				if (buffer[0] != '/' && filename[0] != '/')
				{
					char old_filename[MaxFilenameLength];
					const char *new_filename;

					strncpy(old_filename, platform->GetMassStorage()->CombineName(currentDir, buffer), MaxFilenameLength);
					new_filename = platform->GetMassStorage()->CombineName(currentDir, filename);

					if (platform->GetMassStorage()->Rename(old_filename, new_filename))
					{
						SendReply(250, "Rename successful.");
					}
					else
					{
						SendReply(550, "Could not rename file or directory.");
					}
				}
				else if (buffer[0] == '/' && filename[0] == '/')
				{
					if (platform->GetMassStorage()->Rename(buffer, filename))
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
					SendReply(450, "Illegal file names.");
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
			if (!DebugEnabled() && platform->Time() - portOpenTime > pasvPortTimeout)
			{
				SendReply(425, "Failed to establish connection.");

				net->CloseDataPort();
				state = authenticated;
			}
			else
			{
				net->RepeatTransaction();
			}

			break;

		case pasvPortConnected:
			// save current connection state so we can send '226 Transfer complete.' when ConnectionLost() is called
			net->SaveFTPConnection();

			// list directory entries
			if (StringEquals(clientMessage, "LIST"))
			{
				// send response via main port
				strncpy(ftpResponse, "150 Here comes the directory listing.\r\n", ftpResponseLength);
				NetworkTransaction *ftp_req = net->GetTransaction();
				ftp_req->Write(ftpResponse);
				net->SendAndClose(NULL, true);

				// send file list via data port
				if (net->AcquireDataTransaction())
				{
					FileInfo file_info;
					if (platform->GetMassStorage()->FindFirst(currentDir, file_info))
					{
						NetworkTransaction *data_req = net->GetTransaction();
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
					net->SendAndClose(NULL);
					state = doingPasvIO;
				}
				else
				{
					SendReply(500, "Unknown error.");
					net->CloseDataPort();
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
					SendReply(150, "OK to send data.");
					state = doingPasvIO;
				}
				else
				{
					SendReply(550, "Failed to open file.");
					net->CloseDataPort();
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

					if (net->AcquireDataTransaction())
					{
						// send the file via data port
						net->SendAndClose(fs, false);
						state = doingPasvIO;
					}
					else
					{
						SendReply(500, "Unknown error.");
						net->CloseDataPort();
						state = authenticated;
					}
				}
			}
			// unknown command
			else
			{
				SendReply(500, "Unknown command.");
				net->CloseDataPort();
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
					net->CloseDataPort();
					SendReply(226, "ABOR successful.");
				}
			}
			// unknown command
			else
			{
				SendReply(500, "Unknown command.");
				net->CloseDataPort();
				state = authenticated;
			}

			break;
	}
}

void Webserver::FtpInterpreter::SendReply(int code, const char *message, bool keepConnection)
{
	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();
	req->Printf("%d %s\r\n", code, message);
	net->SendAndClose(NULL, keepConnection);
}

void Webserver::FtpInterpreter::SendFeatures()
{
	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();
	req->Write("211-Features:\r\n");
	req->Write("PASV\r\n");		// support PASV mode
	req->Write("211 End\r\n");
	net->SendAndClose(NULL, true);
}

void Webserver::FtpInterpreter::ReadFilename(int start)
{
	int filenameLength = 0;
	bool readingPath = false;
	for(int i=start; i<clientPointer && i<MaxFilenameLength + start -1; i++)
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

Webserver::TelnetInterpreter::TelnetInterpreter(Platform *p, Webserver *ws) : ProtocolInterpreter(p, ws)
{
	ResetState();
}

void Webserver::TelnetInterpreter::ConnectionEstablished()
{
	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();
	req->Write("RepRapPro Ormerod Telnet Interface\r\n\r\n");
	req->Write("Please enter your password:\r\n");
	req->Write("> ");
	net->SendAndClose(NULL, true);
}

void Webserver::TelnetInterpreter::ConnectionLost(uint16_t local_port)
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
}

void Webserver::TelnetInterpreter::ProcessLine()
{
	Network *net = reprap.GetNetwork();
	NetworkTransaction *req = net->GetTransaction();

	switch (state)
	{
		case authenticating:
			if (webserver->CheckPassword(clientMessage))
			{
				net->SaveTelnetConnection();
				state = authenticated;

				req->Write("Log in successful!\r\n");
				net->SendAndClose(NULL, true);
			}
			else
			{
				req->Write("Invalid password.\r\n> ");
				net->SendAndClose(NULL, true);
			}
			break;

		case authenticated:
			// Special commands for Telnet
			if (StringEquals(clientMessage, "exit") || StringEquals(clientMessage, "quit"))
			{
				req->Write("Goodbye.\r\n");
				net->SendAndClose(NULL);
			}
			// All other commands are processed by the Webserver
			else
			{
				webserver->ProcessGcode(clientMessage);
				net->CloseTransaction();
			}
			break;
	}
}

void Webserver::TelnetInterpreter::HandleGcodeReply(const char *reply, bool haveMore)
{
	Network *net = reprap.GetNetwork();
	if (state >= authenticated && net->AcquireTelnetTransaction())
	{
		NetworkTransaction *req = net->GetTransaction();

		// Whenever a new line is read, we also need to send a carriage return
		bool append_line = false;
		while (reply[0] != 0)
		{
			append_line = true;
			if (reply[0] == '\n')
			{
				req->Write('\r');
				append_line = false;
			}
			req->Write(reply[0]);
			reply++;
		}

		// Only append a line break if reply didn't contain one and if we aren't expecting any more data
		if (append_line && !haveMore)
		{
			req->Write("\r\n");
		}

		net->SendAndClose(NULL, true);
	}
}


//********************************************************************************************
//
// ************************ Helper methods for the Webserver class ***************************
//
//********************************************************************************************

// Get information for a file on the SD card
bool Webserver::GetFileInfo(const char *directory, const char *fileName, GcodeFileInfo& info)
{
	FileStore *f = reprap.GetPlatform()->GetFileStore(directory, fileName, false);
	if (f != NULL)
	{
		// Try to find the object height by looking for the last G1 Zxxx command in the file
		info.fileSize = f->Length();
		info.objectHeight = 0.0;
		info.layerHeight = 0.0;
		info.numFilaments = DRIVES - AXES;
		info.generatedBy[0] = 0;

		if (info.fileSize != 0 && (StringEndsWith(fileName, ".gcode") || StringEndsWith(fileName, ".g") || StringEndsWith(fileName, ".gco") || StringEndsWith(fileName, ".gc")))
		{
			const size_t readSize = 512;					// read 512 bytes at a time (1K doesn't seem to work when we read from the end)
			const size_t overlap = 100;
			char buf[readSize + overlap + 1];				// need the +1 so we can add a null terminator
			bool foundLayerHeight = false;
			unsigned int filamentsFound = 0, nFilaments;
			float filaments[DRIVES - AXES];

			// Get slic3r settings by reading from the start of the file. We only read the first 2K or so, everything we are looking for should be there.
			for (unsigned int i = 0; i < 4; ++i)
			{
				size_t sizeToRead = (size_t)min<unsigned long>(info.fileSize, readSize + overlap);
				int nbytes = f->Read(buf, sizeToRead);
				if (nbytes == (int)sizeToRead)
				{
					buf[sizeToRead] = 0;

					// Search for filament usage (Cura puts it at the beginning of a G-code file)
					nFilaments = FindFilamentUsed(buf, sizeToRead, filaments, DRIVES - AXES);
					if (nFilaments != 0 && nFilaments >= filamentsFound)
					{
						filamentsFound = min<unsigned int>(nFilaments, info.numFilaments);
						for (unsigned int i = 0; i < filamentsFound; ++i)
						{
							info.filamentNeeded[i] = filaments[i];
						}
					}

					// Look for layer height
					foundLayerHeight = FindLayerHeight(buf, sizeToRead, info.layerHeight);

					// Look for slicer or Simplify3D generated-by comment program
					const char* generatedByString = " generated by ";
					const char* pos = strstr(buf, generatedByString);
					size_t generatedByLength = ARRAY_SIZE(info.generatedBy);
					if (pos != NULL)
					{
						pos += strlen(generatedByString);
						size_t i = 0;
						while (i < ARRAY_SIZE(info.generatedBy) - 1 && *pos >= ' ')
						{
							char c = *pos++;
							if (c == '"' || c == '\\')
							{
								// Need to escape the quote-mark for JSON
								if ( i > ARRAY_SIZE(info.generatedBy) - 3)
								{
									break;
								}
								info.generatedBy[i++] = '\\';
							}
							info.generatedBy[i++] = c;
						}
						info.generatedBy[i] = 0;
					}

					// Look for Cura comment
					const char* slicedAtString = ";Sliced at: ";
					pos = strstr(buf, slicedAtString);
					if (pos != NULL)
					{
						pos += strlen(slicedAtString);
						strcpy(info.generatedBy, "Cura at ");
						size_t i = 8;
						while (i < ARRAY_SIZE(info.generatedBy) - 1 && *pos >= ' ')
						{
							char c = *pos++;
							if (c == '"' || c == '\\')
							{
								// Need to escape the quote-mark for JSON
								if (i > ARRAY_SIZE(info.generatedBy) - 3)
								{
									break;
								}
								info.generatedBy[i++] = '\\';
							}
							info.generatedBy[i++] = c;
						}
						info.generatedBy[i] = 0;
					}

					// Add code to look for other values here...
				}
			}

			// Now get the object height and filament used by reading the end of the file
			{
				bool searchForFilaments = (filamentsFound == 0);
				size_t sizeToRead;
				if (info.fileSize <= readSize + overlap)
				{
					sizeToRead = info.fileSize;						// read the whole file in one go
				}
				else
				{
					sizeToRead = info.fileSize % readSize;
					if (sizeToRead <= overlap)
					{
						sizeToRead += readSize;
					}
				}
				unsigned long seekPos = info.fileSize - sizeToRead;	// read on a 512b boundary
				size_t sizeToScan = sizeToRead;
				for (;;)
				{
					if (!f->Seek(seekPos))
					{
						break;
					}
					int nbytes = f->Read(buf, sizeToRead);
					if (nbytes != (int)sizeToRead)
					{
						break;									// read failed so give up
					}
					buf[sizeToScan] = 0;						// add a null terminator

					// Search for filament used
					if (searchForFilaments)
					{
						nFilaments = FindFilamentUsed(buf, sizeToScan, filaments, DRIVES - AXES);
						if (nFilaments != 0 && nFilaments >= filamentsFound)
						{
							filamentsFound = min<unsigned int>(nFilaments, info.numFilaments);
							for (unsigned int i = 0; i < filamentsFound; ++i)
							{
								info.filamentNeeded[i] = filaments[i];
							}
						}
					}

					// Search for layer height
					if (!foundLayerHeight)
					{
						foundLayerHeight = FindLayerHeight(buf, sizeToScan, info.layerHeight);
					}

					// Search for object height
					if (FindHeight(buf, sizeToScan, info.objectHeight))
					{
						break;		// quit if found height
					}
					if (seekPos == 0 || info.fileSize - seekPos >= 200000uL)	// scan up to about the last 200K of the file (32K wasn't enough)
					{
						break;		// quit if reached start of file or already scanned the last 32K of the file
					}
					seekPos -= readSize;
					sizeToRead = readSize;
					sizeToScan = readSize + overlap;
					memcpy(buf + sizeToRead, buf, overlap);
				}
				info.numFilaments = filamentsFound;
			}
		}
		f->Close();
//debugPrintf("Set height %f and filament %f\n", height, filamentUsed);
		return true;
	}
	return false;
}

// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated. Simplify3D uses G0 Z instead.
bool Webserver::FindHeight(const char* buf, size_t len, float& height)
{
//debugPrintf("Scanning %u bytes starting %.100s\n", len, buf);
	if (len >= 5)			// need to check this here to prevent arithmetic underflow later on
	{
		size_t i = len - 5;
		for(;;)
		{
			if (buf[i] == 'G' && (buf[i + 1] == '1' || buf[i + 1] == '0') && buf[i + 2] == ' ' && buf[i + 3] == 'Z' && isDigit(buf[i + 4]))
			{
				// Looks like we found a command to set the height, however it could be in a comment, especially when using slic3r 1.1.1
				size_t j = i;
				while (j != 0)
				{
					--j;
					char c = buf[j];
					if (c == '\n' || c == '\r')
					{
//debugPrintf("Found at offset %u text: %.100s\n", i, &buf[i + 4]);
						// It's not in a comment
						height = strtod(&buf[i + 4], NULL);
						return true;
					}
					if (c == ';')
					{
						// It is in a comment, so give up on this one
						break;
					}
				}
			}
			if (i == 0)
			{
				break;
			}
			--i;
		}
	}
	return false;
}

// Scan the buffer for the layer height. The buffer is null-terminated.
bool Webserver::FindLayerHeight(const char *buf, size_t len, float& layerHeight)
{
	// Look for layer_height as generated by Slic3r
	const char* layerHeightStringSlic3r = "; layer_height ";
	const char *pos = strstr(buf, layerHeightStringSlic3r);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringSlic3r);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		layerHeight = strtod(pos, NULL);
		return true;
	}

	// Look for layer height as generated by Cura
	const char* layerHeightStringCura = "Layer height: ";
	pos = strstr(buf, layerHeightStringCura);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringCura);
		while (strchr(" \t=:", *pos))
		{
			++pos;
		}
		layerHeight = strtod(pos, NULL);
		return true;
	}

	// Look for layer height as generated by S3D
	const char* layerHeightStringS3D = "layerHeight,";
	pos = strstr(buf, layerHeightStringS3D);
	if (pos != NULL)
	{
		pos += strlen(layerHeightStringS3D);
		layerHeight = strtod(pos, NULL);
		return true;
	}
	return false;
}

// Scan the buffer for the filament used. The buffer is null-terminated.
// Returns the number of filaments found.
unsigned int Webserver::FindFilamentUsed(const char* buf, size_t len, float *filamentUsed, unsigned int maxFilaments)
{
	unsigned int filamentsFound = 0;

	// Look for filament usage as generated by Slic3r and Cura
	const char* filamentUsedStr = "ilament used";		// comment string used by slic3r, followed by filament used and "mm"
	const char* p = buf;
	while (filamentsFound < maxFilaments && (p = strstr(p, filamentUsedStr)) != NULL)
	{
		p += strlen(filamentUsedStr);
		while(strchr(" :=\t", *p) != NULL)
		{
			++p;	// this allows for " = " from default slic3r comment and ": " from default Cura comment
		}
		if (isDigit(*p))
		{
			char* q;
			filamentUsed[filamentsFound] = strtod(p, &q);
			if (*q == 'm' && *(q + 1) != 'm')
			{
				filamentUsed[filamentsFound] *= 1000.0;		// Cura outputs filament used in metres not mm
			}
			++filamentsFound;
		}
	}

	if (filamentsFound == 0)
	{
		// Look for filament usage as generated by S3D
		const char *filamentLengthStr = "ilament length";	// comment string used by S3D
		p = buf;
		while (filamentsFound < maxFilaments &&	(p = strstr(p, filamentLengthStr)) != NULL)
		{
			p += strlen(filamentLengthStr);
			while(strchr(" :=\t", *p) != NULL)
			{
				++p;
			}
			if (isDigit(*p))
			{
				filamentUsed[filamentsFound] = strtod(p, NULL); // S3D reports filament usage in mm, no conversion needed
				++filamentsFound;
			}
		}
	}

	return filamentsFound;
}

// End
