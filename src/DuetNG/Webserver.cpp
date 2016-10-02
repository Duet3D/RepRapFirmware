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

 rr_upload?name=xxx
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

const char* overflowResponse = "overflow";
const char* badEscapeResponse = "bad escape";

// Constructor and initialisation
Webserver::Webserver(Platform* p, Network *n) : state(doingFilename), platform(p), network(n), numSessions(0), clientsServed(0)
{
	gcodeReadIndex = gcodeWriteIndex = 0;
	gcodeReply = new OutputStack();
	processingDeferredRequest = false;
	seq = 0;
	webserverActive = false;
}

void Webserver::Init()
{
	// initialise the webserver class
	longWait = platform->Time();
	webserverActive = true;
	numSessions = clientsServed = 0;
	uploadIp = 0;

	// initialise all protocol handlers
	ResetState();
}

// Deal with input/output from/to the client (if any)
void Webserver::Spin()
{
	if (webserverActive)
	{
		uint32_t ip;
		size_t length;
		uint32_t fragment;
		const char* request = network->GetRequest(ip, length, fragment);
		if (request != nullptr)
		{
			if (reprap.Debug(moduleWebserver))
			{
				platform->MessageF(HOST_MESSAGE, "Request: %s fragment %u\n", request, fragment);
			}

			uint32_t fragNum = fragment & ~lastFragmentFlag;
			if (fragNum == 0)
			{
				HttpSession *session = StartSession(ip);
				if (session == nullptr)
				{
					network->SendReply(ip, 400, "Too many sessions");
				}
				else
				{
					// First fragment, so parse the request
					ResetState();
					const char *error = nullptr;
					bool finished = false;
					while (!finished && length != 0)
					{
						finished = CharFromClient(*request++, error);
						--length;
					}

					if (!finished)
					{
						error = "Incomplete command";
					}

					if (error != nullptr)
					{
						network->SendReply(ip, 400, error);
					}
					else
					{
						ProcessFirstFragment(*session, clientMessage, (fragment & lastFragmentFlag) != 0);
					}
				}
			}
			else
			{
				HttpSession *session = FindSession(ip);
				if (session != nullptr)
				{
					ProcessUploadFragment(*session, request, length, fragment);
				}
				else
				{
					// Discard the message
					network->DiscardMessage();
					platform->MessageF(DEBUG_MESSAGE, "session not found, fragment=%u\n", fragment);
				}
			}
		}
	}
	CheckSessions();
	platform->ClassReport(longWait);
}

// This is called to process a file upload request.
void Webserver::StartUpload(HttpSession& session, const char* fileName, uint32_t fileLength)
{
	CancelUpload(session);
	if (uploadIp != 0)
	{
		session.uploadState = uploadBusy;					// upload buffer already in use
	}
	else
	{
		FileStore *file = platform->GetFileStore(FS_PREFIX, fileName, true);
		if (file == nullptr)
		{
			session.uploadState = cantCreate;
		}
		else
		{
			session.fileBeingUploaded.Set(file);
			session.postLength = fileLength;
			session.bytesWritten = 0;
			session.nextFragment = 1;
			session.uploadState = uploading;
			session.bufNumber = 0;
			session.bufPointer = 0;
			uploadIp = session.ip;
		}
	}
}

// This is called when we have received the last upload packet
void Webserver::FinishUpload(HttpSession& session)
{
	bool b = session.fileBeingUploaded.Close();

	if (session.uploadState == uploading)
	{
		if (!b)
		{
			session.uploadState = cantClose;
		}
		else if (session.bytesWritten != session.postLength)
		{
			session.uploadState = wrongLength;
		}
	}

	if (session.uploadState != uploading)
	{
		platform->MessageF(HOST_MESSAGE, "Error: Upload finished in state %d\n", (int)session.uploadState);
	}

	network->SendReply(session.ip, 200 | rcJson, (session.uploadState == uploading) ? "{\"err\":0}" : "{\"err\":1}");
	session.uploadState = notUploading;
}

void Webserver::CancelUpload(HttpSession& session)
{
	if (session.fileBeingUploaded.IsLive())
	{
		session.fileBeingUploaded.Close();
		//TODO delete the file as well
	}
	if (uploadIp == session.ip)
	{
		uploadIp = 0;
	}
}

void Webserver::ProcessUploadFragment(HttpSession& session, const char* request, size_t length, uint32_t fragment)
{
	if (session.uploadState == uploading)
	{
		//platform->MessageF(HOST_MESSAGE, "writing fragment=%u\n", fragment);
		uint32_t frag = fragment & ~lastFragmentFlag;
		if (frag != session.nextFragment)
		{
			platform->MessageF(HOST_MESSAGE, "expecting fragment %u received %u\n", session.nextFragment, frag);
			session.uploadState = wrongFragment;
		}
	}

	if (session.uploadState == uploading)
	{
		size_t bytesToCopy = uploadBufLength - session.bufPointer;
		if (bytesToCopy > length)
		{
			bytesToCopy = length;
		}
		memcpy(reinterpret_cast<uint8_t*>(uploadBuffers[session.bufNumber]) + session.bufPointer, request, bytesToCopy);
		session.bufPointer += bytesToCopy;
		if (session.bufPointer == uploadBufLength)
		{
			// Switch to the other buffer. We know the rest of the data will fit because the buffer size is bigger than the SPI packet size.
			session.bufNumber ^= 1;
			if (bytesToCopy < length)
			{
				memcpy(uploadBuffers[session.bufNumber], request + bytesToCopy, length - bytesToCopy);
			}
			session.bufPointer = length - bytesToCopy;

			// Free up the SPI buffer before we start the file write
			if ((fragment & lastFragmentFlag) == 0)
			{
				network->DiscardMessage();
			}

			if (session.fileBeingUploaded.Write(reinterpret_cast<const char*>(uploadBuffers[session.bufNumber ^ 1]), uploadBufLength))
			{
				session.bytesWritten += uploadBufLength;
			}
			else
			{
				session.uploadState = cantWrite;
			}
		}
		else if ((fragment & lastFragmentFlag) == 0)
		{
			network->DiscardMessage();					// free up the SPI buffer
		}
	}
	else if ((fragment & lastFragmentFlag) == 0)
	{
		network->DiscardMessage();					// free up the SPI buffer
	}
	++session.nextFragment;

	if ((fragment & lastFragmentFlag) != 0)
	{
		if (session.bufPointer != 0 && session.uploadState == uploading)
		{
			if (session.fileBeingUploaded.Write(reinterpret_cast<const char*>(uploadBuffers[session.bufNumber]), session.bufPointer))
			{
				session.bytesWritten += session.bufPointer;
			}
			else
			{
				session.uploadState = cantWrite;
			}

		}
		FinishUpload(session);
	}
}

// Start a new session for this requester. Return nullptr if on more sessions available.
Webserver::HttpSession *Webserver::StartSession(uint32_t ip)
{
	HttpSession *s = FindSession(ip);
	if (s != nullptr)
	{
		// Abandon the existing session for this requester and start a new one
		s->nextFragment = 0;
		s->fileBeingUploaded.Close();		// TODO delete any partially-uploaded file
		return s;
	}

	// Find an empty session
	if (numSessions < maxHttpSessions)
	{
		s = &sessions[numSessions];
		s->ip = ip;
		s->isAuthenticated = false;
		s->nextFragment = 0;
		s->fileBeingUploaded.Close();			// make sure no file is open
		s->lastQueryTime = millis();
		++numSessions;
		return s;
	}
	return nullptr;
}

// Find an existing session for this requester, returning nullptr if there isn't one
Webserver::HttpSession *Webserver::FindSession(uint32_t ip)
{
	for (size_t i = 0; i < numSessions; ++i)
	{
		HttpSession *s = &sessions[i];
		if (s->ip == ip)
		{
			s->lastQueryTime = millis();
			return s;
		}
	}

	return nullptr;
}

// Delete a session
void Webserver::DeleteSession(uint32_t ip)
{
	for (size_t i = 0; i < numSessions; ++i)
	{
		HttpSession *s = &sessions[i];
		if (s->ip == ip)
		{
			s->fileBeingUploaded.Close();		// TODO delete it as well
			for (size_t j = i + 1; j < numSessions; ++j)
			{
				memcpy(&sessions[j - 1], &sessions[j], sizeof(HttpSession));
			}
			--numSessions;
			break;
		}
	}
	if (uploadIp == ip)
	{
		uploadIp = 0;					// free the upload buffer
	}
}

void Webserver::Exit()
{
	platform->Message(GENERIC_MESSAGE, "Webserver class exited.\n");
	webserverActive = false;
}

void Webserver::ResetState()
{
	clientPointer = 0;
	state = doingFilename;
	numQualKeys = 0;
	processingDeferredRequest = false;
}

void Webserver::Diagnostics(MessageType mtype)
{
	platform->Message(mtype, "=== Webserver ===\n");
	platform->MessageF(mtype, "HTTP sessions: %d of %d\n", numSessions, maxHttpSessions);
}

bool Webserver::GCodeAvailable(const WebSource source) const
{
	switch (source)
	{
	case WebSource::HTTP:
		return gcodeReadIndex != gcodeWriteIndex;

	case WebSource::Telnet:
		// Telnet not supported
		return false;
	}

	return false;
}

char Webserver::ReadGCode(const WebSource source)
{
	switch (source)
	{
	case WebSource::HTTP:
		if (gcodeReadIndex != gcodeWriteIndex)
		{
			char c = gcodeBuffer[gcodeReadIndex];
			gcodeReadIndex = (gcodeReadIndex + 1u) % gcodeBufferLength;
			return c;
		}
		break;

	case WebSource::Telnet:
		// Telnet not supported
		return 0;
	}

	return 0;
}

void Webserver::HandleGCodeReply(const WebSource source, OutputBuffer *reply)
{
	switch (source)
	{
	case WebSource::HTTP:
#if 0
		OutputBuffer::ReleaseAll(reply);
#else
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
#endif
		break;

	case WebSource::Telnet:
		// Telnet not supported
	default:
		OutputBuffer::ReleaseAll(reply);
		break;
	}
}

void Webserver::HandleGCodeReply(const WebSource source, const char *reply)
{
	switch (source)
	{
	case WebSource::HTTP:
#if 0
#else
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
			seq++;
		}
#endif
		break;

	case WebSource::Telnet:
	default:
		break;
	}
}

//----------------------------------------------------------------------------------------------------

// Process the first fragment of input from the client.
// Return true if the session should be kept open.
bool Webserver::ProcessFirstFragment(HttpSession& session, const char* command, bool isOnlyFragment)
{
	// Get the first two key/value pairs
	const char* key1 = (numQualKeys >= 1) ? qualifiers[0].key : "";
	const char* value1 = (numQualKeys >= 1) ? qualifiers[0].value : "";
	const char* key2 = (numQualKeys >= 2) ? qualifiers[1].key : "";
	const char* value2 = (numQualKeys >= 2) ? qualifiers[1].value : "";

	// Process connect messages first
	if (StringEquals(command, "connect") && StringEquals(key1, "password"))
	{
		OutputBuffer *response;
		if (OutputBuffer::Allocate(response))
		{
			if (session.isAuthenticated || reprap.CheckPassword(value1))
			{
				// Password OK
				session.isAuthenticated = true;
				response->printf("{\"err\":0,\"sessionTimeout\":%u,\"boardType\":\"%s\"}", httpSessionTimeout, platform->GetBoardString());
			}
			else
			{
				// Wrong password
				response->copy("{\"err\":1}");
			}
			network->SendReply(session.ip, 200 | rcJson, response);
		}
		else
		{
			// If we failed to allocate an output buffer, send back an error string
			network->SendReply(session.ip, 200 | rcJson, "{\"err\":2}");
		}
		return false;
	}

	if (StringEquals(command, "disconnect"))
	{
		network->SendReply(session.ip, 200 | rcJson, "{\"err\":0}");
		DeleteSession(session.ip);
		return false;
	}

	// Try to authorise the user automatically to retain compatibility with the old web interface
	if (!session.isAuthenticated && reprap.NoPasswordSet())
	{
		session.isAuthenticated = true;
	}

	// If the client is still not authenticated, stop here
	if (!session.isAuthenticated)
	{
		network->SendReply(session.ip, 500, "Not authorized");
		return false;
	}

	if (StringEquals(command, "reply"))
	{
		SendGCodeReply(session);
		return false;
	}

	// rr_configfile sends the config as plain text well
	if (StringEquals(command, "configfile"))
	{
		const char *configPath = platform->GetMassStorage()->CombineName(platform->GetSysDir(), platform->GetConfigFile());
		char fileName[FILENAME_LENGTH];
		strncpy(fileName, configPath, FILENAME_LENGTH);

		SendFile(fileName, session);
		return false;
	}

	if (StringEquals(command, "download") && StringEquals(key1, "name"))
	{
		SendFile(value1, session);
		return false;
	}

	if (StringEquals(command, "upload"))
	{
		if (StringEquals(key1, "name") && StringEquals(key2, "length"))
		{
			// Deal with file upload request
			uint32_t fileLength = atol(value2);
			StartUpload(session, value1, fileLength);
			if (session.uploadState == uploading)
			{
				if (isOnlyFragment)
				{
					FinishUpload(session);
					return false;
				}
				else
				{
					network->DiscardMessage();		// no reply needed yet
					return true;		// expecting more data
				}
			}
		}

		network->SendReply(session.ip, 200 | rcJson, "{\"err\":1}");	// TODO return the cause of the error
		return false;
	}

	if (StringEquals(command, "move"))
	{
		const char* response =  "{\"err\":1}";		// assume failure
		if (StringEquals(key1, "old") && StringEquals(key2, "new"))
		{
			bool success = platform->GetMassStorage()->Rename(value1, value2);
			if (success)
			{
				response =  "{\"err\":0}";
			}
		}
		network->SendReply(session.ip, 200 | rcJson, response);
		return false;
	}

	if (StringEquals(command, "mkdir"))
	{
		const char* response =  "{\"err\":1}";		// assume failure
		if (StringEquals(key1, "dir"))
		{
			bool ok = (platform->GetMassStorage()->MakeDirectory(value1));
			if (ok)
			{
				response =  "{\"err\":0}";
			}
		}
		network->SendReply(session.ip, 200 | rcJson, response);
		return false;
	}

	if (StringEquals(command, "delete"))
	{
		const char* response =  "{\"err\":1}";		// assume failure
		if (StringEquals(key1, "name"))
		{
			bool ok = platform->GetMassStorage()->Delete("0:/", value1);
			if (ok)
			{
				response =  "{\"err\":0}";
			}
		}
		network->SendReply(session.ip, 200 | rcJson, response);
		return false;
	}

	// The remaining commands use an OutputBuffer for the response
	OutputBuffer *response = nullptr;
	if (StringEquals(command, "status"))
	{
		int type = 0;
		if (StringEquals(key1, "type"))
		{
			// New-style JSON status responses
			type = atoi(value1);
			if (type < 1 || type > 3)
			{
				type = 1;
			}

			response = reprap.GetStatusResponse(type, ResponseSource::HTTP);
		}
		else
		{
			response = reprap.GetLegacyStatusResponse(1, 0);
		}
	}
	else if (StringEquals(command, "gcode"))
	{
		if (StringEquals(key1, "gcode"))
		{
			LoadGcodeBuffer(value1);
			if (OutputBuffer::Allocate(response))
			{
				response->printf("{\"buff\":%u}", GetGCodeBufferSpace());
			}
		}
		else
		{
			network->SendReply(session.ip, 200 | rcJson, "{\"err\":1}");
			return false;
		}
	}
	else if (StringEquals(command, "filelist") && StringEquals(key1, "dir"))
	{
		response = reprap.GetFilelistResponse(value1);
	}
	else if (StringEquals(command, "files"))
	{
		const char* dir = (StringEquals(key1, "dir")) ? value1 : platform->GetGCodeDir();
		bool flagDirs = false;
		if (numQualKeys >= 2)
		{
			if (StringEquals(qualifiers[1].key, "flagDirs"))
			{
				flagDirs = StringEquals(qualifiers[1].value, "1");
			}
		}
		response = reprap.GetFilesResponse(dir, flagDirs);
	}
	else if (StringEquals(command, "fileinfo"))
	{
		if (reprap.GetPrintMonitor()->GetFileInfoResponse(StringEquals(key1, "name") ? value1 : nullptr, response))
		{
			processingDeferredRequest = false;
		}
		else
		{
			processingDeferredRequest = true;
		}
	}
	else if (StringEquals(command, "config"))
	{
		response = reprap.GetConfigResponse();
	}
	else
	{
		platform->MessageF(HOST_MESSAGE, "KnockOut request: %s not recognised\n", command);
		network->SendReply(session.ip, 400, "Unknown rr_ command");
		return false;
	}

	if (response != nullptr)
	{
		network->SendReply(session.ip, 200 | rcJson, response);
	}
	else if (!processingDeferredRequest)
	{
		network->SendReply(session.ip, 500, "No buffer available");
	}
	return processingDeferredRequest;
}

void Webserver::SendGCodeReply(HttpSession& session)
{
	if (gcodeReply->IsEmpty())
	{
		network->SendReply(session.ip, 200, "");
	}
	else
	{
		clientsServed++;
		if (clientsServed < numSessions)
		{
			gcodeReply->IncreaseReferences(1);
			network->SendReply(session.ip, 200, gcodeReply->GetFirstItem());
		}
		else
		{
			network->SendReply(session.ip, 200, gcodeReply->Pop());
		}

		if (reprap.Debug(moduleWebserver))
		{
			platform->MessageF(HOST_MESSAGE, "Serving client %d of %d\n", clientsServed, numSessions);
		}
	}
}

void Webserver::SendFile(const char *nameOfFileToSend, HttpSession& session)
{
	FileStore *file = platform->GetFileStore(FS_PREFIX, nameOfFileToSend, false);
	if (file == nullptr)
	{
		network->SendReply(session.ip, 400, "File not found");
	}
	else
	{
		// Send an initial message containing the response code and the file size (needed for the Content-length field)
		network->SendReply(session.ip, 200, file);		// tell the network layer to send the file
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
//		We have finished processing the message. No more characters may be read from this message.
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
bool Webserver::CharFromClient(char c, const char* &error)
{
	switch(state)
	{
	case doingFilename:
		switch(c)
		{
		case '\0':
		case ' ':
		case '\t':
		case '\n':
		case '\r':
			clientMessage[clientPointer++] = 0;
			numQualKeys = 0;
			error = nullptr;
			return true;
		case '?':
			clientMessage[clientPointer++] = 0;
			numQualKeys = 0;
			qualifiers[0].key = clientMessage + clientPointer;
			state = doingQualifierKey;
			break;
		case '%':
			state = doingFilenameEsc1;
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
			error = "bad qualifier key";
			return true;
		default:
			clientMessage[clientPointer++] = c;
			break;
		}
		break;

	case doingQualifierValue:
		switch(c)
		{
		case '\0':
		case '\n':
		case ' ':
		case '\t':
		case '\r':
			clientMessage[clientPointer++] = 0;
			qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
			error = nullptr;
			return true;
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
				error = "too many keys in qualifier";
				return true;
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
			error = badEscapeResponse;
			return true;
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
			error = badEscapeResponse;
			return true;
		}
		break;
	}

	if (clientPointer == ARRAY_SIZE(clientMessage))
	{
		error = overflowResponse;
		return true;
	}
	return false;
}

void Webserver::CheckSessions()
{
	// Check if any HTTP session can be purged
	const uint32_t time = millis();
	for (size_t i = 0; i < numSessions; ++i)
	{
		if ((time - sessions[i].lastQueryTime) > httpSessionTimeout)
		{
			DeleteSession(sessions[i].ip);
			clientsServed++;	// assume the disconnected client hasn't fetched the G-Code reply yet
		}
	}

	// If we cannot send the G-Code reply to anyone, we may free up some run-time space by dumping it
	if (numSessions == 0 || clientsServed >= numSessions)
	{
		while (!gcodeReply->IsEmpty())
		{
			OutputBuffer::ReleaseAll(gcodeReply->Pop());
		}
		clientsServed = 0;
	}
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
					platform->Message(HOST_MESSAGE, "Error: GCode local buffer overflow in HTTP webserver.\n");
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

// Process a null-terminated gcode
// We intercept one M Codes so we can deal with emergencies.  That
// way things don't get out of sync, and - as a file name can contain
// a valid G code (!) - confusion is avoided.
void Webserver::ProcessGcode(const char* gc)
{
	if (StringStartsWith(gc, "M112") && !isdigit(gc[4]))	// emergency stop
	{
		reprap.EmergencyStop();
		gcodeReadIndex = gcodeWriteIndex;					// clear the buffer
		reprap.GetGCodes()->Reset();
	}
	else
	{
		StoreGcodeData(gc, strlen(gc) + 1);
	}
}

// Process a received string of gcodes
void Webserver::StoreGcodeData(const char* data, uint16_t len)
{
	if (len > GetGCodeBufferSpace())
	{
		platform->Message(HOST_MESSAGE, "Error: GCode buffer overflow in HTTP Webserver!\n");
	}
	else
	{
		uint16_t remaining = gcodeBufferLength - gcodeWriteIndex;
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

// End
