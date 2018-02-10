/*
 * HttpResponder.cpp
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#include "HttpResponder.h"
#include "Socket.h"
#include "GCodes/GCodes.h"
#include "PrintMonitor.h"
#include "Libraries/General/IP4String.h"

#define KO_START "rr_"
const size_t KoFirst = 3;

const char* const overflowResponse = "overflow";
const char* const badEscapeResponse = "bad escape";

const uint32_t HttpReceiveTimeout = 2000;

HttpResponder::HttpResponder(NetworkResponder *n) : NetworkResponder(n)
{
}

// Ask the responder to accept this connection, returns true if it did
bool HttpResponder::Accept(Socket *s, NetworkProtocol protocol)
{
	if (responderState == ResponderState::free && protocol == HttpProtocol)
	{
		// Make sure we can get an output buffer before we accept the connection, or we won't be able to reply
		if (outBuf != nullptr || OutputBuffer::Allocate(outBuf))
		{
			responderState = ResponderState::reading;
			skt = s;
			timer = millis();

			// Reset the parse state variables
			clientPointer = 0;
			parseState = HttpParseState::doingCommandWord;
			numCommandWords = 0;
			numQualKeys = 0;
			numHeaderKeys = 0;
			commandWords[0] = clientMessage;

			if (reprap.Debug(moduleWebserver))
			{
				debugPrintf("HTTP connection accepted\n");
			}
			return true;
		}
		if (reprap.Debug(moduleWebserver))
		{
			debugPrintf("HTTP connection refused (no buffers)\n");
		}
	}
	return false;
}

// Do some work, returning true if we did anything significant
bool HttpResponder::Spin()
{
	switch (responderState)
	{
	case ResponderState::free:
		return false;

	case ResponderState::reading:
		{
			bool readSomething = false;
			char c;
			while (skt->ReadChar(c))
			{
				if (CharFromClient(c))
				{
					timer = millis();		// restart the timeout
					return true;
				}
				readSomething = true;
			}

			// Here when we were not able to read a character but we didn't receive a finished message
			if (readSomething)
			{
				timer = millis();			// restart the timeout
				return true;
			}

			if (!skt->CanRead() || millis() - timer >= HttpReceiveTimeout)
			{
				ConnectionLost();
				return true;
			}

			return false;
		}

	case ResponderState::gettingFileInfoLock:
		if (!fileInfoLock.Acquire(this))
		{
			return false;
		}
		responderState = ResponderState::gettingFileInfo;
		// no break

	case ResponderState::gettingFileInfo:
		if (SendFileInfo())
		{
			fileInfoLock.Release(this);					// release the lock
		}
		return true;

	case ResponderState::uploading:
		DoUpload();
		return true;

	case ResponderState::sending:
		SendData();
		return true;

	default:	// should not happen
		return false;
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
bool HttpResponder::CharFromClient(char c)
{
	switch (parseState)
	{
	case HttpParseState::doingCommandWord:
		switch(c)
		{
		case '\n':
			clientMessage[clientPointer++] = 0;
			++numCommandWords;
			numHeaderKeys = 0;
			headers[0].key = clientMessage + clientPointer;
			parseState = HttpParseState::doingHeaderKey;
			break;
		case '\r':
			break;
		case ' ':
		case '\t':
			clientMessage[clientPointer++] = 0;
			if (numCommandWords < MaxCommandWords)
			{
				++numCommandWords;
				commandWords[numCommandWords] = clientMessage + clientPointer;
				if (numCommandWords == 1)
				{
					parseState = HttpParseState::doingFilename;
				}
			}
			else
			{
				RejectMessage("too many command words");
				return true;
			}
			break;
		default:
			clientMessage[clientPointer++] = c;
			break;
		}
		break;

	case HttpParseState::doingFilename:
		switch(c)
		{
		case '\n':
			clientMessage[clientPointer++] = 0;
			++numCommandWords;
			numQualKeys = 0;
			numHeaderKeys = 0;
			headers[0].key = clientMessage + clientPointer;
			parseState = HttpParseState::doingHeaderKey;
			break;
		case '?':
			clientMessage[clientPointer++] = 0;
			++numCommandWords;
			numQualKeys = 0;
			qualifiers[0].key = clientMessage + clientPointer;
			parseState = HttpParseState::doingQualifierKey;
			break;
		case '%':
			parseState = HttpParseState::doingFilenameEsc1;
			break;
		case '\r':
			break;
		case ' ':
		case '\t':
			clientMessage[clientPointer++] = 0;
			if (numCommandWords < MaxCommandWords)
			{
				++numCommandWords;
				commandWords[numCommandWords] = clientMessage + clientPointer;
				parseState = HttpParseState::doingCommandWord;
			}
			else
			{
				RejectMessage("too many command words");
				return true;
			}
			break;
		default:
			clientMessage[clientPointer++] = c;
			break;
		}
		break;

	case HttpParseState::doingQualifierKey:
		switch(c)
		{
		case '=':
			clientMessage[clientPointer++] = 0;
			qualifiers[numQualKeys].value = clientMessage + clientPointer;
			++numQualKeys;
			parseState = HttpParseState::doingQualifierValue;
			break;
		case '\n':	// key with no value
		case ' ':
		case '\t':
		case '\r':
		case '%':	// none of our keys needs escaping, so treat an escape within a key as an error
		case '&':	// key with no value
			RejectMessage("bad qualifier key");
			return true;
		default:
			clientMessage[clientPointer++] = c;
			break;
		}
		break;

	case HttpParseState::doingQualifierValue:
		switch(c)
		{
		case '\n':
			clientMessage[clientPointer++] = 0;
			qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
			numHeaderKeys = 0;
			headers[0].key = clientMessage + clientPointer;
			parseState = HttpParseState::doingHeaderKey;
			break;
		case ' ':
		case '\t':
			clientMessage[clientPointer++] = 0;
			qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
			commandWords[numCommandWords] = clientMessage + clientPointer;
			parseState = HttpParseState::doingCommandWord;
			break;
		case '\r':
			break;
		case '%':
			parseState = HttpParseState::doingQualifierValueEsc1;
			break;
		case '&':
			// Another variable is coming
			clientMessage[clientPointer++] = 0;
			qualifiers[numQualKeys].key = clientMessage + clientPointer;	// so that we can read the whole value even if it contains a null
			if (numQualKeys < MaxQualKeys)
			{
				parseState = HttpParseState::doingQualifierKey;
			}
			else
			{
				RejectMessage("too many keys in qualifier");
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

	case HttpParseState::doingFilenameEsc1:
	case HttpParseState::doingQualifierValueEsc1:
		if (c >= '0' && c <= '9')
		{
			decodeChar = (c - '0') << 4;
			parseState = (HttpParseState)((int)parseState + 1);
		}
		else if (c >= 'A' && c <= 'F')
		{
			decodeChar = (c - ('A' - 10)) << 4;
			parseState = (HttpParseState)((int)parseState + 1);
		}
		else
		{
			RejectMessage(badEscapeResponse);
			return true;
		}
		break;

	case HttpParseState::doingFilenameEsc2:
	case HttpParseState::doingQualifierValueEsc2:
		if (c >= '0' && c <= '9')
		{
			clientMessage[clientPointer++] = decodeChar | (c - '0');
			parseState = (HttpParseState)((int)parseState - 2);
		}
		else if (c >= 'A' && c <= 'F')
		{
			clientMessage[clientPointer++] = decodeChar | (c - ('A' - 10));
			parseState = (HttpParseState)((int)parseState - 2);
		}
		else
		{
			RejectMessage(badEscapeResponse);
			return true;
		}
		break;

	case HttpParseState::doingHeaderKey:
		switch(c)
		{
		case '\n':
			if (clientMessage + clientPointer == headers[numHeaderKeys].key)	// if the key hasn't started yet, then this is the blank line at the end
			{
				ProcessMessage();
				return true;
			}
			else
			{
				RejectMessage("unexpected newline");
				return true;
			}
			break;
		case '\r':
			break;
		case ':':
			if (numHeaderKeys == MaxHeaders - 1)
			{
				RejectMessage("too many header key-value pairs");
				return true;
			}
			clientMessage[clientPointer++] = 0;
			headers[numHeaderKeys].value = clientMessage + clientPointer;
			++numHeaderKeys;
			parseState = HttpParseState::expectingHeaderValue;
			break;
		default:
			clientMessage[clientPointer++] = c;
			break;
		}
		break;

	case HttpParseState::expectingHeaderValue:
		if (c == ' ' || c == '\t')
		{
			break;		// ignore spaces between header key and value
		}
		parseState = HttpParseState::doingHeaderValue;
		// no break

	case HttpParseState::doingHeaderValue:
		if (c == '\n')
		{
			parseState = HttpParseState::doingHeaderContinuation;
		}
		else if (c != '\r')
		{
			clientMessage[clientPointer++] = c;
		}
		break;

	case HttpParseState::doingHeaderContinuation:
		switch(c)
		{
		case ' ':
		case '\t':
			// It's a continuation of the previous value
			clientMessage[clientPointer++] = c;
			parseState = HttpParseState::doingHeaderValue;
			break;
		case '\n':
			// It's the blank line
			clientMessage[clientPointer] = 0;
			ProcessMessage();
			return true;
		case '\r':
			break;
		default:
			// It's a new key
			if (clientPointer + 3 <= ARRAY_SIZE(clientMessage))
			{
				clientMessage[clientPointer++] = 0;
				headers[numHeaderKeys].key = clientMessage + clientPointer;
				clientMessage[clientPointer++] = c;
				parseState = HttpParseState::doingHeaderKey;
			}
			else
			{
				RejectMessage(overflowResponse);
				return true;
			}
			break;
		}
		break;

	default:
		break;
	}

	if (clientPointer == ARRAY_SIZE(clientMessage))
	{
		RejectMessage(overflowResponse);
		return true;
	}
	return false;
}

// Get the Json response for this command.
// 'value' is null-terminated, but we also pass its length in case it contains embedded nulls, which matters when uploading files.
// Return true if we generated a json response to send, false if we didn't and changed the state instead
bool HttpResponder::GetJsonResponse(const char* request, OutputBuffer *&response, bool& keepOpen)
{
	keepOpen = false;	// assume we don't want to persist the connection
	if (StringEquals(request, "connect") && GetKeyValue("password") != nullptr)
	{
		if (!CheckAuthenticated())
		{
			if (!reprap.CheckPassword(GetKeyValue("password")))
			{
				// Wrong password
				response->copy("{\"err\":1}");
				reprap.GetPlatform().MessageF(LogMessage, "HTTP client %s attempted login with incorrect password\n", IP4String(GetRemoteIP()).c_str());
				return true;
			}
			if (!Authenticate())
			{
				// No more HTTP sessions available
				response->copy("{\"err\":2}");
				reprap.GetPlatform().MessageF(LogMessage, "HTTP client %s attempted login but no more sessions available\n", IP4String(GetRemoteIP()).c_str());
				return true;
			}
		}

		// Client has been logged in
		response->printf("{\"err\":0,\"sessionTimeout\":%" PRIu32 ",\"boardType\":\"%s\"}", HttpSessionTimeout, GetPlatform().GetBoardString());
		reprap.GetPlatform().MessageF(LogMessage, "HTTP client %s login succeeded\n", IP4String(GetRemoteIP()).c_str());

		// See if we can update the current RTC date and time
		const char* const timeString = GetKeyValue("time");
		if (timeString != nullptr && !GetPlatform().IsDateTimeSet())
		{
			struct tm timeInfo;
			memset(&timeInfo, 0, sizeof(timeInfo));
			if (strptime(timeString, "%Y-%m-%dT%H:%M:%S", &timeInfo) != nullptr)
			{
				GetPlatform().SetDateTime(mktime(&timeInfo));
			}
		}
	}
	else if (!CheckAuthenticated())
	{
		RejectMessage("Not authorized", 500);
		return false;
	}
	else if (StringEquals(request, "disconnect"))
	{
		response->printf("{\"err\":%d}", (RemoveAuthentication()) ? 0 : 1);
		reprap.GetPlatform().MessageF(LogMessage, "HTTP client %s disconnected\n", IP4String(GetRemoteIP()).c_str());
	}
	else if (StringEquals(request, "status"))
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
	else if (StringEquals(request, "gcode") && GetKeyValue("gcode") != nullptr)
	{
		RegularGCodeInput * const httpInput = reprap.GetGCodes().GetHTTPInput();
		httpInput->Put(HttpMessage, GetKeyValue("gcode"));
		response->printf("{\"buff\":%u}", httpInput->BufferSpaceLeft());
	}
	else if (StringEquals(request, "upload"))
	{
		response->printf("{\"err\":%d}", (uploadError) ? 1 : 0);
	}
	else if (StringEquals(request, "delete") && GetKeyValue("name") != nullptr)
	{
		const bool ok = GetPlatform().GetMassStorage()->Delete(FS_PREFIX, GetKeyValue("name"));
		response->printf("{\"err\":%d}", (ok) ? 0 : 1);
	}
	else if (StringEquals(request, "filelist") && GetKeyValue("dir") != nullptr)
	{
		OutputBuffer::Release(response);
		response = reprap.GetFilelistResponse(GetKeyValue("dir"));
	}
	else if (StringEquals(request, "files"))
	{
		const char* dir = GetKeyValue("dir");
		if (dir == nullptr)
		{
			dir = GetPlatform().GetGCodeDir();
		}
		const char* const flagDirsVal = GetKeyValue("flagDirs");
		const bool flagDirs = flagDirsVal != nullptr && atoi(flagDirsVal) == 1;
		OutputBuffer::Release(response);
		response = reprap.GetFilesResponse(dir, flagDirs);
	}
	else if (StringEquals(request, "fileinfo"))
	{
		const char* const nameVal = GetKeyValue("name");
		if (nameVal != nullptr)
		{
			// Regular rr_fileinfo?name=xxx call
			SafeStrncpy(filenameBeingProcessed, nameVal, ARRAY_SIZE(filenameBeingProcessed));
		}
		else
		{
			// Simple rr_fileinfo call to get info about the file being printed
			filenameBeingProcessed[0] = 0;
		}
		responderState = ResponderState::gettingFileInfoLock;
		return false;
	}
	else if (StringEquals(request, "move"))
	{
		const char* const oldVal = GetKeyValue("old");
		const char* const newVal = GetKeyValue("new");
		bool success = false;
		if (oldVal != nullptr && newVal != nullptr)
		{
			MassStorage * const ms = GetPlatform().GetMassStorage();
			if (StringEquals(GetKeyValue("deleteexisting"), "yes") && ms->FileExists(oldVal) && ms->FileExists(newVal))
			{
				ms->Delete(nullptr, newVal, true);
			}
			success = ms->Rename(oldVal, newVal);
		}
		response->printf("{\"err\":%d}", (success) ? 0 : 1);
	}
	else if (StringEquals(request, "mkdir"))
	{
		const char* const dirVal = GetKeyValue("dir");
		bool success = false;
		if (dirVal != nullptr)
		{
			success = GetPlatform().GetMassStorage()->MakeDirectory(dirVal);
		}
		response->printf("{\"err\":%d}", (success) ? 0 : 1);
	}
	else if (StringEquals(request, "config"))
	{
		OutputBuffer::Release(response);
		response = reprap.GetConfigResponse();
	}
	else
	{
		RejectMessage("Unknown request", 500);
		return false;
	}
	return true;
}

const char* HttpResponder::GetKeyValue(const char *key) const
{
	for (size_t i = 0; i < numQualKeys; ++i)
	{
		if (StringEquals(qualifiers[i].key, key))
		{
			return qualifiers[i].value;
		}
	}
	return nullptr;
}

// Called to process a FileInfo request, which may take several calls
// When we have finished, set the state back to free.
bool HttpResponder::SendFileInfo()
{
	OutputBuffer *jsonResponse = nullptr;
	const bool gotFileInfo = reprap.GetPrintMonitor().GetFileInfoResponse(filenameBeingProcessed, jsonResponse);
	if (gotFileInfo)
	{
		// Got it - send the response now
		outBuf->copy(	"HTTP/1.1 200 OK\n"
						"Cache-Control: no-cache, no-store, must-revalidate\n"
						"Pragma: no-cache\n"
						"Expires: 0\n"
						"Access-Control-Allow-Origin: *\n"
						"Content-Type: application/json\n"
					);
		outBuf->catf("Content-Length: %u\n", (jsonResponse != nullptr) ? jsonResponse->Length() : 0);
		outBuf->cat("Connection: close\n\n");
		outBuf->Append(jsonResponse);
		Commit();
	}
	return gotFileInfo;
}

// Authenticate current IP and return true on success
bool HttpResponder::Authenticate()
{
	if (CheckAuthenticated())
	{
		return true;
	}

	if (numSessions < MaxHttpSessions)
	{
		sessions[numSessions].ip = GetRemoteIP();
		sessions[numSessions].lastQueryTime = millis();
		sessions[numSessions].isPostUploading = false;
		numSessions++;
		return true;
	}
	return false;
}

// Check and update the authentication
bool HttpResponder::CheckAuthenticated()
{
	const uint32_t remoteIP = GetRemoteIP();
	for (size_t i = 0; i < numSessions; i++)
	{
		if (sessions[i].ip == remoteIP)
		{
			sessions[i].lastQueryTime = millis();
			return true;
		}
	}
	return false;
}

bool HttpResponder::RemoveAuthentication()
{
	const uint32_t remoteIP = skt->GetRemoteIP();
	for (size_t i = numSessions; i != 0; )
	{
		--i;
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

void HttpResponder::SendFile(const char* nameOfFileToSend, bool isWebFile)
{
	FileStore *fileToSend = nullptr;
	bool zip = false;

	if (isWebFile)
	{
		if (nameOfFileToSend[0] == '/')
		{
			++nameOfFileToSend;						// all web files are relative to the /www folder, so remove the leading '/'
			if (nameOfFileToSend[0] == 0)
			{
				nameOfFileToSend = INDEX_PAGE_FILE;
			}
		}

		// Try to open a gzipped version of the file first
		if (!StringEndsWith(nameOfFileToSend, ".gz") && strlen(nameOfFileToSend) + 3 <= MaxFilenameLength)
		{
			String<MaxFilenameLength> nameBuf;
			nameBuf.copy(nameOfFileToSend);
			nameBuf.cat(".gz");
			fileToSend = GetPlatform().OpenFile(GetPlatform().GetWebDir(), nameBuf.c_str(), OpenMode::read);
			if (fileToSend != nullptr)
			{
				zip = true;
			}
		}

		// If that failed, try to open the normal version of the file
		if (fileToSend == nullptr)
		{
			fileToSend = GetPlatform().OpenFile(GetPlatform().GetWebDir(), nameOfFileToSend, OpenMode::read);
		}

		// If we still couldn't find the file and it was an HTML file, return the 404 error page
		if (fileToSend == nullptr && (StringEndsWith(nameOfFileToSend, ".html") || StringEndsWith(nameOfFileToSend, ".htm")))
		{
			nameOfFileToSend = FOUR04_PAGE_FILE;
			fileToSend = GetPlatform().OpenFile(GetPlatform().GetWebDir(), nameOfFileToSend, OpenMode::read);
		}

		if (fileToSend == nullptr)
		{
			RejectMessage("not found", 404);
			return;
		}
		fileBeingSent = fileToSend;
	}
	else
	{
		fileToSend = GetPlatform().OpenFile(FS_PREFIX, nameOfFileToSend, OpenMode::read);
		if (fileToSend == nullptr)
		{
			RejectMessage("not found", 404);
			return;
		}
		fileBeingSent = fileToSend;
	}

	outBuf->copy("HTTP/1.1 200 OK\n");

	// Don't cache files served by rr_download
	if (!isWebFile)
	{
		outBuf->cat(	"Cache-Control: no-cache, no-store, must-revalidate\n"
						"Pragma: no-cache\n"
						"Expires: 0\n"
						"Access-Control-Allow-Origin: *\n"
					);
	}

	const char* contentType;
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
	else if (StringEndsWith(nameOfFileToSend, ".g") || StringEndsWith(nameOfFileToSend, ".gc") || StringEndsWith(nameOfFileToSend, ".gcode"))
	{
		contentType = "text/plain";
	}
	else
	{
		contentType = "application/octet-stream";
	}
	outBuf->catf("Content-Type: %s\n", contentType);

	if (zip && fileToSend != nullptr)
	{
		outBuf->cat("Content-Encoding: gzip\n");
		outBuf->catf("Content-Length: %lu\n", fileToSend->Length());
	}

	outBuf->cat("Connection: close\n\n");
	Commit();
}

void HttpResponder::SendGCodeReply()
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
			GetPlatform().MessageF(UsbMessage, "Sending G-Code reply to HTTP client %d of %d (length %u)\n", clientsServed, numSessions, gcodeReply->DataLength());
		}
	}

	// Send the whole G-Code reply as plain text to the client
	outBuf->copy(	"HTTP/1.1 200 OK\n"
					"Cache-Control: no-cache, no-store, must-revalidate\n"
					"Pragma: no-cache\n"
					"Expires: 0\n"
					"Access-Control-Allow-Origin: *\n"
					"Content-Type: text/plain\n"
				);
	outBuf->catf("Content-Length: %u\n", gcodeReply->DataLength());
	outBuf->cat("Connection: close\n\n");
	outStack->Append(gcodeReply);
	Commit();

	// Possibly clean up the G-code reply once again
	if (clearReply)
	{
		gcodeReply->Clear();
	}
}

void HttpResponder::SendJsonResponse(const char* command)
{
	// Try to authorise the user automatically to retain compatibility with the old web interface
	if (!CheckAuthenticated() && reprap.NoPasswordSet())
	{
		Authenticate();
	}

	// Update the authentication status and try handle "text/plain" requests here
	if (CheckAuthenticated())
	{
		if (StringEquals(command, "reply"))			// rr_reply
		{
			SendGCodeReply();
			return;
		}

		if (StringEquals(command, "configfile"))	// rr_configfile [DEPRECATED]
		{
			const char *configPath = GetPlatform().GetMassStorage()->CombineName(GetPlatform().GetSysDir(), GetPlatform().GetConfigFile());
			char fileName[MaxFilenameLength];
			SafeStrncpy(fileName, configPath, ARRAY_SIZE(fileName));
			SendFile(fileName, false);
			return;
		}

		if (StringEquals(command, "download"))
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
		// Reset the connection immediately if we cannot write any data. Should never happen.
		skt->Terminate();
		return;
	}

	bool mayKeepOpen;
	const bool gotResponse = GetJsonResponse(command, jsonResponse, mayKeepOpen);
	if (!gotResponse)
	{
		// Either this request was rejected, or it will take longer to process e.g. rr_fileinfo
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
			if (StringEquals(headers[i].key, "Connection"))
			{
				// Comment out the following line to disable persistent connections
				keepOpen = StringEquals(headers[i].value, "keep-alive");
				break;
			}
		}
	}

	outBuf->copy(	"HTTP/1.1 200 OK\n"
					"Cache-Control: no-cache, no-store, must-revalidate\n"
					"Pragma: no-cache\n"
					"Expires: 0\n"
					"Access-Control-Allow-Origin: *\n"
					"Content-Type: application/json\n"
				);
	outBuf->catf("Content-Length: %u\n", (jsonResponse != nullptr) ? jsonResponse->Length() : 0);
	outBuf->catf("Connection: %s\n\n", keepOpen ? "keep-alive" : "close");
	outBuf->Append(jsonResponse);

	Commit(keepOpen ? ResponderState::reading : ResponderState::free);
}

// Process the message received so far. We have reached the end of the headers.
// Return true if the message is complete, false if we want to continue receiving data (i.e. postdata)
void HttpResponder::ProcessMessage()
{
	if (reprap.Debug(moduleWebserver))
	{
		GetPlatform().MessageF(UsbMessage, "HTTP req, command words {");
		for (size_t i = 0; i < numCommandWords; ++i)
		{
			GetPlatform().MessageF(UsbMessage, " %s", commandWords[i]);
		}
		GetPlatform().Message(UsbMessage, " }, parameters {");

		for (size_t i = 0; i < numQualKeys; ++i)
		{
			GetPlatform().MessageF(UsbMessage, " %s=%s", qualifiers[i].key, qualifiers[i].value);
		}
		GetPlatform().Message(UsbMessage, " }\n");
	}

	if (numCommandWords < 2)
	{
		RejectMessage("too few command words");
		return;
	}

	if (StringEquals(commandWords[0], "GET"))
	{
		if (StringStartsWith(commandWords[1], KO_START))
		{
			SendJsonResponse(commandWords[1] + KoFirst);
		}
		else if (commandWords[1][0] == '/' && StringStartsWith(commandWords[1] + 1, KO_START))
		{
			SendJsonResponse(commandWords[1] + 1 + KoFirst);
		}
		else
		{
			SendFile(commandWords[1], true);
		}
		return;
	}

	if (StringEquals(commandWords[0], "OPTIONS"))
	{
		outBuf->copy(	"HTTP/1.1 200 OK\n"
						"Allow: OPTIONS, GET, POST\n"
						"Cache-Control: no-cache, no-store, must-revalidate\n"
						"Pragma: no-cache\n"
						"Expires: 0\n"
						"Access-Control-Allow-Origin: *\n"
						"Access-Control-Allow-Headers: Content-Type\n"
						"Content-Length: 0\n"
						"\n"
					);
		Commit();
		return;
	}

	if (CheckAuthenticated() && StringEquals(commandWords[0], "POST"))
	{
		const bool isUploadRequest = (StringEquals(commandWords[1], KO_START "upload"))
								  || (commandWords[1][0] == '/' && StringEquals(commandWords[1] + 1, KO_START "upload"));
		if (isUploadRequest)
		{
			const char* const filename = GetKeyValue("name");
			if (filename != nullptr)
			{
				// See how many bytes we expect to read
				bool contentLengthFound = false;
				for (size_t i = 0; i < numHeaderKeys; i++)
				{
					if (StringEquals(headers[i].key, "Content-Length"))
					{
						postFileLength = atoi(headers[i].value);
						contentLengthFound = true;
						break;
					}
				}

				// Start POST file upload
				if (!contentLengthFound)
				{
					RejectMessage("invalid POST upload request");
					return;
				}

				// Start a new file upload
				FileStore *file = GetPlatform().OpenFile(FS_PREFIX, filename, OpenMode::write);
				if (file == nullptr)
				{
					RejectMessage("could not create file");
					return;

				}
				StartUpload(file, filename);

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
					GetPlatform().MessageF(UsbMessage, "Start uploading file %s length %lu\n", filename, postFileLength);
				}
				uploadedBytes = 0;

				// Keep track of the connection that is now uploading
				const uint32_t remoteIP = GetRemoteIP();
				const uint16_t remotePort = skt->GetRemotePort();
				for(size_t i = 0; i < numSessions; i++)
				{
					if (sessions[i].ip == remoteIP)
					{
						sessions[i].postPort = remotePort;
						sessions[i].isPostUploading = true;
						break;
					}
				}
				return;
			}
		}
		RejectMessage("only rr_upload is supported for POST requests");
	}
	else
	{
		RejectMessage("Unknown message type or not authenticated");
	}
}

// Reject the current message
void HttpResponder::RejectMessage(const char* response, unsigned int code)
{
	if (reprap.Debug(moduleWebserver))
	{
		GetPlatform().MessageF(UsbMessage, "Webserver: rejecting message with: %u %s\n", code, response);
	}
	outBuf->printf("HTTP/1.1 %u %s\nConnection: close\n\n", code, response);
	Commit();
}

// This function overrides the one in class NetworkResponder.
// It tries to process a chunk of uploaded data and changes the state if finished.
void HttpResponder::DoUpload()
{
	const uint8_t *buffer;
	size_t len;
	if (skt->ReadBuffer(buffer, len))
	{
		skt->Taken(len);
		uploadedBytes += len;

		if (!fileBeingUploaded.Write(buffer, len))
		{
			uploadError = true;
			GetPlatform().Message(ErrorMessage, "Could not write upload data!\n");
			CancelUpload();
			SendJsonResponse("upload");
			return;
		}
	}

	// See if the upload has finished
	if (uploadedBytes >= postFileLength)
	{
		// Reset POST upload state for this client
		const uint32_t remoteIP = GetRemoteIP();
		for (size_t i = 0; i < numSessions; i++)
		{
			if (sessions[i].ip == remoteIP && sessions[i].isPostUploading)
			{
				sessions[i].isPostUploading = false;
				sessions[i].lastQueryTime = millis();
				break;
			}
		}

		FinishUpload(postFileLength, fileLastModified);
		SendJsonResponse("upload");
		return;
	}
	else if (!skt->CanRead())
	{
		// We cannot read any more, discard the transaction
		ConnectionLost();
	}
}

// This is called to force termination if we implement the specified protocol
void HttpResponder::Terminate(NetworkProtocol protocol)
{
	if (responderState != ResponderState::free && (protocol == HttpProtocol || protocol == AnyProtocol))
	{
		ConnectionLost();
	}
}

// This overrides the version in class NetworkResponder
void HttpResponder::ConnectionLost()
{
	fileInfoLock.Release(this);
	NetworkResponder::ConnectionLost();
}

// This overrides the version in class NetworkResponder
void HttpResponder::CancelUpload()
{
	if (skt != nullptr)
	{
		for (size_t i = 0; i < numSessions; i++)
		{
			if (sessions[i].ip == skt->GetRemoteIP() && sessions[i].isPostUploading)
			{
				sessions[i].isPostUploading = false;
				sessions[i].lastQueryTime = millis();
				break;
			}
		}
	}
	NetworkResponder::CancelUpload();
}

// This overrides the version in class NetworkResponder
void HttpResponder::SendData()
{
	NetworkResponder::SendData();
	if (responderState == ResponderState::reading)
	{
		timer = millis();				// restart the timer
	}
}

void HttpResponder::Diagnostics(MessageType mt) const
{
	GetPlatform().MessageF(mt, " HTTP(%d)", (int)responderState);
}

/*static*/ void HttpResponder::HandleGCodeReply(const char *reply)
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

/*static*/ void HttpResponder::HandleGCodeReply(OutputBuffer *reply)
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


/*static*/ void HttpResponder::CheckSessions()
{
	const uint32_t now = millis();
	for (size_t i = numSessions; i != 0; )
	{
		--i;
		if ((now - sessions[i].lastQueryTime) > HttpSessionTimeout)
		{
			// Check for timed out sessions
			for (size_t k = i + 1; k < numSessions; k++)
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
			OutputBuffer::ReleaseAll(gcodeReply->Pop());
		}
		clientsServed = 0;
	}
}

/*static*/ void HttpResponder::CommonDiagnostics(MessageType mtype)
{
	GetPlatform().MessageF(mtype, "HTTP sessions: %u of %u\n", numSessions, MaxHttpSessions);
}

// Static data

HttpResponder::HttpSession HttpResponder::sessions[MaxHttpSessions];
unsigned int HttpResponder::numSessions = 0;
unsigned int HttpResponder::clientsServed = 0;

uint32_t HttpResponder::seq = 0;
OutputStack *HttpResponder::gcodeReply = new OutputStack();

NetworkResponderLock HttpResponder::fileInfoLock;

// End
