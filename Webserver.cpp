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
		gcodeReadIndex = (gcodeReadIndex + 1u) % gcodeBufLength;
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
					platform->Message(HOST_MESSAGE, "Webserver: GCode local buffer overflow.\n");
					HandleReply("Webserver: GCode local buffer overflow", true);
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
		HandleReply("Webserver: GCode buffer overflow", true);
	}
	else
	{
		size_t remaining = gcodeBufLength - gcodeWriteIndex;
		if (len <= remaining)
		{
			memcpy(gcodeBuffer + gcodeWriteIndex, data, len);
		}
		else
		{
			memcpy(gcodeBuffer + gcodeWriteIndex, data, remaining);
			memcpy(gcodeBuffer, data + remaining, len - remaining);
		}
		gcodeWriteIndex = (gcodeWriteIndex + len) % gcodeBufLength;
	}
}

// Process a received string of upload data
void Webserver::StoreUploadData(const char* data, size_t len)
{
	if (len > GetUploadBufferSpace())
	{
		platform->Message(HOST_MESSAGE, "Webserver: Upload buffer overflow.\n");
		HandleReply("Webserver: Upload buffer overflow", true);
	}
	else
	{
		size_t remaining = uploadBufLength - uploadWriteIndex;
		if (len <= remaining)
		{
			memcpy(uploadBuffer + uploadWriteIndex, data, len);
		}
		else
		{
			memcpy(uploadBuffer + uploadWriteIndex, data, remaining);
			memcpy(uploadBuffer, data + remaining, len - remaining);
		}
		uploadWriteIndex = (uploadWriteIndex + len) % uploadBufLength;
	}
}

// Process a null-terminated gcode
// We intercept four G/M Codes so we can deal with file manipulation and emergencies.  That
// way things don't get out of sync, and - as a file name can contain
// a valid G code (!) - confusion is avoided.

void Webserver::ProcessGcode(const char* gc)
{
	int8_t specialAction = 0;
	if (StringStartsWith(gc, "M30 "))		// delete SD card file
	{
		specialAction = 1;
	}
	else if (StringStartsWith(gc, "M23 "))	// select SD card file to print next
	{
		specialAction = 2;
	}
	else if (StringStartsWith(gc, "M112") && !isdigit(gc[4]))	// emergency stop
	{
		specialAction = 3;
	}
	else if (StringStartsWith(gc, "M503") && !isdigit(gc[4]))	// echo config.g file
	{
		specialAction = 4;
	}

	switch (specialAction)
	{
	case 1: // Delete
		reprap.GetGCodes()->DeleteFile(&gc[4]);
		break;

	case 2:	// print
		reprap.GetGCodes()->QueueFileToPrint(&gc[4]);
		break;

	case 3:
		reprap.EmergencyStop();
		gcodeReadIndex = gcodeWriteIndex;		// clear the buffer
		reprap.GetGCodes()->Reset();
		break;

	case 4:
		{
			FileStore *configFile = platform->GetFileStore(platform->GetSysDir(), platform->GetConfigFile(), false);
			if (configFile == NULL)
			{
				HandleReply("Configuration file not found", true);
			}
			else
			{
				char c;
				size_t i = 0;
				while (i < ARRAY_UPB(gcodeReply) && configFile->Read(c))
				{
					gcodeReply[i++] = c;
				}
				configFile->Close();
				gcodeReply[i] = 0;
				++seq;
			}
		}
		break;

	default:
		StoreGcodeData(gc, strlen(gc) + 1);
		break;
	}
}

//********************************************************************************************

// Communications with the client

//--------------------------------------------------------------------------------------------

// Output to the client

// Start sending a file or a JSON response.
void Webserver::SendFile(const char* nameOfFileToSend)
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
	RequestState *req = net->GetRequest();
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

void Webserver::SendJsonResponse(const char* command)
{
	Network *net = reprap.GetNetwork();
	RequestState *req = net->GetRequest();
	GetJsonResponse(command, ((numQualKeys == 0) ? "" : qualifiers[0].key), ((numQualKeys == 0) ? "" : qualifiers[0].value));
	req->Write("HTTP/1.1 200 OK\n");
	req->Write("Content-Type: application/json\n");
	req->Printf("Content-Length: %u\n", strlen(jsonResponse));
	req->Write("Connection: close\n\n");
	req->Write(jsonResponse);
	net->SendAndClose(NULL);
}

//----------------------------------------------------------------------------------------------------

// Input from the client

void Webserver::CheckPassword(const char *pw)
{
	gotPassword = StringEquals(pw, password);
}

void Webserver::JsonReport(bool ok, const char* request)
{
	if (ok)
	{
		jsonResponse[ARRAY_UPB(jsonResponse)] = 0;
		if (reprap.Debug())
		{
			platform->Message(HOST_MESSAGE, "JSON response: ");
			platform->Message(HOST_MESSAGE, jsonResponse);
			platform->Message(HOST_MESSAGE, " queued\n");
		}
	}
	else
	{
		jsonResponse[0] = 0;
		platform->Message(HOST_MESSAGE, "KnockOut request: ");
		platform->Message(HOST_MESSAGE, request);
		platform->Message(HOST_MESSAGE, " not recognised\n");
	}
}

void Webserver::GetJsonResponse(const char* request, const char* key, const char* value)
{
	bool found = true;	// assume success

	if (StringEquals(request, "status"))	// new style status request
	{
		GetStatusResponse(1);
	}
	else if (StringEquals(request, "poll"))		// old style status request
	{
		GetStatusResponse(0);
	}
	else if (StringEquals(request, "gcode") && StringEquals(key, "gcode"))
	{
		LoadGcodeBuffer(value);
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"buff\":%u}", GetReportedGcodeBufferSpace());
	}
	else if (StringEquals(request, "upload_begin") && StringEquals(key, "name"))
	{
		CancelUpload();
		FileStore *f = platform->GetFileStore("0:/", value, true);
		if (f != NULL)
		{
			fileBeingUploaded.Set(f);
			uploadState = uploadOK;
		}
		else
		{
			uploadState = uploadError;
		}
		GetJsonUploadResponse();
	}
	else if (StringEquals(request, "upload_data") && StringEquals(key, "data"))
	{
		if (uploadState == uploadOK)
		{
			StoreUploadData(value, strlen(value));
		}
		GetJsonUploadResponse();
	}
	else if (StringEquals(request, "upload_end"))
	{
		// Write the remaining data
		while (uploadState == uploadOK && uploadReadIndex != uploadWriteIndex)
		{
			char c = uploadBuffer[uploadReadIndex];
			uploadReadIndex = (uploadReadIndex + 1) % uploadBufLength;
			if (!fileBeingUploaded.Write(c))
			{
				uploadState = uploadError;
			}
		}

		// Close the file
		if (!fileBeingUploaded.Close())
		{
			uploadState = uploadError;
		}
		GetJsonUploadResponse();
	}
	else if (StringEquals(request, "upload_cancel"))
	{
		CancelUpload();
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"err\":%d}", 0);
	}
	else if (StringEquals(request, "delete") && StringEquals(key, "name"))
	{
		bool ok = platform->GetMassStorage()->Delete("0:/", value);
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"err\":%d}", (ok) ? 0 : 1);
	}
	else if (StringEquals(request, "files"))
	{
		const char* dir = (StringEquals(key, "dir")) ? value : platform->GetGCodeDir();
		const char* fileList = platform->GetMassStorage()->FileList(dir, false);
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"files\":[%s]}", fileList);
	}
	else if (StringEquals(request, "fileinfo") && StringEquals(key, "name"))
	{
		unsigned long length;
		float height, filament;
		bool found = GetFileInfo(value, length, height, filament);
		if (found)
		{
			snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"size\":%lu,\"height\":\"%.2f\",\"filament\":\"%.1f\"}", length, height, filament);
		}
		else
		{
			snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{}");
		}
	}
	else if (StringEquals(request, "name"))
	{
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"myName\":\"%s\"}", myName);
	}
	else if (StringEquals(request, "password") && StringEquals(key, "password"))
	{
		CheckPassword(value);
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"password\":\"%s\"}", (gotPassword) ? "right" : "wrong");
	}
	else if (StringEquals(request, "axes"))
	{
		strncpy(jsonResponse, "{\"axes\":", ARRAY_UPB(jsonResponse));
		char ch = '[';
		for (int8_t drive = 0; drive < AXES; drive++)
		{
			sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "%c\"%.1f\"", ch, platform->AxisTotalLength(drive));
			ch = ',';
		}
		strncat(jsonResponse, "]}", ARRAY_UPB(jsonResponse));
	}
	else if (StringEquals(request, "connect"))
	{
		CancelUpload();
		GetStatusResponse(1);
	}
	else
	{
		found = false;
	}

	JsonReport(found, request);
}

void Webserver::GetJsonUploadResponse()
{
	snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"ubuff\":%u,\"err\":%d}", GetReportedUploadBufferSpace(), (uploadState == uploadOK) ? 0 : 1);
}

void Webserver::GetStatusResponse(uint8_t type)
{
	GCodes *gc = reprap.GetGCodes();
	if (type == 1)
	{
		// New-style status request
		// Send the printing/idle status
		char ch = (reprap.IsStopped()) ? 'S' : (gc->PrintingAFile()) ? 'P' : 'I';
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"status\":\"%c\",\"heaters\":", ch);

		// Send the heater temperatures
		ch = '[';
		for (int8_t heater = 0; heater < HEATERS; heater++)
		{
			sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "%c\"%.1f\"", ch, reprap.GetHeat()->GetTemperature(heater));
			ch = ',';
		}

		// Send XYZ and extruder positions
		float liveCoordinates[DRIVES + 1];
		reprap.GetMove()->LiveCoordinates(liveCoordinates);
		strncat(jsonResponse, "],\"pos\":", ARRAY_UPB(jsonResponse));		// announce the XYZ position
		ch = '[';
		// We currently provide the extruder 0 value here as well as XYZ. This is only expected by V0.69 and V0.70 of the web interface so it can be removed soon.
		for (int8_t drive = 0; drive < AXES + 1; drive++)
		//for (int8_t drive = 0; drive < AXES; drive++)
		{
			sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "%c\"%.2f\"", ch, liveCoordinates[drive]);
			ch = ',';
		}
		sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "],\"extr\":");		// announce the extruder positions
		ch = '[';
		for (int8_t drive = AXES; drive < DRIVES; drive++)		// loop through extruders
		{
			sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "%c\"%.3f\"", ch, gc->GetExtruderPosition(drive - AXES));
			ch = ',';
		}
		strncat(jsonResponse, "]", ARRAY_UPB(jsonResponse));
	}
	else
	{
		// The old (deprecated) poll response lists the status, then all the heater temperatures, then the XYZ positions, then all the extruder positions.
		// These are all returned in a single vector called "poll".
		// This is a poor choice of format because we can't easily tell which is which unless we already know the number of heaters and extruders.
		char c = (gc->PrintingAFile()) ? 'P' : 'I';
		snprintf(jsonResponse, ARRAY_UPB(jsonResponse), "{\"poll\":[\"%c\",", c); // Printing
		for (int8_t heater = 0; heater < HEATERS; heater++)
		{
			sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "\"%.1f\",", reprap.GetHeat()->GetTemperature(heater));
		}
		// Send XYZ and extruder positions
		float liveCoordinates[DRIVES + 1];
		reprap.GetMove()->LiveCoordinates(liveCoordinates);
		for (int8_t drive = 0; drive < DRIVES; drive++)	// loop through extruders
		{
			char ch = (drive == DRIVES - 1) ? ']' : ',';	// append ] to the last one but , to the others
			sncatf(jsonResponse, ARRAY_UPB(jsonResponse), "\"%.2f\"%c", liveCoordinates[drive], ch);
		}
	}

	// Send the Z probe value
	int v0 = platform->ZProbe();
	int v1, v2;
	switch (platform->GetZProbeSecondaryValues(v1, v2))
	{
	case 1:
		sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"probe\":\"%d (%d)\"", v0, v1);
		break;
	case 2:
		sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"probe\":\"%d (%d, %d)\"", v0, v1, v2);
		break;
	default:
		sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"probe\":\"%d\"", v0);
		break;
	}

	// Send the amount of buffer space available for gcodes
	sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"buff\":%u", GetReportedGcodeBufferSpace());

	// Send the home state. To keep the messages short, we send 1 for homed and 0 for not homed, instead of true and false.
	if (type != 0)
	{
		sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"homed\":[%d,%d,%d]",
				(gc->GetAxisIsHomed(0)) ? 1 : 0,
				(gc->GetAxisIsHomed(1)) ? 1 : 0,
				(gc->GetAxisIsHomed(2)) ? 1 : 0);
	}
	else
	{
		sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"hx\":%d,\"hy\":%d,\"hz\":%d",
				(gc->GetAxisIsHomed(0)) ? 1 : 0,
				(gc->GetAxisIsHomed(1)) ? 1 : 0,
				(gc->GetAxisIsHomed(2)) ? 1 : 0);
	}

	// Send the response sequence number
	sncatf(jsonResponse, ARRAY_UPB(jsonResponse), ",\"seq\":%u", (unsigned int) seq);

	// Send the response to the last command. Do this last because it is long and may need to be truncated.
	strncat(jsonResponse, ",\"resp\":\"", ARRAY_UPB(jsonResponse));
	size_t jp = strnlen(jsonResponse, ARRAY_UPB(jsonResponse));
	const char *p = gcodeReply;
	while (*p != 0 && jp < ARRAY_SIZE(jsonResponse) - 3)	// leave room for the final '"}\0'
	{
		char c = *p++;
		char esc;
		switch (c)
		{
		case '\r':
			esc = 'r';
			break;
		case '\n':
			esc = 'n';
			break;
		case '\t':
			esc = 't';
			break;
		case '"':
			esc = '"';
			break;
		case '\\':
			esc = '\\';
			break;
		default:
			esc = 0;
			break;
		}
		if (esc)
		{
			if (jp == ARRAY_SIZE(jsonResponse) - 4)
			{
				break;
			}
			jsonResponse[jp++] = '\\';
			jsonResponse[jp++] = esc;
		}
		else
		{
			jsonResponse[jp++] = c;
		}
	}
	jsonResponse[jp] = 0;
	strncat(jsonResponse, "\"}", ARRAY_UPB(jsonResponse));
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
bool Webserver::CharFromClient(char c)
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
			numHeaderKeys = 0;
			headers[0].key = clientMessage + clientPointer;
			state = doingHeaderKey;
			break;
		case ' ':
		case '\t':
			clientMessage[clientPointer++] = 0;
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
			if (numQualKeys < maxQualKeys)
			{
				qualifiers[numQualKeys].value = clientMessage + clientPointer;
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
			state = (ServerState)(state + 1);
		}
		else if (c >= 'A' && c <= 'F')
		{
			decodeChar = (c - ('A' - 10)) << 4;
			state = (ServerState)(state + 1);
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
			state = (ServerState)(state - 2);
		}
		else if (c >= 'A' && c <= 'F')
		{
			clientMessage[clientPointer++] = decodeChar | c - ('A' - 10);
			state = (ServerState)(state - 2);
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
			if (clientMessage + clientPointer == headers[numHeaderKeys].key)
			{
				return ProcessMessage();
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
			return ProcessMessage();
		case '\r':
			break;
		default:
			if (clientPointer + 3 <= ARRAY_SIZE(clientMessage))
			{
				clientMessage[clientPointer++] = 0;
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
bool Webserver::ProcessMessage()
{
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
bool Webserver::RejectMessage(const char* response, unsigned int code)
{
	platform->Message(HOST_MESSAGE, "Webserver: rejecting message with: ");
	platform->Message(HOST_MESSAGE, response);
	platform->Message(HOST_MESSAGE, "\n");

	Network *net = reprap.GetNetwork();
	RequestState *req = net->GetRequest();
	req->Printf("HTTP/1.1 %u %s\nConnection: close\n\n", code, response);
	net->SendAndClose(NULL);
	return true;
}

// Deal with input/output from/to the client (if any)

void Webserver::Spin()
{
	if (state != inactive)
	{
		if (uploadState == uploadOK && uploadReadIndex != uploadWriteIndex)
		{
			// Write some uploaded data to file
			for (unsigned int i = 0; i < 256 && uploadReadIndex != uploadWriteIndex && uploadState == uploadOK; ++i)
			{
				char c = uploadBuffer[uploadReadIndex];
				uploadReadIndex = (uploadReadIndex + 1) % uploadBufLength;
				if (!fileBeingUploaded.Write(c))
				{
					uploadState = uploadError;
				}
			}
		}
		else
		{
			Network *net = reprap.GetNetwork();
			RequestState *req = net->GetRequest();
			if (req != NULL)
			{
				// To achieve a high upload speed, we try to process a complete request here
				for (;;)
				{
					char c;
					if (req->Read(c))
					{
						if (CharFromClient(c))
						{
							ResetState();
							break;	// break if we have read all we want of this message
						}
					}
					else
					{
						// We ran out of data before finding a complete request.
						// This can happen if the network connection was lost, so only report it if debug is enabled
						if (reprap.Debug())
						{
							platform->Message(HOST_MESSAGE, "Webserver: incomplete request\n");
						}
						net->SendAndClose(NULL);
						ResetState();
						break;
					}
				}
			}
		}

		platform->ClassReport("Webserver", longWait);
	}
}

//******************************************************************************************

// Constructor and initialisation

Webserver::Webserver(Platform* p)
{
	platform = p;
	state = inactive;
	gotPassword = false;
}

void Webserver::Init()
{
	SetPassword(DEFAULT_PASSWORD);
	SetName(DEFAULT_NAME);
	gcodeReadIndex = gcodeWriteIndex = 0;
	uploadReadIndex = uploadWriteIndex = 0;
	lastTime = platform->Time();
	longWait = lastTime;
	gcodeReply[0] = 0;
	seq = 0;
	uploadState = notUploading;
	ResetState();

	// Reinitialise the message file

	//platform->GetMassStorage()->Delete(platform->GetWebDir(), MESSAGE_FILE);
}

void Webserver::ResetState()
{
	clientPointer = 0;
	state = doingCommandWord;
	numCommandWords = 0;
	numQualKeys = 0;
	numHeaderKeys = 0;
	commandWords[0] = clientMessage;
}

void Webserver::CancelUpload()
{
	fileBeingUploaded.Close();		// cancel any pending file upload (TODO: delete the partially-written file)
	uploadReadIndex = uploadWriteIndex = 0;
	uploadState = notUploading;
}

void Webserver::Exit()
{
	CancelUpload();
	platform->Message(HOST_MESSAGE, "Webserver class exited.\n");
	state = inactive;
}

void Webserver::Diagnostics()
{
	platform->Message(HOST_MESSAGE, "Webserver Diagnostics:\n");
}

void Webserver::SetPassword(const char* pw)
{
	strncpy(password, pw, SHORT_STRING_LENGTH);
	password[SHORT_STRING_LENGTH] = 0; // NB array is dimensioned to SHORT_STRING_LENGTH+1
}

void Webserver::SetName(const char* nm)
{
	strncpy(myName, nm, SHORT_STRING_LENGTH);
	myName[SHORT_STRING_LENGTH] = 0; // NB array is dimensioned to SHORT_STRING_LENGTH+1
}

void Webserver::HandleReply(const char *s, bool error)
{
	if (strlen(s) == 0 && !error)
	{
		strcpy(gcodeReply, "ok");
	}
	else
	{
		if (error)
		{
			strcpy(gcodeReply, "Error: ");
			strncat(gcodeReply, s, ARRAY_UPB(gcodeReply));
		}
		else
		{
			strncpy(gcodeReply, s, ARRAY_UPB(gcodeReply));
		}
		gcodeReply[ARRAY_UPB(gcodeReply)] = 0;
	}
	++seq;
}

void Webserver::AppendReply(const char *s)
{
	strncat(gcodeReply, s, ARRAY_UPB(gcodeReply));
}

// Get the actual amount of gcode buffer space we have
unsigned int Webserver::GetGcodeBufferSpace() const
{
	return (gcodeReadIndex - gcodeWriteIndex - 1u) % gcodeBufLength;
}

// Get the amount of gcode buffer space we are going to report
unsigned int Webserver::GetReportedGcodeBufferSpace() const
{
	unsigned int temp = GetGcodeBufferSpace();
	return (temp > maxReportedFreeBuf) ? maxReportedFreeBuf : temp;
}

// Get the actual amount of gcode buffer space we have
unsigned int Webserver::GetUploadBufferSpace() const
{
	return (uploadReadIndex - uploadWriteIndex - 1u) % uploadBufLength;
}

// Get the amount of gcode buffer space we are going to report
unsigned int Webserver::GetReportedUploadBufferSpace() const
{
	unsigned int temp = GetUploadBufferSpace();
	return (temp > maxReportedFreeBuf) ? maxReportedFreeBuf : temp;
}

// Get information for a file on the SD card
bool Webserver::GetFileInfo(const char *fileName, unsigned long& length, float& height, float& filamentUsed)
{
	FileStore *f = platform->GetFileStore(platform->GetGCodeDir(), fileName, false);
	if (f != NULL)
	{
		// Try to find the object height by looking for the last G1 Zxxx command in the file
		length = f->Length();
		height = 0.0;
		filamentUsed = 0.0;
		if (length != 0)
		{
			const size_t readSize = 512;					// read 512 bytes at a time (tried 1K but it sometimes gives us the wrong data)
			const size_t overlap = 100;
			size_t sizeToRead;
			if (length <= sizeToRead + overlap)
			{
				sizeToRead = length;						// read the whole file in one go
			}
			else
			{
				sizeToRead = length % readSize;
				if (sizeToRead <= overlap)
				{
					sizeToRead += readSize;
				}
			}
			unsigned long seekPos = length - sizeToRead;	// read on a 1K boundary
			size_t sizeToScan = sizeToRead;
			char buf[readSize + overlap + 1];				// need the +1 so we can add a null terminator
			bool foundFilamentUsed = false;
			for (;;)
			{
				if (!f->Seek(seekPos))
				{
//debugPrintf("Seek to %lu failed\n", seekPos);
					break;
				}
//debugPrintf("Reading %u bytes at %lu\n", sizeToRead, seekPos);
				int nbytes = f->Read(buf, sizeToRead);
				if (nbytes != (int)sizeToRead)
				{
//debugPrintf("Read failed, read %d bytes\n", nbytes);
					break;									// read failed so give up
				}
				buf[sizeToScan] = 0;						// add a null terminator
//debugPrintf("About to scan %u bytes starting: %.40s\n", sizeToScan, buf);
				if (!foundFilamentUsed)
				{
					foundFilamentUsed = FindFilamentUsed(buf, sizeToScan, filamentUsed);
//debugPrintf("Found fil: %c\n", foundFilamentUsed ? 'Y' : 'N');
				}
				if (FindHeight(buf, sizeToScan, height))
				{
//debugPrintf("Found height = %f\n", height);
					break;		// quit if found height
				}
				if (seekPos == 0 || length - seekPos >= 200000uL)	// scan up to about the last 200K of the file (32K wasn't enough)
				{
//debugPrintf("Quitting loop at seekPos = %lu\n", seekPos);
					break;		// quit if reached start of file or already scanned the last 32K of the file
				}
				seekPos -= readSize;
				sizeToRead = readSize;
				sizeToScan = readSize + overlap;
				memcpy(buf + sizeToRead, buf, overlap);
			}
		}
		f->Close();
//debugPrintf("Set height %f and filament %f\n", height, filamentUsed);
		return true;
	}
	return false;
}

// Scan the buffer for a G1 Zxxx command. The buffer is null-terminated.
bool Webserver::FindHeight(const char* buf, size_t len, float& height)
{
//debugPrintf("Scanning %u bytes starting %.100s\n", len, buf);
	if (len >= 5)
	{
		size_t i = len - 5;
		for(;;)
		{
			if (buf[i] == 'G' && buf[i + 1] == '1' && buf[i + 2] == ' ' && buf[i + 3] == 'Z' && isDigit(buf[i + 4]))
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

// Scan the buffer for the filament used. The buffer is null-terminated.
bool Webserver::FindFilamentUsed(const char* buf, size_t len, float& filamentUsed)
{
	const char* filamentUsedStr = "ilament used";		// comment string used by slic3r, followed by filament used and "mm"
	const char* p = strstr(buf, filamentUsedStr);
	if (p != NULL)
	{
		p += strlen(filamentUsedStr);
		while(strchr(" :=\t", *p) != NULL)
		{
			++p;	// this allows for " = " from default slic3r comment and ": " from default Cura comment
		}
		if (isDigit(*p))
		{
			char* q;
			filamentUsed = strtod(p, &q);
			if (*q == 'm' && *(q + 1) != 'm')
			{
				filamentUsed *= 1000.0;		// Cura outputs filament used in metres not mm
			}
			return true;
		}
	}
	return false;
}

// End
