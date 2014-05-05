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

 ****************************************************************************************************/

#include "RepRapFirmware.h"

//***************************************************************************************************

bool Webserver::MatchBoundary(char c)
{
	if (!postBoundary[0])
		return false;

	if (c == postBoundary[boundaryCount])
	{
		boundaryCount++;
		if (!postBoundary[boundaryCount])
		{
			boundaryCount = 0;
			return true;
		}
	}
	else
	{
		for (int i = 0; i < boundaryCount; i++)
		{
			postFile->Write(postBoundary[i]);
		}
		postFile->Write(c);
		boundaryCount = 0;
	}
	return false;
}

//****************************************************************************************************

// Feeding G Codes to the GCodes class

bool Webserver::GCodeAvailable()
{
	return gcodeReadIndex != gcodeWriteIndex;
}

byte Webserver::ReadGCode()
{
	byte c;
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
void Webserver::LoadGcodeBuffer(const char* gc, bool convertWeb)
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

		if (c == '+' && convertWeb)
		{
			c = ' ';
		}
		else if (c == '%' && convertWeb)
		{
			c = *gc++;
			if (c != 0)
			{
				unsigned char uc;
				if (isalpha(c))
				{
					uc = 16 * (c - 'A' + 10);
				}
				else
				{
					uc = 16 * (c - '0');
				}
				c = *gc++;
				if (c != 0)
				{
					if (isalpha(c))
					{
						uc += c - 'A' + 10;
					}
					else
					{
						uc += c - '0';
					}
					c = uc;
				}
			}
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

			if (gtp == ARRAY_SIZE(gcodeTempBuf) - 1)
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
				while (i < STRING_LENGTH && configFile->Read(c))
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
		{
			// Copy the gcode to the buffer
			size_t len = strlen(gc) + 1;		// number of characters to copy
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
					memcpy(&gcodeBuffer[gcodeWriteIndex], gc, len);
				}
				else
				{
					memcpy(&gcodeBuffer[gcodeWriteIndex], gc, remaining);
					memcpy(gcodeBuffer, gc + remaining, len - remaining);
				}
				gcodeWriteIndex = (gcodeWriteIndex + len) % gcodeBufLength;
			}
		}
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
	bool doingJsonResponse = StringStartsWith(nameOfFileToSend, KO_START);
	FileStore *fileToSend;

	if (doingJsonResponse)
	{
		fileToSend = NULL;
		GetJsonResponse(&nameOfFileToSend[KO_FIRST]);
	}
	else
	{
		fileToSend = platform->GetFileStore(platform->GetWebDir(), nameOfFileToSend, false);
		if (fileToSend == NULL)
		{
			nameOfFileToSend = FOUR04_FILE;
			fileToSend = platform->GetFileStore(platform->GetWebDir(), nameOfFileToSend, false);
		}
	}

	Network *net = reprap.GetNetwork();
	net->Write("HTTP/1.1 200 OK\n");

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
	else if (doingJsonResponse)
	{
		contentType = "application/json";
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
	net->Printf("Content-Type: %s\n", contentType);

	if (doingJsonResponse)
	{
		net->Printf("Content-Length: %u\n", strlen(jsonResponse));
	}
	else if (zip && fileToSend != NULL)
	{
		net->Write("Content-Encoding: gzip\n");
		net->Printf("Content-Length: %lu", fileToSend->Length());
	}

	net->Write("Connection: close\n");
	net->Write('\n');

	if (doingJsonResponse)
	{
		net->Write(jsonResponse);
	}
	net->SendAndClose(fileToSend);
}

//----------------------------------------------------------------------------------------------------

// Input from the client

void Webserver::CheckPassword()
{
	gotPassword = StringEndsWith(clientQualifier, password);
}

void Webserver::JsonReport(bool ok, const char* request)
{
	if (ok)
	{
		jsonResponse[STRING_LENGTH] = 0;
		if (reprap.Debug())
		{
			platform->Message(HOST_MESSAGE, "JSON response: ");
			platform->Message(HOST_MESSAGE, jsonResponse);
			platform->Message(HOST_MESSAGE, " queued\n");
		}
	}
	else
	{
		platform->Message(HOST_MESSAGE, "KnockOut request: ");
		platform->Message(HOST_MESSAGE, request);
		platform->Message(HOST_MESSAGE, " not recognised\n");
		clientRequest[0] = 0;
	}
}

void Webserver::GetJsonResponse(const char* request)
{
	if (StringStartsWith(request, "status"))	// new style status request
	{
		GetStatusResponse(1);
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "poll"))		// old style status request
	{
		GetStatusResponse(0);
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "gcode") && StringStartsWith(clientQualifier, "gcode="))
	{
		LoadGcodeBuffer(&clientQualifier[6], true);
		snprintf(jsonResponse, STRING_LENGTH, "{\"buff\":%u}", GetReportedGcodeBufferSpace());
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "files"))
	{
		const char* fileList = platform->GetMassStorage()->FileList(platform->GetGCodeDir(), false);
		snprintf(jsonResponse, STRING_LENGTH, "{\"files\":[%s]}", fileList);
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "fileinfo") && StringStartsWith(clientQualifier, "name="))
	{
		unsigned long length;
		float height, filament;
		bool found = GetFileInfo(clientQualifier + 5, length, height, filament);
		if (found)
		{
			snprintf(jsonResponse, STRING_LENGTH, "{\"size\":%lu,\"height\":\"%.2f\",\"filament\":\"%.1f\"}", length, height, filament);
		}
		else
		{
			snprintf(jsonResponse, STRING_LENGTH, "{}");
		}
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "name"))
	{
		snprintf(jsonResponse, STRING_LENGTH, "{\"myName\":\"%s\"}", myName);
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "password"))
	{
		CheckPassword();
		snprintf(jsonResponse, STRING_LENGTH, "{\"password\":\"%s\"}", (gotPassword) ? "right" : "wrong");
		JsonReport(true, request);
		return;
	}

	if (StringStartsWith(request, "axes"))
	{
		strncpy(jsonResponse, "{\"axes\":", STRING_LENGTH);
		char ch = '[';
		for (int8_t drive = 0; drive < AXES; drive++)
		{
			sncatf(jsonResponse, STRING_LENGTH, "%c\"%.1f\"", ch, platform->AxisLength(drive));
			ch = ',';
		}
		strncat(jsonResponse, "]}", STRING_LENGTH);
		JsonReport(true, request);
		return;
	}

	jsonResponse[0] = 0;
	JsonReport(false, request);
}

void Webserver::GetStatusResponse(uint8_t type)
{
	GCodes *gc = reprap.GetGCodes();
	if (type == 1)
	{
		// New-style status request
		// Send the printing/idle status
		char ch = (reprap.IsStopped()) ? 'S' : (gc->PrintingAFile()) ? 'P' : 'I';
		snprintf(jsonResponse, STRING_LENGTH, "{\"status\":\"%c\",\"heaters\":", ch);

		// Send the heater temperatures
		ch = '[';
		for (int8_t heater = 0; heater < HEATERS; heater++)
		{
			sncatf(jsonResponse, STRING_LENGTH, "%c\"%.1f\"", ch, reprap.GetHeat()->GetTemperature(heater));
			ch = ',';
		}

		// Send XYZ and extruder positions
		float liveCoordinates[DRIVES + 1];
		reprap.GetMove()->LiveCoordinates(liveCoordinates);
		strncat(jsonResponse, "],\"pos\":", STRING_LENGTH);		// announce the XYZ position
		ch = '[';
		// We currently provide the extruder 0 value here as well as XYZ. This is only expected by V0.69 and V0.70 of the web interface so it can be removed soon.
		for (int8_t drive = 0; drive < AXES + 1; drive++)
		//for (int8_t drive = 0; drive < AXES; drive++)
		{
			sncatf(jsonResponse, STRING_LENGTH, "%c\"%.2f\"", ch, liveCoordinates[drive]);
			ch = ',';
		}
		sncatf(jsonResponse, STRING_LENGTH, "],\"extr\":");		// announce the extruder positions
		ch = '[';
		for (int8_t drive = AXES; drive < DRIVES; drive++)		// loop through extruders
		{
			sncatf(jsonResponse, STRING_LENGTH, "%c\"%.3f\"", ch, gc->GetExtruderPosition(drive - AXES));
			ch = ',';
		}
		strncat(jsonResponse, "]", STRING_LENGTH);
	}
	else
	{
		// The old (deprecated) poll response lists the status, then all the heater temperatures, then the XYZ positions, then all the extruder positions.
		// These are all returned in a single vector called "poll".
		// This is a poor choice of format because we can't easily tell which is which unless we already know the number of heaters and extruders.
		char c = (gc->PrintingAFile()) ? 'P' : 'I';
		snprintf(jsonResponse, STRING_LENGTH, "{\"poll\":[\"%c\",", c); // Printing
		for (int8_t heater = 0; heater < HEATERS; heater++)
		{
			sncatf(jsonResponse, STRING_LENGTH, "\"%.1f\",", reprap.GetHeat()->GetTemperature(heater));
		}
		// Send XYZ and extruder positions
		float liveCoordinates[DRIVES + 1];
		reprap.GetMove()->LiveCoordinates(liveCoordinates);
		for (int8_t drive = 0; drive < DRIVES; drive++)	// loop through extruders
		{
			char ch = (drive == DRIVES - 1) ? ']' : ',';	// append ] to the last one but , to the others
			sncatf(jsonResponse, STRING_LENGTH, "\"%.2f\"%c", liveCoordinates[drive], ch);
		}
	}

	// Send the Z probe value
	int v0 = platform->ZProbe();
	int v1, v2;
	switch (platform->GetZProbeSecondaryValues(v1, v2))
	{
	case 1:
		sncatf(jsonResponse, STRING_LENGTH, ",\"probe\":\"%d (%d)\"", v0, v1);
		break;
	case 2:
		sncatf(jsonResponse, STRING_LENGTH, ",\"probe\":\"%d (%d, %d)\"", v0, v1, v2);
		break;
	default:
		sncatf(jsonResponse, STRING_LENGTH, ",\"probe\":\"%d\"", v0);
		break;
	}

	// Send the amount of buffer space available for gcodes
	sncatf(jsonResponse, STRING_LENGTH, ",\"buff\":%u", GetReportedGcodeBufferSpace());

	// Send the home state. To keep the messages short, we send 1 for homed and 0 for not homed, instead of true and false.
	if (type != 0)
	{
		sncatf(jsonResponse, STRING_LENGTH, ",\"homed\":[%d,%d,%d]",
				(gc->GetAxisIsHomed(0)) ? 1 : 0,
				(gc->GetAxisIsHomed(1)) ? 1 : 0,
				(gc->GetAxisIsHomed(2)) ? 1 : 0);
	}
	else
	{
		sncatf(jsonResponse, STRING_LENGTH, ",\"hx\":%d,\"hy\":%d,\"hz\":%d",
				(gc->GetAxisIsHomed(0)) ? 1 : 0,
				(gc->GetAxisIsHomed(1)) ? 1 : 0,
				(gc->GetAxisIsHomed(2)) ? 1 : 0);
	}

	// Send the response sequence number
	sncatf(jsonResponse, STRING_LENGTH, ",\"seq\":%u", (unsigned int) seq);

	// Send the response to the last command. Do this last because it is long and may need to be truncated.
	strncat(jsonResponse, ",\"resp\":\"", STRING_LENGTH);
	size_t jp = strnlen(jsonResponse, STRING_LENGTH);
	const char *p = gcodeReply;
	while (*p != 0 && jp < STRING_LENGTH - 2)	// leave room for the final '"}'
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
			if (jp == STRING_LENGTH - 3)
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
	strncat(jsonResponse, "\"}", STRING_LENGTH);
}

/*

 Parse a string in clientLine[] from the user's web browser

 Simple requests have the form:

 GET /page2.htm HTTP/1.1
 ^  Start clientRequest[] at clientLine[5]; stop at the blank or...

 ...fancier ones with arguments after a '?' go:

 GET /gather.asp?pwd=my_pwd HTTP/1.1
 ^ Start clientRequest[]
 ^ Start clientQualifier[]
 */

void Webserver::ParseGetPost()
{
	if (reprap.Debug())
	{
		platform->Message(HOST_MESSAGE, "HTTP request: ");
		platform->Message(HOST_MESSAGE, clientLine);
		platform->Message(HOST_MESSAGE, "\n");
	}

	int i = 5;
	int j = 0;
	clientRequest[j] = 0;
	clientQualifier[0] = 0;
	while (clientLine[i] != ' ' && clientLine[i] != '?')
	{
		clientRequest[j] = clientLine[i];
		j++;
		i++;
	}
	clientRequest[j] = 0;
	if (clientLine[i] == '?')
	{
		i++;
		j = 0;
		for(;;)
		{
			char c = clientLine[i++];
			if (c == ' ')
			{
				break;
			}
			else if (c == '%' && isalnum(clientLine[i]) && isalnum(clientLine[i + 1]))
			{
				c = clientLine[i++];
				unsigned int v = (isdigit(c)) ? (c - '0') : ((toupper(c) - 'A') + 10);
				c = clientLine[i++];
				v = (v << 4) + ((isdigit(c)) ? (c - '0') : ((toupper(c) - 'A') + 10));
				clientQualifier[j++] = (char)v;
			}
			else
			{
				clientQualifier[j++] = c;
			}
		}
		clientQualifier[j] = 0;
	}
}

void Webserver::InitialisePost()
{
	postSeen = false;
	receivingPost = false;
	boundaryCount = 0;
	postBoundary[0] = 0;
	postFileName[0] = 0;
	postFile = NULL;
}

void Webserver::ParseClientLine()
{
	if (StringStartsWith(clientLine, "GET"))
	{
		ParseGetPost();
		postSeen = false;
		getSeen = true;
		if (!clientRequest[0])
			strncpy(clientRequest, INDEX_PAGE, STRING_LENGTH);
		return;
	}

	if (StringStartsWith(clientLine, "POST"))
	{
		ParseGetPost();
		InitialisePost();
		postSeen = true;
		getSeen = false;
		if (!clientRequest[0])
		{
			strncpy(clientRequest, INDEX_PAGE, STRING_LENGTH);
		}
		return;
	}

	int bnd;

	if (postSeen && ((bnd = StringContains(clientLine, "boundary=")) >= 0))
	{
		if (strlen(&clientLine[bnd]) >= POST_LENGTH - 4)
		{
			platform->Message(HOST_MESSAGE, "Post boundary buffer overflow.\n");
			return;
		}
		postBoundary[0] = '-';
		postBoundary[1] = '-';
		strncpy(&postBoundary[2], &clientLine[bnd], POST_LENGTH - 3);
		strncat(postBoundary, "--", POST_LENGTH);
		return;
	}

	if (receivingPost && StringStartsWith(clientLine, "Content-Disposition:"))
	{
		bnd = StringContains(clientLine, "filename=\"");
		if (bnd < 0)
		{
			platform->Message(HOST_MESSAGE, "Post disposition gives no filename.\n");
			return;
		}
		int i = 0;
		while (clientLine[bnd] && clientLine[bnd] != '"')
		{
			postFileName[i++] = clientLine[bnd++];
			if (i >= POST_LENGTH)
			{
				i = 0;
				platform->Message(HOST_MESSAGE, "Post filename buffer overflow.\n");
			}
		}
		postFileName[i] = 0;
		return;
	}
}

// if you've gotten to the end of the line (received a newline
// character) and the line is blank, the http request has ended,
// so you can send a reply
void Webserver::BlankLineFromClient()
{
	clientLine[clientLinePointer] = 0;
	clientLinePointer = 0;

	if (getSeen)
	{
		SendFile(clientRequest);
		clientRequest[0] = 0;
		return;
	}

	if (postSeen)
	{
		receivingPost = true;
		postSeen = false;
		return;
	}

	if (receivingPost)
	{
		postFile = platform->GetFileStore(platform->GetGCodeDir(), postFileName, true);
		if (postFile == NULL || !postBoundary[0])
		{
			platform->Message(HOST_MESSAGE, "Can't open file for write or no post boundary: ");
			platform->Message(HOST_MESSAGE, postFileName);
			platform->Message(HOST_MESSAGE, "\n");
			InitialisePost();
			if (postFile != NULL)
			{
				postFile->Close();
			}
		}
	}
}

// Process a character from the client, returning true if we did more than just store it
bool Webserver::CharFromClient(char c)
{
	if (c == '\n' && clientLineIsBlank)
	{
		BlankLineFromClient();
		return true;
	}

	if (c == '\n')
	{
		clientLine[clientLinePointer] = 0;
		ParseClientLine();
		// you're starting a new line
		clientLineIsBlank = true;
		clientLinePointer = 0;
		return true;
	}
	else if (c != '\r')
	{
		// you've gotten a character on the current line
		clientLineIsBlank = false;
		clientLine[clientLinePointer] = c;
		clientLinePointer++;
		if (clientLinePointer >= STRING_LENGTH)
		{
			platform->Message(HOST_MESSAGE, "Client read buffer overflow. Data:\n");
			clientLine[STRING_LENGTH] = '\n';	// note that clientLine is now STRING_LENGTH+2 characters long to make room for these
			clientLine[STRING_LENGTH + 1] = 0;
			platform->Message(HOST_MESSAGE, clientLine);

			reprap.GetNetwork()->SendAndClose(NULL);		// close the connection

			clientLinePointer = 0;
			clientLine[clientLinePointer] = 0;
			clientLineIsBlank = true;
			return true;
		}
	}
	return false;
}

// Deal with input/output from/to the client (if any) one byte at a time.

void Webserver::Spin()
{
	if (!active)
		return;

	Network *net = reprap.GetNetwork();
	if (net->HaveData())
	{
		for (uint8_t i = 0; i < 16; ++i)
		{
			char c;
			bool ok = net->Read(c);

			if (!ok)
			{
				// We ran out of data before finding a complete request.
				// This can happen if the network connection was lost, so only report it if debug is enabled
				if (reprap.Debug())
				{
					platform->Message(HOST_MESSAGE, "Webserver: incomplete request\n");
				}
				net->SendAndClose(NULL);
				break;
			}

			if (receivingPost && postFile != NULL)
			{
				if (MatchBoundary(c))
				{
					postFile->Close();
					SendFile(clientRequest);
					clientRequest[0] = 0;
					InitialisePost();
				}
				break;
			}

			if (CharFromClient(c))
			{
				break;	// break if we did more than just store the character
			}
		}
	}

	platform->ClassReport("Webserver", longWait);
}

//******************************************************************************************

// Constructor and initialisation

Webserver::Webserver(Platform* p)
{
	platform = p;
	active = false;
	gotPassword = false;
}

void Webserver::Init()
{
	receivingPost = false;
	postSeen = false;
	getSeen = false;
	clientLineIsBlank = true;
	clientLinePointer = 0;
	clientLine[0] = 0;
	clientRequest[0] = 0;
	SetPassword(DEFAULT_PASSWORD);
	SetName(DEFAULT_NAME);
	//gotPassword = false;
	gcodeReadIndex = gcodeWriteIndex = 0;
	InitialisePost();
	lastTime = platform->Time();
	longWait = lastTime;
	active = true;
	gcodeReply[0] = 0;
	seq = 0;

	// Reinitialise the message file

	//platform->GetMassStorage()->Delete(platform->GetWebDir(), MESSAGE_FILE);
}

void Webserver::Exit()
{
	platform->Message(HOST_MESSAGE, "Webserver class exited.\n");
	active = false;
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
			strncat(gcodeReply, s, STRING_LENGTH);
		}
		else
		{
			strncpy(gcodeReply, s, STRING_LENGTH);
		}
		gcodeReply[STRING_LENGTH] = 0;	// array is dimensioned to STRING_LENGTH+1
	}
	++seq;
}

void Webserver::AppendReply(const char *s)
{
	strncat(gcodeReply, s, STRING_LENGTH);
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
	return (temp > maxReportedFreeBuf) ? maxReportedFreeBuf : (temp < minReportedFreeBuf) ? 0 : temp;
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
