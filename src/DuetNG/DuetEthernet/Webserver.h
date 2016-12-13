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

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <cstdint>
#include <cstddef>

// List of protocols that can execute G-Codes
enum class WebSource
{
	HTTP,
	Telnet
};

const uint16_t gcodeBufferLength = 512;			// size of our gcode ring buffer, preferably a power of 2
const uint16_t webMessageLength = 2000;			// maximum length of the web message we accept after decoding
const size_t maxQualKeys = 5;					// max number of key/value pairs in the qualifier
const size_t maxHttpSessions = 8;				// maximum number of simultaneous HTTP sessions
const uint32_t httpSessionTimeout = 20000;		// HTTP session timeout in milliseconds

class Platform;
class Network;

class Webserver
{   
public:

	friend class Platform;

	Webserver(Platform* p, Network *n);
	void Init();
	void Spin();
	void Exit();
	void Diagnostics(MessageType mtype);

	bool GCodeAvailable(const WebSource source) const;
	char ReadGCode(const WebSource source);
	void HandleGCodeReply(const WebSource source, OutputBuffer *reply);
	void HandleGCodeReply(const WebSource source, const char *reply);
	uint32_t GetReplySeq() const { return seq; }
	// Returns the available G-Code buffer space of the HTTP interpreter (may be dropped in a future version)
	uint16_t GetGCodeBufferSpace(const WebSource source) const { return 0; }

private:
	uint32_t seq;
};

#endif
