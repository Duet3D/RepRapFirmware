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
const float httpSessionTimeout = 20.0;			// HTTP session timeout in seconds

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
	static const uint32_t lastFragmentFlag = 0x80000000;
	static const size_t uploadBufLength = 8192;

	enum UploadState
	{
		uploading = 0,
		wrongFragment,
		cantCreate,
		cantWrite,
		wrongLength,
		cantClose,
		notUploading,
		uploadBusy
	};

    // HTTP sessions
	struct HttpSession
	{
		uint32_t ip;
		uint32_t nextFragment;
		float lastQueryTime;
		FileData fileBeingUploaded;
		uint32_t postLength;
		uint32_t bytesWritten;
		UploadState uploadState;
		size_t bufNumber;
		size_t bufPointer;
		bool isAuthenticated;
	};

	void ResetState();
	void CheckSessions();

	bool CharFromClient(char c, const char* &error);
	bool ProcessFirstFragment(HttpSession& session, const char* command, bool isOnlyFragment);
	void ProcessUploadFragment(HttpSession& session, const char* request, size_t length, uint32_t fragment);
	void StartUpload(HttpSession& session, const char* fileName, uint32_t fileLength);
	void FinishUpload(HttpSession& session);
	void CancelUpload(HttpSession& session);
	bool GetJsonResponse(uint32_t remoteIp, const char* request, OutputBuffer *&response, const char* key, const char* value, size_t valueLength, bool& keepOpen);
	void SendConfigFile(HttpSession& session);

	// Deal with incoming G-Codes
	char gcodeBuffer[gcodeBufferLength];
	uint16_t gcodeReadIndex, gcodeWriteIndex;		// head and tail indices into gcodeBuffer
	uint32_t seq;									// sequence number for G-Code replies

	void LoadGcodeBuffer(const char* gc);
	void ProcessGcode(const char* gc);
	void StoreGcodeData(const char* data, uint16_t len);
	void SendGCodeReply(HttpSession& session);
	uint16_t GetGCodeBufferSpace() const;

	HttpSession *StartSession(uint32_t ip);			// start a new session for this requester
	HttpSession *FindSession(uint32_t ip);			// find an existing session for this requester
	void DeleteSession(uint32_t ip);				// delete a session

	// Response from GCodes class
	OutputStack *gcodeReply;

    enum HttpState
	{
		doingFilename,				// receiving the filename (second word in the command line)
		doingFilenameEsc1,			// received '%' in the filename (e.g. we are being asked for a filename with spaces in it)
		doingFilenameEsc2,			// received '%' and one hex digit in the filename
		doingQualifierKey,			// receiving a key name in the HTTP request
		doingQualifierValue,		// receiving a key value in the HTTP request
		doingQualifierValueEsc1,	// received '%' in the qualifier
		doingQualifierValueEsc2,	// received '%' and one hex digit in the qualifier
	};
	HttpState state;

	struct KeyValueIndices
	{
		const char* key;
		const char* value;
	};

	uint32_t uploadBuffers[2][uploadBufLength/4];				// two 8K buffers for uploading files
	uint32_t uploadIp;								// session that owns the upload buffer
	Platform *platform;
	Network *network;
    float longWait;
    bool webserverActive;

	char clientMessage[webMessageLength + 3];		// holds the command, qualifier, and headers
	size_t clientPointer;							// current index into clientMessage
	char decodeChar;

	bool processingDeferredRequest;					// it's no good idea to parse 128kB of text in one go...

	KeyValueIndices qualifiers[maxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
	size_t numQualKeys;								// number of qualifier keys we have found, <= maxQualKeys

	HttpSession sessions[maxHttpSessions];
	size_t numSessions, clientsServed;
};

inline uint16_t Webserver::GetGCodeBufferSpace() const
{
	return (gcodeReadIndex - gcodeWriteIndex - 1u) % gcodeBufferLength;
}

#endif
