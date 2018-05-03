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

#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"
#include "Storage/FileData.h"

/* Generic values */

const size_t gcodeBufferLength = 512;			// size of our gcode ring buffer, preferably a power of 2

/* HTTP */

#define KO_START "rr_"
#define KO_FIRST 3

const uint16_t webMessageLength = TCP_MSS;		// maximum length of the web message we accept after decoding
const size_t minHttpResponseSize = 768;			// minimum number of bytes required for an HTTP response

const size_t maxCommandWords = 4;				// max number of space-separated words in the command
const size_t maxQualKeys = 5;					// max number of key/value pairs in the qualifier
const size_t maxHeaders = 30;					// max number of key/value pairs in the headers

const size_t  maxHttpSessions = 8;				// maximum number of simultaneous HTTP sessions
const uint32_t httpSessionTimeout = 8000;		// HTTP session timeout in milliseconds

/* FTP */

const uint16_t ftpMessageLength = 128;			// maximum line length for incoming FTP commands
const uint32_t ftpPasvPortTimeout = 10000;		// maximum time to wait for an FTP data connection in milliseconds

/* Telnet */

const uint32_t telnetSetupDuration = 4000;		// ignore the first Telnet request within this duration (in ms)


class Webserver;

// List of protocols that can execute G-Codes
enum class WebSource
{
	HTTP,
	Telnet
};

// This is the abstract class for all supported protocols
// Any inherited class should implement a state machine to increase performance and reduce memory usage.
class ProtocolInterpreter
{
	public:

		ProtocolInterpreter(Platform *p, Webserver *ws, Network *n);
		virtual ~ProtocolInterpreter() { }					// to keep Eclipse happy
		virtual void Diagnostics(MessageType mtype) = 0;
		virtual void Spin();

		virtual void ConnectionEstablished();
		virtual void ConnectionLost(Connection conn /*const ConnectionState *cs*/) { }
		virtual bool CanParseData();
		virtual bool CharFromClient(const char c) = 0;
		virtual void NoMoreDataAvailable();

		virtual bool DoingFastUpload() const;
		virtual void DoFastUpload();
		void CancelUpload();								// may be called from ISR!

	protected:

		Platform * const platform;
		Webserver * const webserver;
		Network * const network;

		// Information for file uploading
		enum UploadState
		{
			notUploading,									// no upload in progress
			uploadOK,										// upload in progress, no error so far
			uploadError										// upload in progress but had error
		};

		UploadState uploadState;
		FileData fileBeingUploaded;
		char filenameBeingUploaded[MaxFilenameLength];

		bool StartUpload(FileStore *file, const char *fileName);
		bool IsUploading() const;
		bool FinishUpload(uint32_t fileLength);
};

class Webserver
{   
public:

	friend class Platform;
	friend class ProtocolInterpreter;

	Webserver(Platform* p, Network *n);
	void Init();
	void Spin();
	void Exit();
	void Diagnostics(MessageType mtype);

	void HandleGCodeReply(const WebSource source, OutputBuffer *reply);
	void HandleGCodeReply(const WebSource source, const char *reply);
	uint32_t GetReplySeq() const;

	void ConnectionLost(Connection conn /*const ConnectionState *cs*/);
	void ConnectionError();

protected:

	class HttpInterpreter : public ProtocolInterpreter
	{
	public:

		HttpInterpreter(Platform *p, Webserver *ws, Network *n);
		void Spin();
		void Diagnostics(MessageType mtype) override;
		void ConnectionLost(Connection conn /*const ConnectionState *cs*/) override;
		bool CanParseData() override;
		bool CharFromClient(const char c) override;
		void NoMoreDataAvailable() override;
		void ResetState();
		void ResetSessions();

		bool DoingFastUpload() const override;
		void DoFastUpload();

		void HandleGCodeReply(OutputBuffer *reply);
		void HandleGCodeReply(const char *reply);
		uint32_t GetReplySeq() const;

	private:

		// HTTP server state enumeration. The order is important, in particular xxxEsc1 must follow xxx, and xxxEsc2 must follow xxxEsc1.
		// We assume that qualifier keys do not contain escapes, because none of ours needs to be encoded. If we are sent escapes in the key,
		// it won't do any harm, but the key won't be recognised even if it would be valid were it decoded.
		enum HttpState
		{
			doingCommandWord,			// receiving a word in the first line of the HTTP request
			doingFilename,				// receiving the filename (second word in the command line)
			doingFilenameEsc1,			// received '%' in the filename (e.g. we are being asked for a filename with spaces in it)
			doingFilenameEsc2,			// received '%' and one hex digit in the filename
			doingQualifierKey,			// receiving a key name in the HTTP request
			doingQualifierValue,		// receiving a key value in the HTTP request
			doingQualifierValueEsc1,	// received '%' in the qualifier
			doingQualifierValueEsc2,	// received '%' and one hex digit in the qualifier
			doingHeaderKey,				// receiving a header key
			expectingHeaderValue,		// expecting a header value
			doingHeaderValue,			// receiving a header value
			doingHeaderContinuation		// received a newline after a header value
		};
		HttpState state;

		struct KeyValueIndices
		{
			const char* key;
			const char* value;
		};

		void SendFile(const char* nameOfFileToSend, bool isWebFile);
		void SendGCodeReply();
		void SendJsonResponse(const char* command);
		void GetJsonResponse(const char* request, OutputBuffer *&response, bool& keepOpen);
		bool ProcessMessage();
		bool RejectMessage(const char* s, unsigned int code = 500);

		// Buffers for processing HTTP input
		char clientMessage[webMessageLength + 3];		// holds the command, qualifier, and headers
		size_t clientPointer;							// current index into clientMessage
		char decodeChar;

		const char* commandWords[maxCommandWords];
		KeyValueIndices qualifiers[maxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
		KeyValueIndices headers[maxHeaders];			// offsets into clientHeader of the key/value pairs
		size_t numCommandWords;
		size_t numQualKeys;								// number of qualifier keys we have found, <= maxQualKeys
		size_t numHeaderKeys;							// number of keys we have found, <= maxHeaders

		// HTTP sessions
		struct HttpSession
		{
			uint32_t ip;
			uint32_t lastQueryTime;
			bool isPostUploading;
			uint16_t postPort;
		};

		HttpSession sessions[maxHttpSessions];
		uint8_t numSessions;
		uint8_t clientsServed;

		bool Authenticate();
		bool IsAuthenticated() const;
		void UpdateAuthentication();
		bool RemoveAuthentication();
		const char* GetKeyValue(const char *key) const;	// return the value of the specified key, or nullptr if not present

		// Responses from GCodes class
		uint32_t seq;									// Sequence number for G-Code replies
		OutputStack *gcodeReply;

		// File uploads
		uint32_t postFileLength, uploadedBytes;			// How many POST bytes do we expect and how many have already been written?
		time_t fileLastModified;

		// Deferred requests (rr_fileinfo)
		volatile Connection deferredRequestConnection;	// Which connection expects a response for a deferred request?
		char filenameBeingProcessed[MaxFilenameLength];	// The filename being processed (for rr_fileinfo)

		void ProcessDeferredRequest();
	};
	HttpInterpreter *httpInterpreter;

	class FtpInterpreter : public ProtocolInterpreter
	{
	public:

		FtpInterpreter(Platform *p, Webserver *ws, Network *n);
		void Diagnostics(MessageType mtype) override;

		void ConnectionEstablished() override;
		void ConnectionLost(Connection conn /*const ConnectionState *cs*/) override;
		bool CharFromClient(const char c) override;
		void ResetState();

		bool DoingFastUpload() const override;

	private:

		enum FtpState
		{
			idle,					// no client connected
			authenticating,			// not logged in
			authenticated,			// logged in
			waitingForPasvPort,		// waiting for connection to be established on PASV port
			pasvPortConnected,		// client connected to PASV port, ready to send data
			doingPasvIO				// client is connected and data is being transferred
		};
		FtpState state;
		uint8_t connectedClients;

		char clientMessage[ftpMessageLength];
		size_t clientPointer;

		String<MaxFilenameLength> filename;
		char currentDir[MaxFilenameLength];

		uint32_t portOpenTime;

		void ProcessLine();
		void SendReply(int code, const char *message, bool keepConnection = true);
		void SendFeatures();

		void ReadFilename(uint16_t start);
		void ChangeDirectory(const char *newDirectory);
	};
	FtpInterpreter *ftpInterpreter;

	class TelnetInterpreter : public ProtocolInterpreter
	{
	public:

		TelnetInterpreter(Platform *p, Webserver *ws, Network *n);
		void Diagnostics(MessageType mtype) override;

		void ConnectionEstablished() override;
		void ConnectionLost(Connection conn /*const ConnectionState *cs*/) override;
		bool CanParseData() override;
		bool CharFromClient(const char c) override;
		void ResetState();

		void HandleGCodeReply(OutputBuffer *reply);
		void HandleGCodeReply(const char *reply);

		void SendGCodeReply();

	private:

		enum TelnetState
		{
			idle,					// not connected
			justConnected,			// not logged in, but the client has just connected
			authenticating,			// not logged in
			authenticated			// logged in
		};
		TelnetState state;
		uint8_t connectedClients;
		uint32_t connectTime;

		bool processNextLine;
		char clientMessage[GCODE_LENGTH];
		size_t clientPointer;

		bool ProcessLine();

		// Converted response from GCodes class (NL -> CRNL)
		OutputBuffer * volatile gcodeReply;
	};
	TelnetInterpreter *telnetInterpreter;

  private:

    Platform* platform;
    Network* network;
    bool webserverActive;
	NetworkTransaction *currentTransaction;
	volatile Connection readingConnection;

    uint32_t longWait;
};

inline bool ProtocolInterpreter::CanParseData() { return true; }
inline bool ProtocolInterpreter::DoingFastUpload() const { return false; }
inline bool ProtocolInterpreter::IsUploading() const { return uploadState != notUploading; }

inline uint32_t Webserver::GetReplySeq() const { return httpInterpreter->GetReplySeq(); }

inline uint32_t Webserver::HttpInterpreter::GetReplySeq() const { return seq; }

#endif
