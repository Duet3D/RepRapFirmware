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

const uint16_t gcodeBufferLength = 512;			// size of our gcode ring buffer, preferably a power of 2

/* HTTP */

#define KO_START "rr_"
#define KO_FIRST 3

const uint16_t webUploadBufferSize = 2300;		// maximum size of HTTP GET upload packets (webMessageLength - 700)
const uint16_t webMessageLength = 3000;			// maximum length of the web message we accept after decoding
const size_t minHttpResponseSize = 1024;		// minimum number of bytes required for an HTTP response

const size_t maxCommandWords = 4;				// max number of space-separated words in the command
const size_t maxQualKeys = 5;					// max number of key/value pairs in the qualifier
const size_t maxHeaders = 16;					// max number of key/value pairs in the headers

const size_t  maxHttpSessions = 8;				// maximum number of simultaneous HTTP sessions
const float httpSessionTimeout = 8.0;			// HTTP session timeout in seconds

/* FTP */

const uint16_t ftpResponseLength = 128;			// maximum FTP response length
const uint16_t ftpFileListLineLength = 256;		// maximum length for one FTP file listing line
const uint16_t ftpMessageLength = 128;			// maximum line length for incoming FTP commands
const float ftpPasvPortTimeout = 10.0;			// maximum time to wait for an FTP data connection

/* Telnet */

const uint16_t telnetMessageLength = 128;		// maximum line length for incoming Telnet commands
const float telnetSetupDuration = 4.0;			// ignore the first Telnet request within this duration


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

		virtual void ConnectionEstablished() { }
		virtual void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort) { }
		virtual bool CharFromClient(const char c) = 0;
		virtual void ResetState() = 0;
		virtual bool NeedMoreData();

		virtual bool DoFastUpload(NetworkTransaction *transaction);
		virtual bool DoingFastUpload() const;
	    virtual void CancelUpload();

	protected:

	    Platform *platform;
	    Webserver *webserver;
	    Network *network;

	    // Information for file uploading
	    enum UploadState
	    {
			notUploading,									// no upload in progress
			uploadOK,										// upload in progress, no error so far
			uploadError										// upload in progress but had error
	    };

	    UploadState uploadState;
	    FileData fileBeingUploaded;
	    char filenameBeingUploaded[FILENAME_LENGTH];

	    virtual bool StartUpload(FileStore *file);
		bool IsUploading() const;
	    virtual void FinishUpload(uint32_t fileLength);
};

class Webserver
{   
  public:

	friend class Platform;

	Webserver(Platform* p, Network *n);
	void Init();
	void Spin();
	void Exit();
	void Diagnostics();

	bool GCodeAvailable(const WebSource source) const;
	char ReadGCode(const WebSource source);
	void HandleGCodeReply(const WebSource source, OutputBuffer *reply);
	void HandleGCodeReply(const WebSource source, const char *reply);
	uint32_t GetReplySeq() const;

	// Returns the available G-Code buffer space of the HTTP interpreter (may be dropped in a future version)
	uint16_t GetGCodeBufferSpace(const WebSource source) const;

	void ConnectionLost(const ConnectionState *cs);
	void ConnectionError();

  protected:

	class HttpInterpreter : public ProtocolInterpreter
	{
		public:

			HttpInterpreter(Platform *p, Webserver *ws, Network *n);
			void Diagnostics();
			void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort) override;
			bool CharFromClient(const char c) override;
			void ResetState() override;
			bool NeedMoreData() override;

			bool DoFastUpload(NetworkTransaction *transaction) override;
			bool DoingFastUpload() const override;
			void CancelUpload() override;
			void CancelUpload(uint32_t remoteIP);

			void ResetSessions();
			void CheckSessions();

			bool GCodeAvailable() const;
			char ReadGCode();
			void HandleGCodeReply(OutputBuffer *reply);
			void HandleGCodeReply(const char *reply);
			uint16_t GetGCodeBufferSpace() const;
			uint32_t GetReplySeq() const;

			bool IsReady();					// returns true if the transaction can be parsed

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
				doingHeaderContinuation,	// received a newline after a header value
				doingPost					// receiving post data
			};
			HttpState state;

			struct KeyValueIndices
			{
				const char* key;
				const char* value;
			};

			void SendFile(const char* nameOfFileToSend);
			void SendConfigFile(NetworkTransaction *transaction);
			void SendGCodeReply(NetworkTransaction *transaction);
			void SendJsonResponse(const char* command);
			bool GetJsonResponse(const char* request, OutputBuffer *&response, const char* key, const char* value, size_t valueLength, bool& keepOpen);
			void GetJsonUploadResponse(OutputBuffer *response);
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
				float lastQueryTime;
				bool isPostUploading;
				uint16_t postPort;
			};

			HttpSession sessions[maxHttpSessions];
			size_t numSessions, clientsServed;

			bool Authenticate();
			bool IsAuthenticated() const;
			void UpdateAuthentication();
			void RemoveAuthentication();

			// Deal with incoming G-Codes

			char gcodeBuffer[gcodeBufferLength];
			uint16_t gcodeReadIndex, gcodeWriteIndex;		// head and tail indices into gcodeBuffer
			uint32_t seq;									// sequence number for G-Code replies

			void LoadGcodeBuffer(const char* gc);
			void ProcessGcode(const char* gc);
			void StoreGcodeData(const char* data, uint16_t len);

			// Response from GCodes class

			OutputStack *gcodeReply;

		protected:
			bool processingDeferredRequest;					// it's no good idea to parse 128kB of text in one go...
		    bool uploadingTextData;							// do we need to count UTF-8 continuation bytes?
		    uint32_t numContinuationBytes;					// number of UTF-8 continuation bytes we have received

		    uint32_t postFileLength, uploadedBytes;			// how many POST bytes do we expect and how many have already been written?

		    bool StartUpload(FileStore *file) override;
			void WriteUploadedData(const char* buffer, unsigned int length);
			void FinishUpload(uint32_t fileLength) override;
	};
	HttpInterpreter *httpInterpreter;

	class FtpInterpreter : public ProtocolInterpreter
	{
		public:

			FtpInterpreter(Platform *p, Webserver *ws, Network *n);
			void Diagnostics();
			void ConnectionEstablished() override;
			void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort) override;
			bool CharFromClient(const char c) override;
			void ResetState() override;

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
			unsigned int clientPointer;
			char ftpResponse[ftpResponseLength];

			char filename[FILENAME_LENGTH];
			char currentDir[FILENAME_LENGTH];

			float portOpenTime;

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
			void Diagnostics();
			void ConnectionEstablished() override;
			void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t local_port) override;
			bool CharFromClient(const char c) override;
			void ResetState() override;
			bool NeedMoreData() override;

			bool GCodeAvailable() const;
			char ReadGCode();
			void HandleGCodeReply(OutputBuffer *reply);
			void HandleGCodeReply(const char *reply);
			uint16_t GetGCodeBufferSpace() const;

			bool HasDataToSend() const;
			void SendGCodeReply(NetworkTransaction *transaction);

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
			float connectTime;

			char clientMessage[telnetMessageLength];
			uint16_t clientPointer;

			void ProcessLine();

			// Deal with incoming G-Codes

			char gcodeBuffer[gcodeBufferLength];
			uint16_t gcodeReadIndex, gcodeWriteIndex;		// head and tail indices into gcodeBuffer

			void ProcessGcode(const char* gc);
			void StoreGcodeData(const char* data, uint16_t len);

			// Converted response from GCodes class (NL -> CRNL)

			OutputBuffer * volatile gcodeReply;
	};
	TelnetInterpreter *telnetInterpreter;

  private:

    Platform* platform;
    Network* network;
    bool webserverActive;
    const ConnectionState *readingConnection;

    float lastTime;
    float longWait;
};

inline bool ProtocolInterpreter::NeedMoreData()  { return true; }
inline bool ProtocolInterpreter::DoingFastUpload() const { return false; }
inline bool ProtocolInterpreter::IsUploading() const { return uploadState != notUploading; }

inline uint32_t Webserver::GetReplySeq() const { return httpInterpreter->GetReplySeq(); }

inline void Webserver::HttpInterpreter::FinishUpload(uint32_t fileLength) { ProtocolInterpreter::FinishUpload(fileLength + numContinuationBytes); }
inline uint16_t Webserver::HttpInterpreter::GetGCodeBufferSpace() const { return (gcodeReadIndex - gcodeWriteIndex - 1u) % gcodeBufferLength; }
inline bool Webserver::HttpInterpreter::GCodeAvailable() const { return gcodeReadIndex != gcodeWriteIndex; }
inline uint32_t Webserver::HttpInterpreter::GetReplySeq() const { return seq; }

inline bool Webserver::TelnetInterpreter::NeedMoreData() { return false; }	// We don't want a Telnet connection to block everything else
inline uint16_t Webserver::TelnetInterpreter::GetGCodeBufferSpace() const { return (gcodeReadIndex - gcodeWriteIndex - 1u) % gcodeBufferLength; }
inline bool Webserver::TelnetInterpreter::GCodeAvailable() const { return gcodeReadIndex != gcodeWriteIndex; }

inline bool Webserver::TelnetInterpreter::HasDataToSend() const { return gcodeReply != nullptr; }

#endif
