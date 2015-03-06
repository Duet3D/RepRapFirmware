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

const unsigned int gcodeBufferLength = 512;		// size of our gcode ring buffer, preferably a power of 2

/* HTTP */

#define KO_START "rr_"
#define KO_FIRST 3

const unsigned int webUploadBufferSize = 2300;	// maximum size of HTTP GET upload packets (webMessageLength - 700)
const unsigned int webMessageLength = 3000;		// maximum length of the web message we accept after decoding

const unsigned int maxCommandWords = 4;			// max number of space-separated words in the command
const unsigned int maxQualKeys = 5;				// max number of key/value pairs in the qualifier
const unsigned int maxHeaders = 16;				// max number of key/value pairs in the headers

const unsigned int jsonReplyLength = 2048;		// size of buffer used to hold JSON reply

const unsigned int maxSessions = 8;				// maximum number of simultaneous HTTP sessions
const unsigned int httpSessionTimeout = 30;		// HTTP session timeout in seconds

/* FTP */

const unsigned int ftpResponseLength = 128;		// maximum FTP response length
const unsigned int ftpMessageLength = 128;		// maximum line length for incoming FTP commands

/* Telnet */

const unsigned int telnetMessageLength = 256;	// maximum line length for incoming Telnet commands


class Webserver;

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

		virtual bool DoFastUpload();
		virtual bool DoingFastUpload() const;
	    bool FlushUploadData();
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
	    char filenameBeingUploaded[MaxFilenameLength];
	    const char *uploadPointer;							// pointer to start of uploaded data not yet written to file
	    unsigned int uploadLength;							// amount of data not yet written to file

	    virtual bool StartUpload(FileStore *file);
	    virtual bool StoreUploadData(const char* data, unsigned int len);
		bool IsUploading() const;
	    virtual void FinishUpload(uint32_t fileLength);
};

class Webserver
{   
  public:

    Webserver(Platform* p, Network *n);
    void Init();
    void Spin();
    void Exit();
    void Diagnostics();

    bool GCodeAvailable();
    char ReadGCode();
    unsigned int GetGcodeBufferSpace() const;

    void ConnectionLost(const ConnectionState *cs);
    void ConnectionError();

    friend class Platform;

  protected:

    void ResponseToWebInterface(const char *s, bool error);
    void AppendResponseToWebInterface(const char* s);

	class HttpInterpreter : public ProtocolInterpreter
	{
		public:

			HttpInterpreter(Platform *p, Webserver *ws, Network *n);
			void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort) override;
			bool CharFromClient(const char c) override;
			void ResetState() override;
			bool NeedMoreData() override;

			bool DoFastUpload() override;
			bool DoingFastUpload() const override;
			void CancelUpload() override;
			void CancelUpload(uint32_t remoteIP);

			void ResetSessions();
			void CheckSessions();

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

			struct KeyValueIndices
			{
				const char* key;
				const char* value;
			};

			void SendFile(const char* nameOfFileToSend);
			void SendGCodeReply();
			void SendJsonResponse(const char* command);
			bool GetJsonResponse(const char* request, StringRef& response, const char* key, const char* value, size_t valueLength, bool& keepOpen);
			void GetJsonUploadResponse(StringRef& response);
			bool ProcessMessage();
			bool RejectMessage(const char* s, unsigned int code = 500);

			bool Authenticate();
			bool IsAuthenticated() const;
			void UpdateAuthentication();
			void RemoveAuthentication();

			HttpState state;

			// Buffers for processing HTTP input
			char clientMessage[webMessageLength];			// holds the command, qualifier, and headers
			unsigned int clientPointer;						// current index into clientMessage
			char decodeChar;

			const char* commandWords[maxCommandWords];
			KeyValueIndices qualifiers[maxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
			KeyValueIndices headers[maxHeaders];			// offsets into clientHeader of the key/value pairs
			unsigned int numCommandWords;
			unsigned int numQualKeys;						// number of qualifier keys we have found, <= maxQualKeys
			unsigned int numHeaderKeys;						// number of keys we have found, <= maxHeaders

		    // HTTP sessions
			struct HttpSession
			{
				uint32_t ip;
				float lastQueryTime;
				bool isPostUploading;
				uint16_t postPort;
			};

			HttpSession sessions[maxSessions];
		    unsigned int numActiveSessions;

		protected:
		    bool uploadingTextData;							// do we need to count UTF-8 continuation bytes?
		    uint32_t numContinuationBytes;					// number of UTF-8 continuation bytes we have received

		    uint32_t postFileLength, uploadedBytes;			// how many POST bytes do we expect and how many have already been written?

		    bool StartUpload(FileStore *file) override;
			bool StoreUploadData(const char* data, unsigned int len) override;
			void FinishUpload(uint32_t fileLength) override;
	};
	HttpInterpreter *httpInterpreter;

	class FtpInterpreter : public ProtocolInterpreter
	{
		public:

			FtpInterpreter(Platform *p, Webserver *ws, Network *n);
			void ConnectionEstablished() override;
			void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t localPort) override;
			bool CharFromClient(const char c) override;
			void ResetState() override;

			bool DoingFastUpload() const override;

		private:

			enum FtpState
			{
				authenticating,			// not logged in
				authenticated,			// logged in
				waitingForPasvPort,		// waiting for connection to be established on PASV port
				pasvPortConnected,		// client connected to PASV port, ready to send data
				doingPasvIO				// client is connected and data is being transferred
			};
			FtpState state;

			char clientMessage[ftpMessageLength];
			unsigned int clientPointer;
			char ftpResponse[ftpResponseLength];

			char filename[MaxFilenameLength];
			char currentDir[MaxFilenameLength];

			float portOpenTime;

			void ProcessLine();
			void SendReply(int code, const char *message, bool keepConnection = true);
			void SendFeatures();

			void ReadFilename(int start);
			void ChangeDirectory(const char *newDirectory);
	};
	FtpInterpreter *ftpInterpreter;

	class TelnetInterpreter : public ProtocolInterpreter
	{
		public:

			TelnetInterpreter(Platform *p, Webserver *ws, Network *n);
			void ConnectionEstablished() override;
			void ConnectionLost(uint32_t remoteIP, uint16_t remotePort, uint16_t local_port) override;
			bool CharFromClient(const char c) override;
			void ResetState() override;
			bool NeedMoreData() override;

			void HandleGcodeReply(const char* reply);
			bool HasRemainingData() const;
			void RemainingDataSent();

		private:

			enum TelnetState
			{
				authenticating,			// not logged in
				authenticated			// logged in
			};
			TelnetState state;

			char clientMessage[telnetMessageLength];
			unsigned int clientPointer;
			bool sendPending;

			void ProcessLine();
	};
	TelnetInterpreter *telnetInterpreter;

	// G-Code processing
    void ProcessGcode(const char* gc);
    void LoadGcodeBuffer(const char* gc);
    void StoreGcodeData(const char* data, size_t len);

  private:

    // Buffer to hold gcode that is ready for processing
    char gcodeBuffer[gcodeBufferLength];
    unsigned int gcodeReadIndex, gcodeWriteIndex;	// head and tail indices into gcodeBuffer

    // Misc
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

inline void Webserver::HttpInterpreter::FinishUpload(uint32_t fileLength) { ProtocolInterpreter::FinishUpload(fileLength + numContinuationBytes); }

inline bool Webserver::TelnetInterpreter::NeedMoreData() { return false; }	// we don't want a Telnet connection to block everything else
inline bool Webserver::TelnetInterpreter::HasRemainingData() const { return sendPending; }
inline void Webserver::TelnetInterpreter::RemainingDataSent() { sendPending = false; }

inline unsigned int Webserver::GetGcodeBufferSpace() const { return (gcodeReadIndex - gcodeWriteIndex - 1u) % gcodeBufferLength; }

#endif
