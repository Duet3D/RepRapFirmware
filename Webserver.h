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

const unsigned int gcodeBufLength = 512;		// size of our gcode ring buffer, preferably a power of 2

/* HTTP */

#define KO_START "rr_"
#define KO_FIRST 3

const unsigned int webUploadBufferSize = 2300;	// maximum size of HTTP upload packets (webMessageLength - 700)
const unsigned int webMessageLength = 3000;		// maximum length of the web message we accept after decoding, excluding POST data
const unsigned int maxFilenameLength = 100;		// maximum length of a filename (inc. path from root) that we can upload
//const unsigned int postBoundaryLength = 100;	// max length of the POST boundary string
//const unsigned int postFilenameLength = 100;	// max length of the POST filename
//const unsigned int postDataLength = 1000;		// max amount of POST data

const unsigned int maxCommandWords = 4;			// max number of space-separated words in the command
const unsigned int maxQualKeys = 5;				// max number of key/value pairs in the qualifier
const unsigned int maxHeaders = 10;				// max number of key/value pairs in the headers

const unsigned int jsonReplyLength = 1000;		// size of buffer used to hold JSON reply

/* FTP */

const unsigned int ftpMessageLength = 256;		// maximum line length for incoming FTP commands
const unsigned int ftpResponseLength = 256;		// maximum FTP response length

/* Telnet */

const unsigned int telnetMessageLength = 256;	// maximum line length for incoming Telnet commands


class Webserver;

// This is the abstract class for all supported protocols
// Any inherited class should implement a state machine to increase performance and reduce memory usage.
class ProtocolInterpreter
{
	public:

		ProtocolInterpreter(Platform *p, Webserver *ws);
		virtual void ConnectionEstablished() {}
		virtual void ConnectionLost(uint16_t local_port) {}
		virtual bool CharFromClient(const char c) = 0;
		virtual void ResetState() = 0;

	    virtual bool StoreUploadData(const char* data, unsigned int len);
	    virtual bool FlushUploadData();
	    void CancelUpload();
		bool IsUploading() const { return uploadState != notUploading; }

		virtual ~ProtocolInterpreter() { }					// to keep Eclipse happy

	protected:

	    bool gotPassword;
	    Platform *platform;
	    Webserver *webserver;

	    // Information for file uploading
	    enum UploadState
	    {
			notUploading,									// no upload in progress
			uploadOK,										// upload in progress, no error so far
			uploadError										// upload in progress but had error
	    };

	    UploadState uploadState;
	    FileData fileBeingUploaded;
	    char filenameBeingUploaded[maxFilenameLength + 1];
	    const char *uploadPointer;							// pointer to start of uploaded data not yet written to file
	    unsigned int uploadLength;							// amount of data not yet written to file

	    bool StartUpload(FileStore *file);
	    void FinishUpload(const long file_length);
};

class Webserver
{   
  public:

    Webserver(Platform* p);
    void Init();
    void Spin();
    void Exit();
    void Diagnostics();

    void SetPassword(const char* pw);
    void SetName(const char* nm);
    const char *GetName() const;
    bool CheckPassword(const char* pw) const;

    bool GCodeAvailable();
    char ReadGCode();

    void ConnectionLost(const ConnectionState *cs);
    void SetReadingConnection();
    void ConnectionError();
    void WebDebug(bool wdb);

    friend class Platform;

  protected:

    void MessageStringToWebInterface(const char *s, bool error, bool finished = true);
    void AppendReplyToWebInterface(const char* s, bool error, bool finished = true);

  private:

	class HttpInterpreter : public ProtocolInterpreter
	{
		public:

			HttpInterpreter(Platform *p, Webserver *ws);
			bool CharFromClient(const char c);
			void ResetState();

			bool FlushUploadData();
			void ReceivedGcodeReply();
			void SetDebug(bool b) { webDebug = b; }

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
			void SendJsonResponse(const char* command);
			bool GetJsonResponse(const char* request, const char* key, const char* value, size_t valueLength);
			void GetJsonUploadResponse();
			void GetStatusResponse(uint8_t type);
			bool ProcessMessage();
			bool RejectMessage(const char* s, unsigned int code = 500);
			void JsonReport(bool ok, const char* request);

			HttpState state;

//			bool receivingPost;
//			int boundaryCount;
//			FileStore* postFile;
//			bool postSeen;
//			bool getSeen;
//			bool clientLineIsBlank;

			// Buffers for processing HTTP input
			char clientMessage[webMessageLength];			// holds the command, qualifier, and headers
//			char postBoundary[postBoundaryLength];			// holds the POST boundary string
//			char postFileName[postFilenameLength];			// holds the POST filename
//			char postData[postDataLength];			    	// holds the POST data
			unsigned int clientPointer;						// current index into clientMessage

			const char* commandWords[maxCommandWords];
			KeyValueIndices qualifiers[maxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
			KeyValueIndices headers[maxHeaders];			// offsets into clientHeader of the key/value pairs
			unsigned int numCommandWords;
			unsigned int numQualKeys;						// number of qualifier keys we have found, <= maxQualKeys
			unsigned int numHeaderKeys;						// number of keys we have found, <= maxHeaders

			// Buffers to hold reply
			char jsonResponse[jsonReplyLength];
			char decodeChar;
			uint16_t seq;									// reply sequence number, so that the client can tell if a json reply is new or not
		    bool webDebug;
	};
	HttpInterpreter *httpInterpreter;

	class FtpInterpreter : public ProtocolInterpreter
	{
		public:

			FtpInterpreter(Platform *p, Webserver *ws);
			void ConnectionEstablished();
			void ConnectionLost(uint16_t local_port);
			bool CharFromClient(const char c);
			void ResetState();

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

			char currentDir[maxFilenameLength];
			char filename[maxFilenameLength];

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

			TelnetInterpreter(Platform *p, Webserver *ws);
			void ConnectionEstablished();
			void ConnectionLost(uint16_t local_port);
			bool CharFromClient(const char c);
			void ResetState();

			void HandleGcodeReply(const char *reply);

		private:

			enum TelnetState
			{
				authenticating,			// not logged in
				authenticated			// logged in
			};
			TelnetState state;

			char clientMessage[telnetMessageLength];
			unsigned int clientPointer;

			void ProcessLine();
	};
	TelnetInterpreter *telnetInterpreter;

	// G-Code processing
    unsigned int GetGcodeBufferSpace() const;
    void ProcessGcode(const char* gc);
    void LoadGcodeBuffer(const char* gc);
    void StoreGcodeData(const char* data, size_t len);

    // File info methods
    bool GetFileInfo(const char *fileName, unsigned long& length, float& height, float& filamentUsed, float& layerHeight, char* generatedBy, size_t generatedByLength);
    static bool FindHeight(const char* buf, size_t len, float& height);
    static bool FindFilamentUsed(const char* buf, size_t len, float& filamentUsed);
    static void CopyParameterText(const char* src, char *dst, size_t length);

    // Buffer to hold gcode that is ready for processing
    char gcodeBuffer[gcodeBufLength];
    unsigned int gcodeReadIndex, gcodeWriteIndex;	// head and tail indices into gcodeBuffer
    char gcodeReply[STRING_LENGTH + 1];

    // Misc
    char password[SHORT_STRING_LENGTH + 1];
    char myName[SHORT_STRING_LENGTH + 1];

    Platform* platform;
    bool webserverActive;
    const ConnectionState *readingConnection;

    float lastTime;
    float longWait;
};


#endif
