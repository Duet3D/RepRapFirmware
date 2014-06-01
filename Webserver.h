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


#define KO_START "rr_"
#define KO_FIRST 3

const unsigned int gcodeBufLength = 512;		// size of our gcode ring buffer, preferably a power of 2
const unsigned int uploadBufLength = 4096;		// size of our file upload buffer, preferably a power of 2
const unsigned int maxReportedFreeBuf = 2048;	// the max we own up to having free, to avoid overlong messages

const unsigned int webMessageLength = 3000;		// maximum length of the web message we accept after decoding, excluding POST data.
												// needs to be maxReportedFreeBuf + lots more to hold the HTTP headers.
const unsigned int maxFilenameLength = 100;		// maximum length of a filename (inc. path from root) that we can upload
//const unsigned int postBoundaryLength = 100;	// max length of the POST boundary string
//const unsigned int postFilenameLength = 100;	// max length of the POST filename
//const unsigned int postDataLength = 1000;		// max amount of POST data

const unsigned int maxCommandWords = 4;			// max number of space-separated words in the command
const unsigned int maxQualKeys = 5;				// max number of key/value pairs in the qualifier
const unsigned int maxHeaders = 10;				// max number of key/value pairs in the headers

const unsigned int jsonReplyLength = 1000;		// size of buffer used to hold JSON reply

class Webserver
{   
  public:
  
    Webserver(Platform* p);
    bool GCodeAvailable();
    char ReadGCode();
    void Init();
    void Spin();
    void Exit();
    void Diagnostics();
    void SetPassword(const char* pw);
    void SetName(const char* nm);
    void HandleReply(const char *s, bool error);
    void AppendReply(const char* s);
    void ResetState(const HttpState *connection);

  private:

    // Server state enumeration. The order is important, in particular xxxEsc1 must follow xxx, and xxxEsc2 must follow xxxEsc1.
    // We assume that qualifier keys do not contain escapes, because none of ours needs to be encoded. If we are sent escapes in the key,
    // it won't do any harm, but the key won't be recognised even if it would be valid were it decoded.
    enum ServerState
    {
    	inactive,					// not up and running
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

    enum UploadState
    {
    	notUploading,				// no upload in progress
    	uploadOK,					// upload in progress, no error so far
    	uploadError					// upload in progress but had error
    };

    struct KeyValueIndices
    {
    	const char* key;
    	const char* value;
    };
  
    void CancelUpload();
    void SendFile(const char* nameOfFileToSend);
    void SendJsonResponse(const char* command);
    void CheckPassword(const char* pw);
    void LoadGcodeBuffer(const char* gc);
    void StoreGcodeData(const char* data, size_t len);
    bool GetJsonResponse(const char* request, const char* key, const char* value, size_t valueLength);
    void GetJsonUploadResponse();
    void GetStatusResponse(uint8_t type);
    bool CharFromClient(char c);
    bool ProcessMessage();
    bool RejectMessage(const char* s, unsigned int code = 500);
    void JsonReport(bool ok, const char* request);
    unsigned int GetGcodeBufferSpace() const;
    unsigned int GetReportedGcodeBufferSpace() const;
    unsigned int GetUploadBufferSpace() const;
    unsigned int GetReportedUploadBufferSpace() const;
    void ProcessGcode(const char* gc);
    bool GetFileInfo(const char *fileName, unsigned long& length, float& height, float& filamentUsed, float& layerHeight, char* generatedBy, size_t generatedByLength);
    static bool FindHeight(const char* buf, size_t len, float& height);
    static bool FindFilamentUsed(const char* buf, size_t len, float& filamentUsed);

    Platform* platform;

    // Web server state machine data
    ServerState state;
    char decodeChar;
    const HttpState *currentConnection;

    float lastTime;
    float longWait;

    // Buffers for processing HTTP input
    char clientMessage[webMessageLength];			// holds the command, qualifier, and headers
//  char postBoundary[postBoundaryLength];			// holds the POST boundary string
//  char postFileName[postFilenameLength];			// holds the POST filename
//  char postData[postDataLength];					// holds the POST data
    unsigned int clientPointer;						// current index into clientMessage

    const char* commandWords[maxCommandWords];
    KeyValueIndices qualifiers[maxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
    KeyValueIndices headers[maxHeaders];			// offsets into clientHeader of the key/value pairs
    unsigned int numCommandWords;
    unsigned int numQualKeys;						// number of qualifier keys we have found, <= maxQualKeys
    unsigned int numHeaderKeys;						// number of keys we have found, <= maxHeaders

    // Buffer to hold gcode that is ready for processing
    char gcodeBuffer[gcodeBufLength];
    unsigned int gcodeReadIndex, gcodeWriteIndex;	// head and tail indices into gcodeBuffer

    // Information for file uploading
    UploadState uploadState;
    FileData fileBeingUploaded;
    char filenameBeingUploaded[maxFilenameLength + 1];
    const char *uploadPointer;						// pointer to start of uploaded data not yet written to file
    unsigned int uploadLength;						// amount of data not yet written to file

    // Buffers to hold reply
    char jsonResponse[jsonReplyLength];
    char gcodeReply[STRING_LENGTH+1];
    uint16_t seq;	// reply sequence number, so that the client can tell if a json reply is new or not

    // Misc
    bool gotPassword;
    char password[SHORT_STRING_LENGTH+1];
    char myName[SHORT_STRING_LENGTH+1];
};


#endif
