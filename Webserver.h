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
#define POST_LENGTH				(1300)			// max amount of POST data we can accept

const unsigned int gcodeBufLength = 2048;		// size of our gcode ring buffer, ideally a power of 2
const unsigned int minReportedFreeBuf = 100;	// the minimum free buffer we report if not zero
const unsigned int maxReportedFreeBuf = 900;	// the max we own up to having free, to avoid overlong messages

class Webserver
{   
  public:
  
    Webserver(Platform* p);
    bool GCodeAvailable();						// Called by G Codes to see if the webserver has received one
    byte ReadGCode();							// Return the next byte of a G Code - they are read one by one
//    bool WebserverIsWriting();
    void Init();								// Set it all up
    void Spin();								// Called in a loop to monitor the web input and send output
    void Exit();								// Shut down
    void Diagnostics();							// Print useful stuff
    void SetPassword(const char* pw);			// Set the password to the string
    void SetName(const char* nm);				// Set the name of this RepRap machine
    void ConnectionError();						// Called when a connection is lost to tidy up
    void HandleReply(const char *s, bool error);// Called to send the string as a reply to a G Code
    void AppendReply(const char* s);			// Stick s on the end of a reply being constructed

  private:
  
    void ParseClientLine();						// Parse an HTTP line
    void SendFile(const char* nameOfFileToSend);// Send a file to the client
    bool WriteBytes();							// Send some bytes to the network
//    void ParseQualifier();
    void CheckPassword();						// Has the user remembered?
    void LoadGcodeBuffer(const char* gc, bool convertWeb);  // Put incoming G Code(s) from the client into the buffer
    void CloseClient();							// Bye!
//    bool PrintHeadString();						//
//    bool PrintLinkTable();
//    void GetGCodeList();
    void GetJsonResponse(const char* request);	// Compose a response to a JSON request from the client
    void ParseGetPost();
    bool CharFromClient(char c);				// Deal with a byte from the client
    void BlankLineFromClient();					// The client has sent a blank line.  This means its request is finished.
    void InitialisePost();						// Start receiving a post from the client
    bool MatchBoundary(char c);					// Have we got to the end of the post?
    void JsonReport(bool ok, const char* request); // Output debugging information
    unsigned int GetGcodeBufferSpace() const;	// How much space is really left in the buffer?
    unsigned int GetReportedGcodeBufferSpace() const; // How much buffer space are we going to admit to (less than above).
    void ProcessGcode(const char* gc);			// Some special GCodes are intercepted by the webserver before they get to the GCode class for wmergencies etc.

    Platform* platform;
    bool active;
    float lastTime;
    float longWait;
    FileStore* fileBeingSent;
    bool writing;
    bool receivingPost;
    char postBoundary[POST_LENGTH];
    int boundaryCount;  
    char postFileName[POST_LENGTH];
    FileStore* postFile;
    bool postSeen;
    bool getSeen;
    bool clientLineIsBlank;
    float clientCloseTime;
    bool needToCloseClient;

    char clientLine[STRING_LENGTH+2];	// 2 chars extra so we can append \n\0
    char clientRequest[STRING_LENGTH];
    char clientQualifier[STRING_LENGTH];
    char jsonResponse[STRING_LENGTH+1];
    char gcodeBuffer[gcodeBufLength];
    unsigned int gcodeReadIndex, gcodeWriteIndex;		// head and tail indices into gcodeBuffer
    int jsonPointer;
    int clientLinePointer;
    bool gotPassword;
    char password[SHORT_STRING_LENGTH+1];
    char myName[SHORT_STRING_LENGTH+1];
    char gcodeReply[STRING_LENGTH+1];
    uint16_t seq;	// reply sequence number, so that the client can tell if a reply is new or not
};


#endif
