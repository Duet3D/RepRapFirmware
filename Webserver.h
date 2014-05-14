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

const unsigned int postLength = 1400;			// max amount of POST data we can accept
const unsigned int webInputLength = 1400;		// max size of web interface requests and related stuff
const unsigned int gcodeBufLength = 2048;		// size of our gcode ring buffer, ideally a power of 2
const unsigned int minReportedFreeBuf = 100;	// the minimum free buffer we report if not zero
const unsigned int maxReportedFreeBuf = 1024;	// the max we own up to having free, to avoid overlong messages
const unsigned int jsopnReplyLength = 1200;		// size of buffer used to hold JSON reply

class Webserver
{   
  public:
  
    Webserver(Platform* p);
    bool GCodeAvailable();
    byte ReadGCode();
    bool WebserverIsWriting();
    void Init();
    void Spin();
    void Exit();
    void Diagnostics();
    void SetPassword(const char* pw);
    void SetName(const char* nm);
    void HandleReply(const char *s, bool error);
    void AppendReply(const char* s);

  private:
  
    void ParseClientLine();
    void SendFile(const char* nameOfFileToSend);
    void ParseQualifier();
    void CheckPassword();
    void LoadGcodeBuffer(const char* gc);
    bool PrintHeadString();
    bool PrintLinkTable();
    void GetGCodeList();
    void GetJsonResponse(const char* request);
    void GetStatusResponse(uint8_t type);
    void ParseGetPost();
    bool CharFromClient(char c);
    void BlankLineFromClient();
    void InitialisePost();
    bool MatchBoundary(char c);
    void JsonReport(bool ok, const char* request);
    unsigned int GetGcodeBufferSpace() const;
    unsigned int GetReportedGcodeBufferSpace() const;
    void ProcessGcode(const char* gc);
    bool GetFileInfo(const char *fileName, unsigned long& length, float& height, float& filamentUsed);
    static bool FindHeight(const char* buf, size_t len, float& height);
    static bool FindFilamentUsed(const char* buf, size_t len, float& filamentUsed);

    Platform* platform;
    bool active;
    float lastTime;
    float longWait;
    bool receivingPost;
    char postBoundary[postLength];
    int boundaryCount;  
    char postFileName[postLength];
    FileStore* postFile;
    bool postSeen;
    bool getSeen;
    bool clientLineIsBlank;

    char clientLine[webInputLength];
    char clientRequest[webInputLength];
    char clientQualifier[webInputLength];
    char jsonResponse[jsopnReplyLength];
    char gcodeBuffer[gcodeBufLength];
    unsigned int gcodeReadIndex, gcodeWriteIndex;		// head and tail indices into gcodeBuffer
    int clientLinePointer;
    bool gotPassword;
    char password[SHORT_STRING_LENGTH+1];
    char myName[SHORT_STRING_LENGTH+1];
    char gcodeReply[STRING_LENGTH+1];
    uint16_t seq;	// reply sequence number, so that the client can tell if a json reply is new or not
};


#endif
