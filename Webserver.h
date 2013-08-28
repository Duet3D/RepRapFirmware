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


#define INDEX_PAGE "reprap.htm"
#define MESSAGE_FILE "messages.txt"
#define FOUR04_FILE "html404.htm"
#define KO_START "rr_"
#define KO_FIRST 3
#define POST_LENGTH 200
#define GCODE_LENGTH 100 // Maximum lenght of internally-generated G Code string

class Webserver
{   
  public:
  
    Webserver(Platform* p);
    bool GCodeAvailable();
    byte ReadGCode();
    void Init();
    void Spin();
    void Exit();
    void Diagnostics();
    
  private:
  
    void ParseClientLine();
    void SendFile(char* nameOfFileToSend);
    void WriteByte();
    void ParseQualifier();
    void CheckPassword();
    bool LoadGcodeBuffer(char* gc, bool convertWeb);
    void CloseClient();
    bool PrintHeadString();
    bool PrintLinkTable();
    void GetGCodeList();
    void GetJsonResponse(char* request);
    void ParseGetPost();
    void CharFromClient(char c);
    void BlankLineFromClient();
    void InitialisePost();
    bool MatchBoundary(char c);
    void JsonReport(bool ok, char* request);

    
    Platform* platform;
    bool active;
    float lastTime;
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
    unsigned long clientCloseTime;
    bool needToCloseClient;

    char clientLine[STRING_LENGTH];
    char clientRequest[STRING_LENGTH];
    char clientQualifier[STRING_LENGTH];
    char jsonResponse[STRING_LENGTH];
    char gcodeBuffer[GCODE_LENGTH];
    int jsonPointer;
    bool gcodeAvailable;
    int gcodePointer;
    int clientLinePointer;
    bool gotPassword;
    char* password;
    char* myName;
};


#endif
