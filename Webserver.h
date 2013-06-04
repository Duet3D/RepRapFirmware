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
    boolean GCodeAvailable();
    byte ReadGCode();
    void Init();
    void Spin();
    void Exit();
    
  private:
  
    void ParseClientLine();
    void SendFile(char* nameOfFileToSend);
    void WriteByte();
    void ParseQualifier();
    void CheckPassword();
    boolean LoadGcodeBuffer(char* gc, boolean convertWeb);
    void CloseClient();
    boolean PrintHeadString();
    boolean PrintLinkTable();
    void GetGCodeList();
    void GetJsonResponse(char* request);
    void ParseGetPost();
    void CharFromClient(char c);
    void BlankLineFromClient();
    void InitialisePost();
    boolean MatchBoundary(char c);
    
    Platform* platform;
    boolean active;
    unsigned long lastTime;
    int fileBeingSent;
    boolean writing;
    boolean receivingPost;
    char postBoundary[POST_LENGTH];
    int boundaryCount;  
    char postFileName[POST_LENGTH];
    int postFile;
    boolean postSeen;
    boolean getSeen;
    boolean clientLineIsBlank;
    unsigned long clientCloseTime;
    boolean needToCloseClient;

    char clientLine[STRING_LENGTH];
    char clientRequest[STRING_LENGTH];
    char clientQualifier[STRING_LENGTH];
    char jsonResponse[STRING_LENGTH];
    char gcodeBuffer[GCODE_LENGTH];
    int jsonPointer;
    boolean gcodeAvailable;
    int gcodePointer;
    int clientLinePointer;
    boolean gotPassword;
    char* password;
    char* myName;
};


#endif
