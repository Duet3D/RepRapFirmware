/****************************************************************************************************

RepRapFirmware - Webserver

This class serves web pages to the attached network.  These pages form the user's interface with the
RepRap machine.  It interprests returned values from those pages and uses them to Generate G Codes,
which it sends to the RepRap.  It also collects values from the RepRap like temperature and uses
those to construct the web pages.

It implements very very restricted PHP.  It can do:

   <?php print(myStringFunction()); ?>
   <?php if(myBooleanFunction()) print(myOtherStringFunction()); ?>
   <?php if(myOtherBooleanFunction()) echo 'Some arbitrarily long string of HTML including newlines up to this quote:'; ?>

Note that by printing a function that returns "" you can just call 
that function in this C++ code with no effect on the loaded web page.

-----------------------------------------------------------------------------------------------------

Version 0.1

13 February 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef WEBSERVER_H
#define WEBSERVER_H


class Webserver
{   
  public:
  
    Webserver(Platform* p);
    boolean Available();
    byte Read();
    void Init();
    void Spin();
    void Exit();
    
  private:
  
    void ParseClientLine();
    void SendFile(char* nameOfFileToSend);
    void WriteByte();
    boolean StringEndsWith(char* string, char* ending);
    boolean StringStartsWith(char* string, char* starting);
    boolean StringEquals(char* s1, char* s2);
    void ParseQualifier();
    void CheckPassword();
    boolean LoadGcodeBuffer(char* gc, boolean convertWeb);
    void CloseClient();
    void InitialisePHP();
    char PHPParse(char* phpString);
    boolean PrintHeadString();
    boolean PrintLinkTable();
    void GetGCodeList();
    boolean CallPHPBoolean(char* phpRecord);
    void CallPHPString(char* phpRecord);  
    void ProcessPHPByte(char b);
    void WritePHPByte();
    void ParseGetPost();
    void CharFromClient(char c);
    void BlankLineFromClient();
    void InitialisePost();
    int StringContains(char* string, char* match);
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
    //long postLength;
    boolean inPHPFile;
    boolean clientLineIsBlank;
    unsigned long clientCloseTime;
    boolean needToCloseClient;

    char clientLine[STRING_LENGTH];
    char clientRequest[STRING_LENGTH];
    char clientQualifier[STRING_LENGTH];
    char gcodeBuffer[GCODE_LENGTH];
    boolean gcodeAvailable;
    int gcodePointer;
    int clientLinePointer;
    boolean gotPassword;
    char* password;
    char* myName;
    char phpTag[PHP_TAG_LENGTH];
    char phpRecord[PHP_TAG_LENGTH];
    int inPHPString;
    int phpPointer;
    boolean phpEchoing;
    boolean phpIfing;
    boolean phpPrinting;
    boolean eatInput;
    boolean recordInput;
    boolean ifWasTrue;
    boolean sendTable;
    char eatInputChar;
    int phpRecordPointer;
};


#endif
