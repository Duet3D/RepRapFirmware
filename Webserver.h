/****************************************************************************************************

RepRapFirmware - Webserver

This class serves web pages to the attached network.  These pages form the user's interface with the
RepRap machine.  It interprests returned values from those pages and uses them to Generate G Codes,
which it sends to the RepRap.  It also collects values from the RepRap like temperature and uses
those to construct the web pages.

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

#define CLIENT_CLOSE_DELAY 1000 // Microseconds to wait after serving a page

class Webserver
{   
  public:
  
    Webserver(Platform* p);
    void spin();
    
  private:
  
    void CheckClientLine();
    void IncomingByte(unsigned char b);
    void SendFile(char* nameOfFileToSend);
    void WriteByte();
    boolean fileHasExtension(char* fileName, char* extension);
    
    Platform* platform;
    unsigned long lastTime;
    int fileBeingSent;
    boolean writing;
    boolean clientLineIsBlank;
    unsigned long clientCloseTime;
    boolean needToCloseClient;
    char clientLine[1000];
    char clientRequest[1000];
    int clientLinePointer;
};


#endif
