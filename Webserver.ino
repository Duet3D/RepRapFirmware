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

#include "RepRapFirmware.h"

Webserver::Webserver(Platform* p)
{
  Serial.println("Webserver constructor"); 
  platform = p;
  lastTime = platform->time();
  writing = false;
  clientLineIsBlank = true;
  needToCloseClient = false;
  clientLinePointer = 0;
  clientLine[0] = 0;
  clientRequest[0] = 0;
}

boolean Webserver::fileHasExtension(char* fileName, char* extension)
{
  int j = strlen(fileName);
  int k = strlen(extension);
  if(k > j)
    return false;
  
  return(!strcmp(&fileName[j - k], extension));
}

void Webserver::SendFile(char* nameOfFileToSend)
{
    platform->SendToClient("HTTP/1.1 200 OK\n");
    if(fileHasExtension(nameOfFileToSend, ".png"))
      platform->SendToClient("Content-Type: image/png\n");
    else
      platform->SendToClient("Content-Type: text/html\n");
    platform->SendToClient("Connnection: close\n");
//          if(loadingImage)
//          {
//           platform->SendToHost("Cache-Control: max-age=3600\n");
//           Serial.println("Image requested");
//          }
    platform->SendToClient('\n');
    //Serial.print("File requested: ");
    //Serial.println(nameOfFileToSend);
    fileBeingSent = platform->OpenFile(nameOfFileToSend, false);
    if(fileBeingSent < 0)
        fileBeingSent = platform->OpenFile("html404.htm", false);
    writing = true; 
}

void Webserver::WriteByte()
{
    unsigned char b;
    if(platform->Read(fileBeingSent, b))
      platform->SendToClient(b);
    else
    {     
      platform->Close(fileBeingSent);    
      writing = false;
      clientCloseTime = platform->time();
      needToCloseClient = true; 
    }  
}

void Webserver::CheckClientLine()
{   
  if(!(clientLine[0] == 'G' && clientLine[1] == 'E' && clientLine[2] == 'T'))
    return;
    
  int i = 5;
  int j = 0;
  clientRequest[j] = 0;
  while(clientLine[i] != ' ')
  {
    clientRequest[j] = clientLine[i];
    j++;
    i++;
  }
  clientRequest[j] = 0;
  if(!clientRequest[0])
    strcpy(clientRequest, "index.htm");
}


void Webserver::spin()
{
  if(writing)
  {
    WriteByte();
    return;         
  }
  
  if(platform->ClientStatus() & CONNECTED)
  {
    if (platform->ClientStatus() & AVAILABLE) 
    {
      char c = platform->ClientRead();
      //Serial.write(c);
      // if you've gotten to the end of the line (received a newline
      // character) and the line is blank, the http request has ended,
      // so you can send a reply
      if (c == '\n' && clientLineIsBlank) 
      {
        clientLine[clientLinePointer] = 0;
        clientLinePointer = 0;
        SendFile(clientRequest);
        clientRequest[0] = 0;
        return;
      }
      if (c == '\n') 
      {
        clientLine[clientLinePointer] = 0;
        CheckClientLine();
          // you're starting a new line
        clientLineIsBlank = true;
        clientLinePointer = 0;
      } else if (c != '\r') 
      {
        // you've gotten a character on the current line
        clientLineIsBlank = false;
        clientLine[clientLinePointer] = c;
        clientLinePointer++;
      }
    }
    //return;
  }  
   
  if (platform->ClientStatus() & CLIENT) 
  {
    if(needToCloseClient)
    {
      if(platform->time() - clientCloseTime < CLIENT_CLOSE_DELAY)
        return;
      needToCloseClient = false;  
      platform->DisconnectClient();
    }   
  }
}






