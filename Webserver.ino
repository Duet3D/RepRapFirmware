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
  client = platform->OpenHost();
  lp = 0;
  writing = false;
  loadingImage = false;
  currentLineIsBlank = false;
  needToCloseClient = false;
}


boolean Webserver::parseLine()
{
  if(!(line[0] == 'G' && line[1] == 'E' && line[2] == 'T'))
    return false;
  int i = 5;
  int j = 0;
  while(line[i] != ' ')
  {
    page[j] = line[i];
    j++;
    i++;
  }
  page[j] = 0;
  if(!page[0])
    strcpy(page, "index.htm");
  j = strlen(page);
  return(page[j-3] == 'g' && page[j-2] == 'i' && page[j-1] == 'f');
}

void Webserver::SendFile()
{
    platform->WriteString(client, "HTTP/1.1 200 OK\n");
    platform->WriteString(client, "Content-Type: text/html\n");
    platform->WriteString(client, "Connnection: close\n");
//          if(loadingImage)
//          {
//           platform->WriteString(client, "Cache-Control: max-age=3600\n");
//           Serial.println("Image requested");
//          }
    platform->Write(client, '\n');
    htmlFile = platform->OpenFile(page,false);
    page[0] = 0;
    lp = 0;    
    writing = true; 
}

void Webserver::Write()
{
    unsigned char b;
    if(platform->Read(htmlFile, b))
      platform->Write(client, b);
    else
    {     
      platform->Close(htmlFile);    
      writing = false;
      clientCloseTime = platform->time() + 1000;
      needToCloseClient = true;
    }  
}


void Webserver::Read(unsigned char b)
{
  Serial.write(b);
  // if you've gotten to the end of the line (received a newline
  // character) and the line is blank, the http request has ended,
  // so you can send a reply
  if (b == '\n' && currentLineIsBlank) 
    SendFile();
    
  if (b == '\n') 
  {
    line[lp] = 0;
    if(parseLine())
      loadingImage = true;
    // you're starting a new line
    currentLineIsBlank = true;
    lp = 0;
  } else if (b != '\r') 
  {
    // you've gotten a character on the current line
    currentLineIsBlank = false;
    line[lp]=b;
    lp++;
  } 
}



void Webserver::spin()
{
  unsigned char b; 
  
  if(platform->Read(client, b)) 
    Read(b);
  
  if(writing) 
    Write();
  
  if(needToCloseClient)
  {
    if(platform->time() - clientCloseTime > 0)
    {
      platform->Close(client); // NB this stops the connection, but we can still read from client.
      needToCloseClient = false;
    }
  }
}



