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
  
  lp = 0;
}

void error(char* s)
{
  Serial.println(s); 
}

void comment(char* s)
{
  Serial.println(s); 
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


void Webserver::spin()
{
  int htmlFile;
  boolean loadingImage;
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) 
  {
    comment("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    loadingImage = false;
    while (client.connected()) 
    {
      if (client.available()) 
      {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) 
        {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connnection: close");
//          if(loadingImage)
//          {
//           client.println("Cache-Control: max-age=3600");
//           Serial.println("Image requested");
//          }
          client.println();
      
          htmlFile = OpenFile(page,false);
          unsigned char b;
          while(Read(htmlFile, &b))
            client.write(b);
          Close(htmlFile);
          
          page[0] = 0;
          lp = 0;
          break;
        }
        if (c == '\n') 
        {
          line[lp] = 0;
          if(parseLine())
            loadingImage = true;
          // you're starting a new line
          currentLineIsBlank = true;
          lp = 0;
        } else if (c != '\r') 
        {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
          line[lp]=c;
          lp++;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}




