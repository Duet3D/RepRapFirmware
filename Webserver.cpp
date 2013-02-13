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
}

void Webserver::spin()
{

}


/*

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,1,9);

// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
EthernetServer server(80);

#define MAX_FILES 5

#define null 0

File files[MAX_FILES];
bool inUse[MAX_FILES];

char line[1000];
char page[1000];
int lp;

void setup() // Thanks to SurferTim: http://arduino.cc/forum/index.php?action=profile;u=49379
{
  lp = 0;
  for(int i=0; i < MAX_FILES; i++)
    inUse[i] = false;

  Serial.begin(9600);

  // disable SD SPI while starting w5100
  // or you will have trouble
  pinMode(4, OUTPUT);
  digitalWrite(4,HIGH);   

  Ethernet.begin(mac, ip);
  server.begin();
  
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
  
  // this corrects a bug in the Ethernet.begin() function
  // even tho the call to Ethernet.localIP() does the same thing
  digitalWrite(10,HIGH);
 
  if (!SD.begin(4)) 
     Serial.println("SD initialization failed.");
  // SD.begin() returns with the SPI disabled, so you need not disable it here  
}



void error(char* s)
{
  Serial.println(s); 
}

void comment(char* s)
{
  Serial.println(s); 
}

// Open a local file (for example on an SD card).

int OpenFile(char* fileName, bool write)
{
  int result = -1;
  for(int i=0; i < MAX_FILES; i++)
    if(!inUse[i])
    {
      result = i;
      break;
    }
  if(result < 0)
  {
      error("Max open file count exceeded.");
      return -1;    
  }
  
  if(!SD.exists(fileName))
  {
    if(!write)
    {
      error("File not found for reading");
      return -1;
    }
    files[result] = SD.open(fileName, FILE_WRITE);
  } else
  {
    if(write)
      files[result] = SD.open(fileName, FILE_WRITE);
    else
      files[result] = SD.open(fileName, FILE_READ);
  }

  inUse[result] = true;
  return result;
}

void Close(int file)
{
    files[file].close();
    inUse[file] = false;
}

bool Read(int file, unsigned char* b)
{
  if(!files[file].available())
    return false;
  *b = (unsigned char) files[file].read();
  return true;
}


bool parseLine()
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


void loop() 
{
  int htmlFile;
  bool loadingImage;
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
*/

