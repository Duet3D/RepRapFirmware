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
  //Serial.println("Webserver constructor"); 
  platform = p;
  lastTime = platform->time();
  writing = false;
  clientLineIsBlank = true;
  needToCloseClient = false;
  clientLinePointer = 0;
  clientLine[0] = 0;
  clientRequest[0] = 0;
  password = DEFAULT_PASSWORD;
  myName = DEFAULT_NAME;
  gotPassword = false;
  gcodeAvailable = false;
  gcodePointer = 0;
}

boolean Webserver::Available()
{
  return gcodeAvailable;
}

byte Webserver::Read()
{
  byte c = gcodeBuffer[gcodePointer];
  if(!c)
  {
     gcodeAvailable = false;
     gcodePointer = 0;
     gcodeBuffer[gcodePointer] = 0;
  } else
    gcodePointer++;
  return c;
}


boolean Webserver::LoadGcodeBuffer(char* gc, boolean convertWeb)
{
  if(gcodeAvailable)
    return false;
  
  if(strlen(gc) > GCODE_LENGTH-1)
  {
    platform->Message(HOST_MESSAGE, "Webserver: GCode buffer overflow.\n");
    return false;  
  }
  
  int gcp = 0;
  gcodePointer = 0;
  gcodeBuffer[gcodePointer] = 0;
  
  char c;
  
  while(c = gc[gcp++])
  {
    if(c == '+' && convertWeb)
      c = ' ';
    if(c == '%'&& convertWeb) // FIXME - just convert the next two hex bytes to a char
    {
      if(StringStartsWith(&gc[gcp], "2B"))
        c = '+';
      else if(StringStartsWith(&gc[gcp], "20"))
        c = ' ';
      else if(StringStartsWith(&gc[gcp], "0A"))
        c = '\n';
      else
      {
        platform->Message(HOST_MESSAGE, "Webserver: Dud web-form byte: ");
        platform->Message(HOST_MESSAGE, gc);
        platform->Message(HOST_MESSAGE, "\n");
      }
      gcp += 2;
      
    }
    gcodeBuffer[gcodePointer++] = c;
  }
  
  gcodeBuffer[gcodePointer] = 0;
  gcodePointer = 0;  
  gcodeAvailable = true;
  
  return true;
}

boolean Webserver::StringEndsWith(char* string, char* ending)
{
  int j = strlen(string);
  int k = strlen(ending);
  if(k > j)
    return false;
  
  return(!strcmp(&string[j - k], ending));
}

boolean Webserver::StringStartsWith(char* string, char* starting)
{ 
  int j = strlen(string);
  int k = strlen(starting);
  if(k > j)
    return false;
  
  for(int i = 0; i < k; i++)
    if(string[i] != starting[i])
      return false;
      
  return true;
}


void Webserver::InternalHead(boolean sendTab, int noLink, char* headString)
{
  platform->SendToClient("<!DOCTYPE HTML><head>");
  if(headString)
    platform->SendToClient(headString);
  platform->SendToClient("</head><html><h2>RepRap: ");
  platform->SendToClient(myName);
  if(gotPassword)
    platform->SendToClient("&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<a href=\"http://reprappro.com\" target=\"_blank\"><img src=\"logo.png\" alt=\"RepRapPro logo\"></a>");
  platform->SendToClient("</h2><br><br>");
  if(sendTab)
  {
    platform->SendToClient("<table><tr>");  //  border=\"1\"
    
    if(noLink == 0)
      platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;Control&nbsp;&nbsp;&nbsp;</td>");
    else
      platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;<a href=\"control.htm\">Control</a>&nbsp;&nbsp;&nbsp;</td>");

    if(noLink == 1)      
      platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;Print&nbsp;&nbsp;&nbsp;</td>");
    else
      platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;<a href=\"print.htm\">Print</a>&nbsp;&nbsp;&nbsp;</td>");
 
    platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;<a href=\"http://reprap.org/wiki/RepRapPro_RepRap_Firmware\" target=\"_blank\">Help</a>&nbsp;&nbsp;&nbsp;</td>");
 
    if(noLink == 3)     
      platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;Settings&nbsp;&nbsp;&nbsp;</td>");
    else
      platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;<a href=\"settings.htm\">Settings</a>&nbsp;&nbsp;&nbsp;</td>");
 
    platform->SendToClient("<td>&nbsp;&nbsp;&nbsp;<a href=\"logout.htm\">Logout</a>&nbsp;&nbsp;&nbsp;</td>");
      
    platform->SendToClient("</tr></table>");
  }
  platform->SendToClient("<br><br>");
}

void Webserver::CloseClient()
{
  writing = false;
  clientCloseTime = platform->time();
  needToCloseClient = true;   
}

void Webserver::InternalTail()
{
  platform->SendToClient("<br><br></html>\n");    
  CloseClient();
}

void Webserver::SendControlPage()
{
   InternalHead(true, 0, "<style type=\"text/css\">td { text-align: center; } </style>");
   
   platform->SendToClient("<table border=\"1\"><div align=\"center\">");
   
   
   platform->SendToClient("<tr>");
   platform->SendToClient("<th colspan=\"9\">Move X Y Z</th>");
   platform->SendToClient("</tr>");
   
   platform->SendToClient("<tr>");
   platform->SendToClient("<td rowspan=\"2\"><button type=\"button\" onclick=\"return homea()\">Home<br>All</button></td>");
   platform->SendToClient("<td colspan=\"4\">- mm</td>");
   platform->SendToClient("<td colspan=\"4\">+ mm</td>");
   platform->SendToClient("</tr>");
   
   platform->SendToClient("<tr>");
   platform->SendToClient("<td>-100</td>");
   platform->SendToClient("<td>-10</td>");
   platform->SendToClient("<td>-1</td>");
   platform->SendToClient("<td>-0.1</td>");
   platform->SendToClient("<td>0.1</td>");
   platform->SendToClient("<td>1</td>");
   platform->SendToClient("<td>10</td>");
   platform->SendToClient("<td>100</td>");
   platform->SendToClient("</tr>");
   
   platform->SendToClient("<tr>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return homex()\">Home X</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xm100mm()\">&lt;- X</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xm10mm()\">&lt;- X</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xm1mm()\">&lt;- X</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xm01mm()\">&lt;- X</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xp01mm()\">X -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xp1mm()\">X -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xp10mm()\">X -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return xp100mm()\">X -&gt;</button></td>");
   platform->SendToClient("</tr>");
   
   platform->SendToClient("<tr>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return homey()\">Home Y</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return ym100mm()\">&lt;- Y</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return ym10mm()\">&lt;- Y</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return ym1mm()\">&lt;- Y</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return ym01mm()\">&lt;- Y</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return yp01mm()\">Y -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return yp1mm()\">Y -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return yp10mm()\">Y -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return yp100mm()\">Y -&gt;</button></td>");
   platform->SendToClient("</tr>");
   
   platform->SendToClient("<tr>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return homez()\">Home Z</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zm100mm()\">&lt;- Z</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zm10mm()\">&lt;- Z</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zm1mm()\">&lt;- Z</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zm01mm()\">&lt;- Z</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zp01mm()\">Z -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zp1mm()\">Z -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zp10mm()\">Z -&gt;</button></td>");
   platform->SendToClient("<td><button type=\"button\" onclick=\"return zp100mm()\">Z -&gt;</button></td>");
   platform->SendToClient("</tr>");
    
   platform->SendToClient("</div></table>");
   
   platform->SendToClient("<br><br><form name=\"input\" action=\"gather.asp\" method=\"get\">Send a G Code: <input type=\"text\" name=\"gcode\"><input type=\"submit\" value=\"Send\"></form>\n");
   
   platform->SendToClient("<script language=\"javascript\" type=\"text/javascript\">");
   
   // FIXME - this lot can be be easily generated by a single function
   
   platform->SendToClient("function homea(){ window.location.href = \"control.htm?gcode=G28\";}");
   platform->SendToClient("function homex(){ window.location.href = \"control.htm?gcode=G28 X0\";}");
   platform->SendToClient("function homey(){ window.location.href = \"control.htm?gcode=G28 Y0\";}");
   platform->SendToClient("function homez(){ window.location.href = \"control.htm?gcode=G28 Z0\";}");
   
   platform->SendToClient("function xp01mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X0.1%0AG90\";}");
   platform->SendToClient("function xp1mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X1%0AG90\";}");
   platform->SendToClient("function xp10mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X10%0AG90\";}");
   platform->SendToClient("function xp100mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X100%0AG90\";}");
   
   platform->SendToClient("function xm01mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X-0.1%0AG90\";}");
   platform->SendToClient("function xm1mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X-1%0AG90\";}");
   platform->SendToClient("function xm10mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X10%0AG90\";}");
   platform->SendToClient("function xm100mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 X100%0AG90\";}");
   
   platform->SendToClient("function yp01mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y0.1%0AG90\";}");
   platform->SendToClient("function yp1mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y1%0AG90\";}");
   platform->SendToClient("function yp10mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y10%0AG90\";}");
   platform->SendToClient("function yp100mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y100%0AG90\";}");
   
   platform->SendToClient("function ym01mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y-0.1%0AG90\";}");
   platform->SendToClient("function ym1mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y-1%0AG90\";}");
   platform->SendToClient("function ym10mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y10%0AG90\";}");
   platform->SendToClient("function ym100mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Y100%0AG90\";}");
   
   platform->SendToClient("function zp01mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z0.1%0AG90\";}");
   platform->SendToClient("function zp1mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z1%0AG90\";}");
   platform->SendToClient("function zp10mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z10%0AG90\";}");
   platform->SendToClient("function zp100mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z100%0AG90\";}");
   
   platform->SendToClient("function zm01mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z-0.1%0AG90\";}");
   platform->SendToClient("function zm1mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z-1%0AG90\";}");
   platform->SendToClient("function zm10mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z10%0AG90\";}");
   platform->SendToClient("function zm100mm(){ window.location.href = \"control.htm?gcode=G91%0AG1 Z100%0AG90\";}");
   
   platform->SendToClient("</script>"); 
   InternalTail();  
}

void Webserver::SendPrintPage()
{
   InternalHead(true, 1, 0);
   platform->SendToClient("Print Page - List files in a table: click one to print it; upload a file; delete a file\n");
   InternalTail();  
}

void Webserver::SendHelpPage()
{
   InternalHead(true, 2, 0);
   platform->SendToClient("Help Page");
   InternalTail();  
}

void Webserver::SendSettingsPage()
{
   InternalHead(true, 3, 0);
   platform->SendToClient("Settings Page - Change things and save the result: machine name, password, e steps/mm etc etc");
   InternalTail();  
}

void Webserver::SendLogoutPage()
{
  gotPassword = false;
  SendPasswordPage(); 
}

void Webserver::SendPasswordPage()
{
   InternalHead(false, -1, 0);
   platform->SendToClient("<form name=\"input\" action=\"gather.asp\" method=\"get\">Password: <input type=\"password\" name=\"pwd\"><input type=\"submit\" value=\"Submit\">");
   InternalTail();  
}

void Webserver::Send404Page()
{
   InternalHead(false, -1, 0);
   platform->SendToClient("<h3><br><br>404 Error: page not found.</h3>");
   InternalTail();  
}

boolean Webserver::InternalFile(char* nameOfFileToSend)
{
  if(StringStartsWith(nameOfFileToSend, "index.htm") ||
    StringStartsWith(nameOfFileToSend, "control.htm"))
  {
    SendControlPage();
    return true; 
  }
  
  if(StringStartsWith(nameOfFileToSend, "print.htm"))
  {
    SendPrintPage();
    return true; 
  }
  
  if(StringStartsWith(nameOfFileToSend, "help.htm"))
  {
    SendHelpPage();
    return true; 
  }
  
  if(StringStartsWith(nameOfFileToSend, "settings.htm"))
  {
    SendSettingsPage();
    return true; 
  }
  
  if(StringStartsWith(nameOfFileToSend, "logout.htm"))
  {
    SendLogoutPage();
    return true; 
  }
  
  if(StringStartsWith(nameOfFileToSend, "passwd.htm"))
  {
    SendPasswordPage();
    return true; 
  }
  
  if(StringStartsWith(nameOfFileToSend, "html404.htm"))
  {
    Send404Page();
    return true; 
  }
  
  return false; 
}

void Webserver::SendFile(char* nameOfFileToSend)
{
  if(!gotPassword)
    nameOfFileToSend = PASSWORD_PAGE;
    
  platform->SendToClient("HTTP/1.1 200 OK\n");
  
  if(StringEndsWith(nameOfFileToSend, ".png"))
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
  
  if(InternalFile(nameOfFileToSend))
    return;
  
  //Serial.print("File requested: ");
  //Serial.println(nameOfFileToSend);
  
  fileBeingSent = platform->OpenFile(nameOfFileToSend, false);
  if(fileBeingSent < 0)
  {
    InternalFile("html404.htm");
    return;
  }
      
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
      CloseClient(); 
    }  
}


/*

Parse a string in clientLine[] from the user's web browser

Simple requests have the form:

GET /page2.htm HTTP/1.1
     ^  Start clientRequest[] at clientLine[5]; stop at the blank or...

...fancier ones with arguments after a '?' go:

GET /gather.asp?pwd=my_pwd HTTP/1.1
     ^ Start clientRequest[]
                ^ Start clientQualifier[]
*/

void Webserver::ParseClientLine()
{ 
  if(!StringStartsWith(clientLine, "GET"))
    return;
  
  int i = 5;
  int j = 0;
  clientRequest[j] = 0;
  clientQualifier[0] = 0;
  while(clientLine[i] != ' ' && clientLine[i] != '?')
  {
    clientRequest[j] = clientLine[i];
    j++;
    i++;
  }
  clientRequest[j] = 0;
  if(clientLine[i] == '?')
  {
    i++;
    j = 0;
    while(clientLine[i] != ' ')
    {
      clientQualifier[j] = clientLine[i];
      j++;
      i++;      
    }
    clientQualifier[j] = 0;
  }
  
  if(!clientRequest[0])
    strcpy(clientRequest, "index.htm");
}

void Webserver::CheckPassword()
{
  if(!StringEndsWith(clientQualifier, password))
    return;
    
  gotPassword = true;
  strcpy(clientRequest, "index.htm");
}
  

void Webserver::ParseQualifier()
{
  if(!clientQualifier[0])
    return;
  if(StringStartsWith(clientQualifier, "pwd="))
    CheckPassword();
  if(StringStartsWith(clientQualifier, "gcode="))
  {
    if(!LoadGcodeBuffer(&clientQualifier[6], true))
      platform->Message(HOST_MESSAGE, "Webserver: buffer not free!");
    strcpy(clientRequest, "index.htm"); // FIXME
  } 
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
      Serial.write(c);
      // if you've gotten to the end of the line (received a newline
      // character) and the line is blank, the http request has ended,
      // so you can send a reply
      if (c == '\n' && clientLineIsBlank) 
      {
        clientLine[clientLinePointer] = 0;
        clientLinePointer = 0;
        ParseQualifier();
        SendFile(clientRequest);
        clientRequest[0] = 0;
        return;
      }
      if (c == '\n') 
      {
        clientLine[clientLinePointer] = 0;
        ParseClientLine();
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






