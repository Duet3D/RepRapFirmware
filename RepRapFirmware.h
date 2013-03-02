/****************************************************************************************************

RepRapFirmware - Main Include

This defines versions etc, includes all the other include files in the right order and defines the 
master RepRap class.  No other definitions or information should be in here.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef REPRAPFIRMWARE_H
#define REPRAPFIRMWARE_H


#define NAME "RepRapFirmware"
#define VERSION "0.1"
#define DATE "2012-11-18"
#define LAST_AUTHOR "reprappro.com"

class Platform;
class Move;
class Heat;
class GCodes;
class Webserver;

class RepRap
{    
  public:
      
    RepRap();
    void Init();
    void Spin();
    void Exit();
    
//    Platform* getPlatform();
//    Move* getMove();
//    Heat* getHeat();
//    GCodes* getGcodes();
//    Webserver* getWebserver();    
    void Interrupt();
    
    
  private:
  
    Platform* platform;
    Move* move;
    Heat* heat;
    GCodes* gcodes;
    Webserver* webserver;
};

#include "Configuration.h"
#include "Platform.h"
#include "Move.h"
#include "Heat.h"
#include "GCodes.h"
#include "Webserver.h"

// Do nothing more in the constructor; put what you want in RepRap:Init()

inline RepRap::RepRap() 
{
  platform = new Platform(this);
  move = new Move(platform);
  heat = new Heat(platform);
  webserver = new Webserver(platform);
  gcodes = new GCodes(platform, move, heat, webserver);
}

//inline Platform* RepRap::getPlatform() { return platform; }
//inline Move* RepRap::getMove() { return move; }
//inline Heat* RepRap::getHeat() { return heat; }
//inline GCodes* RepRap::getGcodes() { return gcodes; }
//inline Webserver* RepRap::getWebserver() { return webserver; }  

extern RepRap reprap;


#endif


