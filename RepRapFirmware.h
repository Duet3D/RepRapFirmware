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

#include "Configuration.h"
#include "Platform.h"
#include "Move.h"
#include "Heat.h"

class RepRap
{    
  public:
      
    RepRap();
    void init();
    void spin();
    
  private:
  
    Platform* platform;
    Move* move;
    Heat* heat;
    

};

extern RepRap* reprap;

#endif
