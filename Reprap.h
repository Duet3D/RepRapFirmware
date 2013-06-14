/****************************************************************************************************

RepRapFirmware - Reprap

RepRap is a simple class that acts as a container for an instance of all the others.

-----------------------------------------------------------------------------------------------------

Version 0.1

21 May 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef REPRAP_H
#define REPRAP_H

class RepRap
{    
  public:
      
    RepRap();
    void Init();
    void Spin();
    void Exit();
    void Interrupt();
//    void InterruptTime();
    boolean debug();
    void debug(boolean d);
    Platform* GetPlatform();
    Move* GetMove();
    Heat* GetHeat();
    GCodes* GetGCodes();
    Webserver* GetWebserver();  
    
  private:
  
    Platform* platform;
    boolean active;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;
    boolean dbg;
};

inline Platform* RepRap::GetPlatform() { return platform; }
inline Move* RepRap::GetMove() { return move; }
inline Heat* RepRap::GetHeat() { return heat; }
inline GCodes* RepRap::GetGCodes() { return gCodes; }
inline Webserver* RepRap::GetWebserver() { return webserver; }
inline boolean RepRap::debug() { return dbg; }
inline void RepRap::debug(boolean d) { dbg = d; }
inline void RepRap::Interrupt() { move->Interrupt(); }

#endif


