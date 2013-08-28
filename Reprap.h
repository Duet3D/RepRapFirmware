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
    void Diagnostics();
//    void InterruptTime();
    bool debug();
    void debug(bool d);
    Platform* GetPlatform();
    Move* GetMove();
    Heat* GetHeat();
    GCodes* GetGCodes();
    Webserver* GetWebserver();  
    
  private:
  
    Platform* platform;
    bool active;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;
    bool dbg;
};

inline Platform* RepRap::GetPlatform() { return platform; }
inline Move* RepRap::GetMove() { return move; }
inline Heat* RepRap::GetHeat() { return heat; }
inline GCodes* RepRap::GetGCodes() { return gCodes; }
inline Webserver* RepRap::GetWebserver() { return webserver; }
inline bool RepRap::debug() { return dbg; }
inline void RepRap::debug(bool d) { dbg = d; }
inline void RepRap::Interrupt() { move->Interrupt(); }


#endif


