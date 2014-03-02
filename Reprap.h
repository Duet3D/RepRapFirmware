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
    void EmergencyStop();
    void Init();
    void Spin();
    void Exit();
    void Interrupt();
    void Diagnostics();
    bool Debug() const;
    void SetDebug(int d);
    Platform* GetPlatform() const;
    Move* GetMove() const;
    Heat* GetHeat() const;
    GCodes* GetGCodes() const;
    Webserver* GetWebserver() const;
    void Tick();
    
  private:
  
    Platform* platform;
    bool active;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;
    bool debug;
};

inline Platform* RepRap::GetPlatform() const { return platform; }
inline Move* RepRap::GetMove() const { return move; }
inline Heat* RepRap::GetHeat() const { return heat; }
inline GCodes* RepRap::GetGCodes() const { return gCodes; }
inline Webserver* RepRap::GetWebserver() const { return webserver; }
inline bool RepRap::Debug() const { return debug; }
inline void RepRap::Interrupt() { move->Interrupt(); }


#endif


