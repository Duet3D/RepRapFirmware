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
    bool Debug();
    void SetDebug(bool d);
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
    bool debug;
};

inline Platform* RepRap::GetPlatform() { return platform; }
inline Move* RepRap::GetMove() { return move; }
inline Heat* RepRap::GetHeat() { return heat; }
inline GCodes* RepRap::GetGCodes() { return gCodes; }
inline Webserver* RepRap::GetWebserver() { return webserver; }
inline bool RepRap::Debug() { return debug; }

inline void RepRap::SetDebug(bool d)
{
	debug = d;
	if(debug)
	{
		platform->Message(HOST_MESSAGE, "Debugging enabled\n");
		platform->PrintMemoryUsage();
	}
}

inline void RepRap::Interrupt() { move->Interrupt(); }


#endif


