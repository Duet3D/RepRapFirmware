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
      
    RepRap();				// Constructor for everything; calls all the other constructors
    void EmergencyStop();	// Immediately turn off all heaters and motors
    void Init();			// Initialise everything
    void Spin();			// Run everything
    void Exit();			// Stop everything
    void Interrupt();		// Called by the system interrupt
    void Diagnostics();		// Print useful stuff
    bool Debug() const;		// Is debugging enabled?
    void SetDebug(bool d);	// Turn debugging on and off
    Platform* GetPlatform() const;		// Get the sysyem specific platform
    Move* GetMove() const;				// Get the instance of the class that handles all movement
    Heat* GetHeat() const;				// Get the instance of the class that handles all heat and temperature
    GCodes* GetGCodes() const;			// Get the instance of the class that handles G Codes from all sources
    Webserver* GetWebserver() const;	// Get the instance of the class that handles Web traffic
    
  private:
  
    Platform* platform;
    bool active;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;
    bool debug;
    float lastTime, fastTime, slowTime;
};

inline Platform* RepRap::GetPlatform() const { return platform; }
inline Move* RepRap::GetMove() const { return move; }
inline Heat* RepRap::GetHeat() const { return heat; }
inline GCodes* RepRap::GetGCodes() const { return gCodes; }
inline Webserver* RepRap::GetWebserver() const { return webserver; }
inline bool RepRap::Debug() const { return debug; }

inline void RepRap::SetDebug(bool d)
{
	debug = d;
	if(debug)
	{
		platform->Message(HOST_MESSAGE, "Debugging enabled\n");
		webserver->HandleReply("Debugging enabled\n", false);
		platform->PrintMemoryUsage();
	}
	else
	{
		webserver->HandleReply("", false);
	}
}

inline void RepRap::Interrupt() { move->Interrupt(); }


#endif


