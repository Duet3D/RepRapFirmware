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

const size_t maxMessageLength = 30;

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
    void Timing();
    bool Debug(Module module) const;
    void SetDebug(Module m, bool enable);
    void SetDebug(bool enable);
    void PrintDebug();
    void AddTool(Tool* t);
    void SelectTool(int toolNumber);
    void StandbyTool(int toolNumber);
    Tool* GetCurrentTool();
    Tool* GetTool(int toolNumber);
    Tool* GetToolByDrive(int driveNumber);
    void SetToolVariables(int toolNumber, float* standbyTemperatures, float* activeTemperatures);
    void AllowColdExtrude();
    void DenyColdExtrude();
    bool ColdExtrude() const;
    void GetExtruderCapabilities(bool canDrive[], const bool directions[]) const;
    void PrintTool(int toolNumber, StringRef& reply) const;
    void FlagTemperatureFault(int8_t dudHeater);
    void ClearTemperatureFault(int8_t wasDudHeater);
    Platform* GetPlatform() const;
    Move* GetMove() const;
    Heat* GetHeat() const;
    GCodes* GetGCodes() const;
    Network* GetNetwork() const;
    Webserver* GetWebserver() const;
    void Tick();
    bool IsStopped() const;
    uint16_t GetTicksInSpinState() const;
    uint16_t GetExtrudersInUse() const;
    uint16_t GetHeatersInUse() const;
    void GetStatusResponse(StringRef& response, uint8_t type) const;
    void GetNameResponse(StringRef& response) const;
    void GetFilesResponse(StringRef& response, const char* dir) const;
    void GetFileInfoResponse(StringRef& response, const char* filename) const;
    void StartingFilePrint(const char *filename);
    void SetMessage(const char *msg);
    
  private:
    static void EncodeString(StringRef& response, const char* src, size_t spaceToLeave, bool allowControlChars);
  
    Platform* platform;
    Network* network;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;
    Tool* toolList;
    Tool* currentTool;
    uint16_t ticksInSpinState;
    Module currentModule;
    uint16_t debug;
    float fastLoop, slowLoop;
    float lastTime;
    bool stopped;
    bool active;
    bool resetting;
    bool processingConfig;
    uint16_t activeExtruders;
    uint16_t activeHeaters;
    bool coldExtrude;

    // File information about the file being printed
	bool fileInfoDetected;
	char fileBeingPrinted[255];
	GcodeFileInfo currentFileInfo;
	float printStartTime;
	char message[maxMessageLength + 1];
};

inline Platform* RepRap::GetPlatform() const { return platform; }
inline Move* RepRap::GetMove() const { return move; }
inline Heat* RepRap::GetHeat() const { return heat; }
inline GCodes* RepRap::GetGCodes() const { return gCodes; }
inline Network* RepRap::GetNetwork() const { return network; }
inline Webserver* RepRap::GetWebserver() const { return webserver; }
inline bool RepRap::Debug(Module m) const { return debug & (1 << m); }
inline Tool* RepRap::GetCurrentTool() { return currentTool; }
inline uint16_t RepRap::GetExtrudersInUse() const { return activeExtruders; }
inline uint16_t RepRap::GetHeatersInUse() const { return activeHeaters; }
inline bool RepRap::ColdExtrude() const { return coldExtrude; }
inline void RepRap::AllowColdExtrude() { coldExtrude = true; }
inline void RepRap::DenyColdExtrude() { coldExtrude = false; }
inline void RepRap::Interrupt() { move->Interrupt(); }
inline bool RepRap::IsStopped() const { return stopped; }
inline uint16_t RepRap::GetTicksInSpinState() const { return ticksInSpinState; }

#endif


