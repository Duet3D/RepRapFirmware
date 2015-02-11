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
    Module GetSpinningModule() const;
    void Spin();
    void Exit();
    void Interrupt();
    void Diagnostics();
    void Timing();

    bool Debug(Module module) const;
    void SetDebug(Module m, bool enable);
    void SetDebug(bool enable);
    void PrintDebug();

    bool NoPasswordSet() const;
    bool CheckPassword(const char* pw) const;
    void SetPassword(const char* pw);
    const char *GetName() const;
    void SetName(const char* nm);

    void AddTool(Tool* t);
    void SelectTool(int toolNumber);
    void StandbyTool(int toolNumber);
    Tool* GetCurrentTool();
    Tool* GetTool(int toolNumber);
    Tool* GetToolByDrive(int driveNumber);
    void SetToolVariables(int toolNumber, float* standbyTemperatures, float* activeTemperatures);

    void AllowColdExtrude();
    void DenyColdExtrude();
    bool ColdExtrude();

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
    uint16_t GetTicksInSpinState() const;
    bool IsStopped() const;

    uint16_t GetExtrudersInUse() const;
    uint16_t GetHeatersInUse() const;

    void GetStatusResponse(StringRef& response, uint8_t type, bool forWebserver);
    void GetLegacyStatusResponse(StringRef &response, uint8_t type, int seq) const;
    void GetNameResponse(StringRef& response) const;
    void GetFilesResponse(StringRef& response, const char* dir) const;
    void GetFileInfoResponse(StringRef& response, const char* filename) const;

    void StartingFilePrint(const char *filename);
    void Beep(int freq, int ms);
    void SetMessage(const char *msg);
    
    void MessageToGCodeReply(const char *message);
    void AppendMessageToGCodeReply(const char *message);
    void AppendCharToStatusResponse(const char c);

    const StringRef& GetGcodeReply() const;

    static void CopyParameterText(const char* src, char *dst, size_t length);

  private:

    static void EncodeString(StringRef& response, const char* src, size_t spaceToLeave, bool allowControlChars);
  
    void UpdatePrintProgress();
    float EstimateTimeLeft(uint8_t method) const;

    Platform* platform;
    Network* network;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;

    Tool* toolList;
    Tool* currentTool;
    uint16_t activeExtruders;
    uint16_t activeHeaters;
    bool coldExtrude;

    uint16_t ticksInSpinState;
    Module spinningModule;
    float fastLoop, slowLoop;
    float lastTime;

    uint16_t debug;
    bool stopped;
    bool active;
    bool resetting;
    bool processingConfig;

    char password[SHORT_STRING_LENGTH + 1];
    char myName[SHORT_STRING_LENGTH + 1];

    bool fileInfoDetected;
    char fileBeingPrinted[255];
    GcodeFileInfo currentFileInfo;

    int beepFrequency, beepDuration;
    char message[SHORT_STRING_LENGTH + 1];

    char gcodeReplyBuffer[GCODE_REPLY_LENGTH];
    StringRef gcodeReply;
    unsigned int replySeq;

    float printStartTime;
    unsigned int currentLayer;
    float warmUpDuration;

    float firstLayerDuration;
    float firstLayerHeight;
    float firstLayerFilament;
    float firstLayerProgress;

    float lastLayerTime, lastLayerFilament;
    unsigned int numLayerSamples;
    float layerDurations[MAX_LAYER_SAMPLES];
    float filamentUsagePerLayer[MAX_LAYER_SAMPLES];
    float fileProgressPerLayer[MAX_LAYER_SAMPLES];
    float layerEstimatedTimeLeft;
};

inline Platform* RepRap::GetPlatform() const { return platform; }
inline Move* RepRap::GetMove() const { return move; }
inline Heat* RepRap::GetHeat() const { return heat; }
inline GCodes* RepRap::GetGCodes() const { return gCodes; }
inline Network* RepRap::GetNetwork() const { return network; }
inline Webserver* RepRap::GetWebserver() const { return webserver; }
inline Module RepRap::GetSpinningModule() const { return spinningModule; }
inline bool RepRap::Debug(Module m) const { return debug & (1 << m); }
inline Tool* RepRap::GetCurrentTool() { return currentTool; }
inline uint16_t RepRap::GetExtrudersInUse() const { return activeExtruders; }
inline uint16_t RepRap::GetHeatersInUse() const { return activeHeaters; }
inline bool RepRap::ColdExtrude() { return coldExtrude; }
inline void RepRap::AllowColdExtrude() { coldExtrude = true; }
inline void RepRap::DenyColdExtrude() { coldExtrude = false; }
inline void RepRap::Interrupt() { move->Interrupt(); }
inline bool RepRap::IsStopped() const { return stopped; }
inline uint16_t RepRap::GetTicksInSpinState() const { return ticksInSpinState; }

inline const StringRef& RepRap::GetGcodeReply() const { return gcodeReply; }

#endif


