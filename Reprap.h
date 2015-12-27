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

enum class ResponseSource
{
	HTTP,
	AUX,
	Generic
};

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
    Module GetSpinningModule() const;

    const char *GetName() const;
    void SetName(const char* nm);
    bool NoPasswordSet() const;
    bool CheckPassword(const char* pw) const;
    void SetPassword(const char* pw);

    void AddTool(Tool* t);
    void DeleteTool(Tool* t);
    void SelectTool(int toolNumber);
    void StandbyTool(int toolNumber);
    Tool* GetCurrentTool() const;
    Tool* GetTool(int toolNumber) const;
    Tool* GetOnlyTool() const;
    //Tool* GetToolByDrive(int driveNumber);
    void SetToolVariables(int toolNumber, const float* standbyTemperatures, const float* activeTemperatures);
	bool ToolWarningsAllowed();
	bool IsHeaterAssignedToTool(int8_t heater) const;

    unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions);
    void PrintTool(int toolNumber, StringRef& reply) const;
    void FlagTemperatureFault(int8_t dudHeater);
    void ClearTemperatureFault(int8_t wasDudHeater);

    Platform* GetPlatform() const;
    Move* GetMove() const;
    Heat* GetHeat() const;
    GCodes* GetGCodes() const;
    Network* GetNetwork() const;
    Webserver* GetWebserver() const;
	Roland* GetRoland() const;
    PrintMonitor* GetPrintMonitor() const;

    void Tick();
    uint16_t GetTicksInSpinState() const;
    bool IsStopped() const;

    uint16_t GetExtrudersInUse() const;
    uint16_t GetHeatersInUse() const;

	// Allocate an unused OutputBuffer instance. Returns true on success or false if no instance could be allocated.
	// Setting isAppending to true will guarantee that one OutputBuffer will remain available for single allocation.
	bool AllocateOutput(OutputBuffer *&buf, bool isAppending = false);

	// Get the number of bytes left for allocation. If writingBuffer is not NULL, this returns the number of free bytes for
	// continuous writes, i.e. for writes that need to allocate an extra OutputBuffer instance to finish the message.
	size_t GetOutputBytesLeft(const OutputBuffer *writingBuffer) const;

	// Replace an existing OutputBuffer with another one.
	void ReplaceOutput(OutputBuffer *&destination, OutputBuffer *source);

	// Truncate an OutputBuffer instance to free up more memory. Returns the number of released bytes.
	size_t TruncateOutput(OutputBuffer *buffer, size_t bytesNeeded);

	// Release one OutputBuffer instance. Returns the next item from the chain or nullptr if this was the last instance.
	OutputBuffer *ReleaseOutput(OutputBuffer *buf);

	OutputBuffer *GetStatusResponse(uint8_t type, ResponseSource source);
	OutputBuffer *GetConfigResponse();
	OutputBuffer *GetLegacyStatusResponse(uint8_t type, int seq);
	OutputBuffer *GetNameResponse();
	OutputBuffer *GetFilesResponse(const char* dir, bool flagsDirs);

	void Beep(int freq, int ms);
	void SetMessage(const char *msg);

    static void CopyParameterText(const char* src, char *dst, size_t length);

private:

    static void EncodeString(StringRef& response, const char* src, size_t spaceToLeave, bool allowControlChars = false, char prefix = 0);
  
    char GetStatusCharacter() const;

    Platform* platform;
    Network* network;
    Move* move;
    Heat* heat;
    GCodes* gCodes;
    Webserver* webserver;
	Roland* roland;
    PrintMonitor* printMonitor;

    Tool* toolList;
    Tool* currentTool;
	float lastToolWarningTime;

	uint16_t activeExtruders;
    uint16_t activeHeaters;

    uint16_t ticksInSpinState;
    Module spinningModule;
    float fastLoop, slowLoop;
    float lastTime;

    uint16_t debug;
    bool stopped;
    bool active;
    bool resetting;
    bool processingConfig;

    char password[PASSWORD_LENGTH + 1];
    char myName[MACHINE_NAME_LENGTH + 1];

    int beepFrequency, beepDuration;
    char message[MESSAGE_LENGTH + 1];

	OutputBuffer * volatile freeOutputBuffers;		// Messages may also be sent by ISRs,
	volatile size_t usedOutputBuffers;				// so make these volatile.
	volatile size_t maxUsedOutputBuffers;
};

inline Platform* RepRap::GetPlatform() const { return platform; }
inline Move* RepRap::GetMove() const { return move; }
inline Heat* RepRap::GetHeat() const { return heat; }
inline GCodes* RepRap::GetGCodes() const { return gCodes; }
inline Network* RepRap::GetNetwork() const { return network; }
inline Webserver* RepRap::GetWebserver() const { return webserver; }
inline Roland* RepRap::GetRoland() const { return roland; }
inline PrintMonitor* RepRap::GetPrintMonitor() const { return printMonitor; }

inline bool RepRap::Debug(Module m) const { return debug & (1 << m); }
inline Module RepRap::GetSpinningModule() const { return spinningModule; }

inline Tool* RepRap::GetCurrentTool() const { return currentTool; }
inline uint16_t RepRap::GetExtrudersInUse() const { return activeExtruders; }
inline uint16_t RepRap::GetHeatersInUse() const { return activeHeaters; }
inline void RepRap::Interrupt() { move->Interrupt(); }
inline bool RepRap::IsStopped() const { return stopped; }
inline uint16_t RepRap::GetTicksInSpinState() const { return ticksInSpinState; }

#endif


