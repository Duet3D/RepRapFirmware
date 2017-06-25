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

#include "RepRapFirmware.h"
#include "MessageType.h"

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
	void Diagnostics(MessageType mtype);
	void Timing(MessageType mtype);

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
	Tool* GetCurrentOrDefaultTool() const;
	uint32_t GetCurrentXAxes() const;									// Get the current axes used as X axes
	void SetToolVariables(int toolNumber, const float* standbyTemperatures, const float* activeTemperatures);
	bool IsHeaterAssignedToTool(int8_t heater) const;
	unsigned int GetNumberOfContiguousTools() const;

	unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions);
	void PrintTool(int toolNumber, StringRef& reply) const;
	void FlagTemperatureFault(int8_t dudHeater);
	void ClearTemperatureFault(int8_t wasDudHeater);

	Platform& GetPlatform() const;
	Move& GetMove() const;
	Heat& GetHeat() const;
	GCodes& GetGCodes() const;
	Network& GetNetwork() const;
	Roland& GetRoland() const;
	Scanner& GetScanner() const;
	PrintMonitor& GetPrintMonitor() const;

#if SUPPORT_IOBITS
 	PortControl& GetPortControl() const;
#endif

	void Tick();
	uint16_t GetTicksInSpinState() const;
	bool IsStopped() const;

	uint16_t GetExtrudersInUse() const;
	uint16_t GetToolHeatersInUse() const;

	OutputBuffer *GetStatusResponse(uint8_t type, ResponseSource source);
	OutputBuffer *GetConfigResponse();
	OutputBuffer *GetLegacyStatusResponse(uint8_t type, int seq);
	OutputBuffer *GetFilesResponse(const char* dir, bool flagsDirs);
	OutputBuffer *GetFilelistResponse(const char* dir);

	void Beep(int freq, int ms);
	void SetMessage(const char *msg);
	void SetAlert(const char *msg, const char *title, int mode, float timeout, bool showZControls);
	void ClearAlert();

	static void CopyParameterText(const char* src, char *dst, size_t length);
	static uint32_t DoDivide(uint32_t a, uint32_t b);		// helper function for diagnostic tests
	static uint32_t ReadDword(const char* p);				// helper function for diagnostic tests

private:
	static void EncodeString(StringRef& response, const char* src, size_t spaceToLeave, bool allowControlChars = false, char prefix = 0);

	char GetStatusCharacter() const;

	Platform* platform;
	Network* network;
	Move* move;
	Heat* heat;
	GCodes* gCodes;
	Roland* roland;
	Scanner* scanner;
 	PrintMonitor* printMonitor;

#if SUPPORT_IOBITS
 	PortControl *portControl;
#endif

	Tool* toolList;
	Tool* currentTool;
	uint32_t lastWarningMillis;					// When we last sent a warning message for things that can happen very often

	uint16_t activeExtruders;
	uint16_t activeToolHeaters;

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

	bool displayMessageBox;
	char boxMessage[MESSAGE_LENGTH + 1], boxTitle[MESSAGE_LENGTH + 1];
	int boxMode;
	uint32_t boxTimer, boxTimeout;
	bool boxZControls;
};

inline Platform& RepRap::GetPlatform() const { return *platform; }
inline Move& RepRap::GetMove() const { return *move; }
inline Heat& RepRap::GetHeat() const { return *heat; }
inline GCodes& RepRap::GetGCodes() const { return *gCodes; }
inline Network& RepRap::GetNetwork() const { return *network; }
inline Roland& RepRap::GetRoland() const { return *roland; }
inline Scanner& RepRap::GetScanner() const { return *scanner; }
inline PrintMonitor& RepRap::GetPrintMonitor() const { return *printMonitor; }

#if SUPPORT_IOBITS
inline PortControl& RepRap::GetPortControl() const { return *portControl; }
#endif

inline bool RepRap::Debug(Module m) const { return debug & (1 << m); }
inline Module RepRap::GetSpinningModule() const { return spinningModule; }

inline Tool* RepRap::GetCurrentTool() const { return currentTool; }
inline uint16_t RepRap::GetExtrudersInUse() const { return activeExtruders; }
inline uint16_t RepRap::GetToolHeatersInUse() const { return activeToolHeaters; }
inline bool RepRap::IsStopped() const { return stopped; }
inline uint16_t RepRap::GetTicksInSpinState() const { return ticksInSpinState; }

#endif


