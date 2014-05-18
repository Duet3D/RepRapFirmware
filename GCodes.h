/****************************************************************************************************

RepRapFirmware - G Codes

This class interprets G Codes from one or more sources, and calls the functions in Move, Heat etc
that drive the machine to do what the G Codes command.

-----------------------------------------------------------------------------------------------------

Version 0.1

13 February 2013

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef GCODES_H
#define GCODES_H

#define STACK 5
#define GCODE_LENGTH 100 // Maximum length of internally-generated G Code string

#define GCODE_LETTERS { 'X', 'Y', 'Z', 'E', 'F' } // The drives and feedrate in a GCode

// Small class to hold an individual GCode and provide functions to allow it to be parsed

class GCodeBuffer
{
  public:
    GCodeBuffer(Platform* p, const char* id);
    void Init();
    bool Put(char c);
    bool Seen(char c);
    float GetFValue();
    int GetIValue();
    long GetLValue();
    const char* GetUnprecedentedString();
    const char* GetString();
    const char* Buffer();
    bool Finished() const;
    void SetFinished(bool f);
    const char* WritingFileDirectory() const;
    void SetWritingFileDirectory(const char* wfd);
    
  private:
    int CheckSum();
    Platform* platform;
    char gcodeBuffer[GCODE_LENGTH];
    const char* identity;
    int gcodePointer;
    int readPointer;
    bool inComment;
    bool finished;
    const char* writingFileDirectory;
};

// Small struct to hold an open file and data relating to it.
// This is designed so that files are never left open and we never duplicate a file reference.
class FileData
{
public:
	FileData() : f(NULL) {}

	// Set this to refer to a newly-opened file
	void Set(FileStore* pfile)
	{
		Close();	// close any existing file
		f = pfile;
	}

	bool IsLive() const { return f != NULL; }

	void Close()
	{
		if (f != NULL)
		{
			f->Close();
			f = NULL;
		}
	}

	bool Read(char& b)
	{
		return f->Read(b);
	}

	// Assignment operator
	void CopyFrom(const FileData& other)
	{
		Close();
		f = other.f;
		if (f != NULL)
		{
			f->Duplicate();
		}
	}

	// Move operator
	void MoveFrom(FileData& other)
	{
		Close();
		f = other.f;
		other.Init();
	}

private:
	FileStore *f;

	void Init()
	{
		f = NULL;
	}

	// Private assignment operator to prevent us assigning these objects
	FileData& operator=(const FileData&);

	// Private copy constructor to prevent us copying these objects
	FileData(const FileData&);
};

//****************************************************************************************************

// The GCode interpreter

class GCodes
{   
  public:
  
    GCodes(Platform* p, Webserver* w);
    void Spin();
    void Init();
    void Exit();
    void Reset();
    bool RunConfigurationGCodes();
    bool ReadMove(float* m, bool& ce);
    void QueueFileToPrint(const char* fileName);
    void DeleteFile(const char* fileName);
    bool GetProbeCoordinates(int count, float& x, float& y, float& z);
    char* GetCurrentCoordinates();
    bool PrintingAFile() const;
    void Diagnostics();
    bool HaveIncomingData() const;
    bool GetAxisIsHomed(uint8_t axis) const { return axisIsHomed[axis]; }
    void SetAxisIsHomed(uint8_t axis) { axisIsHomed[axis] = true; }
    float GetExtruderPosition(uint8_t extruder) const;
    
  private:
  
    void doFilePrint(GCodeBuffer* gb);
    bool AllMovesAreFinishedAndMoveBufferIsLoaded();
    bool DoCannedCycleMove(bool ce);
    bool DoFileCannedCycles(const char* fileName);
    bool FileCannedCyclesReturn();
    bool ActOnGcode(GCodeBuffer* gb);
    bool HandleGcode(GCodeBuffer* gb);
    bool HandleMcode(GCodeBuffer* gb);
    bool HandleTcode(GCodeBuffer* gb);
    int SetUpMove(GCodeBuffer* gb);
    bool DoDwell(GCodeBuffer *gb);
    bool DoDwellTime(float dwell);
    bool DoHome(char *reply, bool& error);
    bool DoSingleZProbeAtPoint();
    bool DoSingleZProbe();
    bool SetSingleZProbeAtAPosition(GCodeBuffer *gb);
    bool DoMultipleZProbe();
    bool SetPrintZProbe(GCodeBuffer *gb, char *reply);
    bool SetOffsets(GCodeBuffer *gb);
    bool SetPositions(GCodeBuffer *gb);
    void LoadMoveBufferFromGCode(GCodeBuffer *gb, bool doingG92, bool applyLimits);
    bool NoHome() const;
    bool Push();
    bool Pop();
    bool DisableDrives();
    bool StandbyHeaters();
    void SetEthernetAddress(GCodeBuffer *gb, int mCode);
    void HandleReply(bool error, bool fromLine, const char* reply, char gMOrT, int code, bool resend);
    bool OpenFileToWrite(const char* directory, const char* fileName, GCodeBuffer *gb);
    void WriteGCodeToFile(GCodeBuffer *gb);
    bool SendConfigToLine();
    void WriteHTMLToFile(char b, GCodeBuffer *gb);
    bool OffsetAxes(GCodeBuffer *gb);
    void SetPidParameters(GCodeBuffer *gb, int heater, char reply[STRING_LENGTH]);
    void SetHeaterParameters(GCodeBuffer *gb, char reply[STRING_LENGTH]);
    int8_t Heater(int8_t head) const;

    Platform* platform;
    bool active;
    Webserver* webserver;
    float dwellTime;
    bool dwellWaiting;
    GCodeBuffer* webGCode;
    GCodeBuffer* fileGCode;
    GCodeBuffer* serialGCode;
    GCodeBuffer* cannedCycleGCode;
    bool moveAvailable;
    float moveBuffer[DRIVES+1]; // Last is feed rate
    bool checkEndStops;
    bool drivesRelative; // All except X, Y and Z
    bool axesRelative;   // X, Y and Z
    bool drivesRelativeStack[STACK];
    bool axesRelativeStack[STACK];
    float feedrateStack[STACK];
    FileData fileStack[STACK];
    int8_t stackPointer;
    char gCodeLetters[DRIVES + 1]; // Extra is for F
    float lastPos[DRIVES - AXES]; // Just needed for relative moves.
	float record[DRIVES+1];
	float moveToDo[DRIVES+1];
	bool activeDrive[DRIVES+1];
	bool offSetSet;
    float distanceScale;
    FileData fileBeingPrinted;
    FileData fileToPrint;
    FileStore* fileBeingWritten;
    FileStore* configFile;
    bool doingCannedCycleFile;
    char* eofString;
    uint8_t eofStringCounter;
    uint8_t eofStringLength;
    int8_t selectedHead;
    bool homeX;
    bool homeY;
    bool homeZ;
    float gFeedRate;
    int probeCount;
    int8_t cannedCycleMoveCount;
    bool cannedCycleMoveQueued;
    bool zProbesSet;
    float longWait;
    bool limitAxes;			// Don't think outside the box.
    bool axisIsHomed[3];	// these record which of the axes have been homed
    bool waitingForMoveToComplete;
    bool coolingInverted;
    float speedFactor;		// speed factor, including the conversion from mm/min to mm/sec, normally 1/60
    float extrusionFactor;	// extrusion factor, normally 1.0
    bool writingWebFile;
};

//*****************************************************************************************************

// Get an Int after a G Code letter

inline int GCodeBuffer::GetIValue()
{
  return (int)GetLValue();
}

inline const char* GCodeBuffer::Buffer()
{
  return gcodeBuffer;
}

inline bool GCodeBuffer::Finished() const
{
  return finished;
}

inline void GCodeBuffer::SetFinished(bool f)
{
  finished = f;
}

inline const char* GCodeBuffer::WritingFileDirectory() const
{
	return writingFileDirectory;
}

inline void GCodeBuffer::SetWritingFileDirectory(const char* wfd)
{
	writingFileDirectory = wfd;
}

inline bool GCodes::PrintingAFile() const
{
	return fileBeingPrinted.IsLive();
}

inline bool GCodes::HaveIncomingData() const
{
	return fileBeingPrinted.IsLive() || webserver->GCodeAvailable() || (platform->GetLine()->Status() & byteAvailable);
}

inline bool GCodes::NoHome() const
{
   return !(homeX || homeY || homeZ);
}

// This function takes care of the fact that the heater and head indices 
// don't match because the bed is heater 0.

inline int8_t GCodes::Heater(int8_t head) const
{
   return head+1; 
}

// Run the configuration G Code file to set up the machine.  Usually just called once
// on re-boot.

inline bool GCodes::RunConfigurationGCodes()
{
	return !DoFileCannedCycles(platform->GetConfigFile());
}

#endif
