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

#define GCODE_LETTERS { 'X', 'Y', 'Z', 'E', 'F' } // The drives and feedrate in a GCode

// Small class to hold an individual GCode and provide functions to allow it to be parsed

class GCodeBuffer
{
  public:
    GCodeBuffer(Platform* p, char* id);
    void Init();
    bool Put(char c);
    bool Seen(char c);
    float GetFValue();
    int GetIValue();
    long GetLValue();
    char* GetUnprecedentedString();
    char* GetString();
    char* Buffer();
    bool Finished();
    void SetFinished(bool f);
    char* WritingFileDirectory();
    void SetWritingFileDirectory(char* wfd);
    
  private:
    int CheckSum();
    Platform* platform;
    char gcodeBuffer[GCODE_LENGTH];
    char* identity;
    int gcodePointer;
    int readPointer;
    bool inComment;
    bool finished;
    char* writingFileDirectory;
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
    void RunConfigurationGCodes();
    bool ReadMove(float* m, bool& ce);
    void QueueFileToPrint(char* fileName);
    bool GetProbeCoordinates(int count, float& x, float& y, float& z);
    char* GetCurrentCoordinates();
    bool PrintingAFile();
    void Diagnostics();
    
  private:
  
    bool AllMovesAreFinishedAndMoveBufferIsLoaded();
    bool DoCannedCycleMove(bool ce);
    bool ActOnGcode(GCodeBuffer* gb);
    bool SetUpMove(GCodeBuffer* gb);
    bool DoDwell(GCodeBuffer *gb);
    bool DoHome();
    bool DoSingleZProbe();
    bool SetSingleZProbeAtAPosition(GCodeBuffer *gb);
    bool DoMultipleZProbe();
    bool SetPrintZProbe(GCodeBuffer *gb, char *reply);
    bool SetOffsets(GCodeBuffer *gb);
    bool SetPositions(GCodeBuffer *gb);
    void LoadMoveBufferFromGCode(GCodeBuffer *gb);
    void LoadMoveBufferFromArray(float m[]);
    bool NoHome();
    bool Push();
    bool Pop();
    bool DisableDrives();
    bool StandbyHeaters();
    void SetEthernetAddress(GCodeBuffer *gb, int mCode);
    void HandleReply(bool error, bool fromLine, char* reply, char gMOrT, int code, bool resend);
    char* OpenFileToWrite(char* directory, char* fileName, GCodeBuffer *gb);
    void WriteGCodeToFile(GCodeBuffer *gb);
    bool SendConfigToLine();
    void WriteHTMLToFile(char b, GCodeBuffer *gb);
    bool OffsetAxes(GCodeBuffer *gb);

    int8_t Heater(int8_t head);
    Platform* platform;
    bool active;
    Webserver* webserver;
    float dwellTime;
    bool dwellWaiting;
    GCodeBuffer* webGCode;
    GCodeBuffer* fileGCode;
    GCodeBuffer* serialGCode;
    bool moveAvailable;
    float moveBuffer[DRIVES+1]; // Last is feedrate
    bool checkEndStops;
    bool drivesRelative; // All except X, Y and Z
    bool axesRelative;   // X, Y and Z
    bool drivesRelativeStack[STACK];
    bool axesRelativeStack[STACK];
    float feedrateStack[STACK];
    int8_t stackPointer;
    char gCodeLetters[DRIVES + 1]; // Extra is for F
    float lastPos[DRIVES - AXES]; // Just needed for relative moves.
	float record[DRIVES+1];
	float moveToDo[DRIVES+1];
	bool activeDrive[DRIVES+1];
	bool offSetSet;
    float distanceScale;
    FileStore* fileBeingPrinted;
    FileStore* fileToPrint;
    FileStore* fileBeingWritten;
    FileStore* configFile;
    char* eofString;
    uint8_t eofStringCounter;
    uint8_t eofStringLength;
    int8_t selectedHead;
    bool homeX;
    bool homeY;
    bool homeZ;
    bool homeAxisFinalMove;
    float gFeedRate;
    int probeCount;
    int8_t cannedCycleMoveCount;
    bool cannedCycleMoveQueued;
    bool zProbesSet;
    float longWait;
};

//*****************************************************************************************************

// Get an Int after a G Code letter

inline int GCodeBuffer::GetIValue()
{
  return (int)GetLValue();
}

inline char* GCodeBuffer::Buffer()
{
  return gcodeBuffer;
}

inline bool GCodeBuffer::Finished()
{
  return finished;
}

inline void GCodeBuffer::SetFinished(bool f)
{
  finished = f;
}

inline char* GCodeBuffer::WritingFileDirectory()
{
	return writingFileDirectory;
}

inline void GCodeBuffer::SetWritingFileDirectory(char* wfd)
{
	writingFileDirectory = wfd;
}

inline bool GCodes::PrintingAFile()
{
  return fileBeingPrinted != NULL;
}

inline bool GCodes::NoHome()
{
   return !(homeX || homeY || homeZ || homeAxisFinalMove);
}

// This function takes care of the fact that the heater and head indices 
// don't match because the bed is heater 0.

inline int8_t GCodes::Heater(int8_t head)
{
   return head+1; 
}

#endif
