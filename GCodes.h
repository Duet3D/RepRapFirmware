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

// Small class to hold an individual GCode

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
    char* GetString();
    char* Buffer();
    bool Finished();
    void SetFinished(bool f);
    
  private:
    Platform* platform;
    char gcodeBuffer[GCODE_LENGTH];
    char* identity;
    int gcodePointer;
    int readPointer;
    bool inComment;
    bool finished;
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
    bool ReadHeat(float* h);
    void QueueFileToPrint(char* fileName);
    bool GetProbeCoordinates(int count, float& x, float& y, float& z);
    bool PrintingAFile();
    void Diagnostics();
    
  private:
  
    bool AllMovesAreFinishedAndMoveBufferIsLoaded();
    bool ActOnGcode(GCodeBuffer* gb);
    bool SetUpMove(GCodeBuffer* gb);
    bool DoDwell(GCodeBuffer *gb);
    bool DoHome();
    bool DoSingleZProbe();
    bool DoMultipleZProbe();
    bool SetOffsets(GCodeBuffer *gb);
    bool NoHome();
    bool Push();
    bool Pop();
    int8_t Heater(int8_t head);
    Platform* platform;
    bool active;
    Webserver* webserver;
    float dwellTime;
    bool dwellWaiting;
    GCodeBuffer* webGCode;
    GCodeBuffer* fileGCode;
    GCodeBuffer* serialGCode;
//    bool webGCodeFinished;
//    bool fileGCodeFinished;
    bool moveAvailable;
    bool heatAvailable;
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
    float distanceScale;
    FileStore* fileBeingPrinted;
    FileStore* fileToPrint;
    int8_t selectedHead;
    bool homeX;
    bool homeY;
    bool homeZ;
    bool homeXQueued;
    bool homeYQueued;
    bool homeZQueued;
    float gFeedRate;
    int probeCount;
    int8_t probeMoveCount;
    bool probeMoveQueued;
    float bedZs[NUMBER_OF_PROBE_POINTS];
    bool zProbesSet;
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

inline bool GCodes::PrintingAFile()
{
  return fileBeingPrinted != NULL;
}

inline bool GCodes::NoHome()
{
   return !(homeX || homeY || homeZ); 
}

// This function takes care of the fact that the heater and head indices 
// don't match because the bed is heater 0.

inline int8_t GCodes::Heater(int8_t head)
{
   return head+1; 
}

#endif
