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
    boolean Put(char c);
    boolean Seen(char c);
    float GetFValue();
    int GetIValue();
    long GetLValue();
    char* Buffer();
    
  private:
    Platform* platform;
    char gcodeBuffer[GCODE_LENGTH];
    char* identity;
    int gcodePointer;
    int readPointer;
    boolean inComment;
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
    boolean ReadMove(float* m, boolean& ce);
    boolean ReadHeat(float* h);
    void QueueFileToPrint(char* fileName);
    boolean PrintingAFile();
    void Diagnostics();
    
  private:
  
    boolean AllMovesAreFinishedAndMoveBufferIsLoaded();
    boolean ActOnGcode(GCodeBuffer* gb);
    boolean SetUpMove(GCodeBuffer* gb);
    boolean DoDwell(GCodeBuffer *gb);
    boolean DoHome();
    boolean SetOffsets(GCodeBuffer *gb);
    boolean NoHome();
    boolean Push();
    boolean Pop();
    int8_t Heater(int8_t head);
    Platform* platform;
    boolean active;
    Webserver* webserver;
    float dwellTime;
    boolean dwellWaiting;
    GCodeBuffer* webGCode;
    GCodeBuffer* fileGCode;
    boolean webGCodeFinished;
    boolean fileGCodeFinished;
    boolean moveAvailable;
    boolean heatAvailable;
    float moveBuffer[DRIVES+1]; // Last is feedrate
    boolean checkEndStops;
    boolean drivesRelative; // All except X, Y and Z
    boolean axesRelative;   // X, Y and Z
    boolean drivesRelativeStack[STACK];
    boolean axesRelativeStack[STACK];
    float feedrateStack[STACK];
    int8_t stackPointer;
    char gCodeLetters[DRIVES + 1]; // Extra is for F
    float lastPos[DRIVES - AXES]; // Just needed for relative moves.
    float distanceScale;
    int fileBeingPrinted;
    int fileToPrint;
    int8_t selectedHead;
    boolean homeX;
    boolean homeY;
    boolean homeZ;
    boolean homeXQueued;
    boolean homeYQueued;
    boolean homeZQueued;
    float gFeedRate;
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

inline boolean GCodes::PrintingAFile()
{
  return fileBeingPrinted >= 0;
}

inline boolean GCodes::NoHome()
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
