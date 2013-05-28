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

// Small class to hold an individual GCode

class GCodeBuffer
{
  public:
    GCodeBuffer(Platform* p);
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
    int gcodePointer;
    int readPointer; 
};

// The GCode interpreter

class GCodes
{   
  public:
  
    GCodes(Platform* p, Webserver* w);
    void Spin();
    void Init();
    void Exit();
    boolean ReadMove(float* m);
    boolean ReadHeat(float* h);
    void QueueFileToPrint(char* fileName);
    
  private:
  
    boolean ActOnGcode(GCodeBuffer* gb);
    boolean SetUpMove(GCodeBuffer* gb);
    boolean doDwell(GCodeBuffer *gb);
    Platform* platform;
    boolean active;
    Webserver* webserver;
    unsigned long dwellTime;
    boolean dwellWaiting;
    GCodeBuffer* webGCode;
    GCodeBuffer* fileGCode;
    boolean webGCodePending;
    boolean fileGCodePending;
    boolean moveAvailable;
    boolean heatAvailable;
    float moveBuffer[DRIVES+1]; // Last is feedrate
    boolean drivesRelative; // All except X, Y and Z
    boolean axesRelative;   // X, Y and Z
    char gCodeLetters[DRIVES + 1]; // Extra is for F
    float lastPos[DRIVES - AXES]; // Just needed for relative moves.
    float distanceScale;
    int fileBeingPrinted;
    int fileToPrint;
};

#endif
