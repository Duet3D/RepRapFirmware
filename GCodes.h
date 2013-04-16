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

class GCodes
{   
  public:
  
    GCodes(Platform* p, Webserver* w);
    void Spin();
    void Init();
    void Exit();
    boolean ReadMove(float* m);
    boolean ReadHeat(float* h);
    
  private:
  
    void ActOnGcode();
    boolean Seen(char c);
    float GetFValue();
    int GetIValue();
    void SetUpMove();
  
    Platform* platform;
    boolean active;
    Webserver* webserver;
    unsigned long lastTime;
    char gcodeBuffer[GCODE_LENGTH];
    int gcodePointer;
    int readPointer;
    boolean moveAvailable;
    boolean heatAvailable;
    float moveBuffer[DRIVES+1]; // Last is feedrate
    boolean drivesRelative; // All except X, Y and Z
    boolean axesRelative;   // X, Y and Z
    char gCodeLetters[DRIVES + 1]; // Extra is for F
};

#endif
