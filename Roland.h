/****************************************************************************************************

RepRapFirmware - Roland

This class can interface with a Roland mill (e.g. Roland MDX-20/15) and allows the underlying hardware
to act as a G-Code proxy, which translates G-Codes to internal Roland commands.

-----------------------------------------------------------------------------------------------------

Version 0.1

Created on: Oct 14, 2015

Adrian Bowyer

Licence: GPL

****************************************************************************************************/

#ifndef ROLAND_H
#define ROLAND_H

// This class allows the RepRap firmware to transmit commands to a Roland mill
// See: http://www.rolanddg.com/product/3d/3d/mdx-20_15/mdx-20_15.html
//      http://altlab.org/d/content/m/pangelo/ideas/rml_command_guide_en_v100.pdf

#include "Arduino.h"
#include "Platform.h"

const float ROLAND_FACTOR = (1.016088061*100.0/2.54);	// Roland units are 0.001"
const int8_t ROLAND_RTS_PIN = 75;						// Expansion pin 29, SPI0_MOSI
const int8_t ROLAND_CTS_PIN = 76;						// Expansion pin 28, SPI0_SPCK
const size_t ROLAND_BUFFER_SIZE = 50;

// TX and RX
// Expansion pin 11, PA13_TXD1
// Expansion pin 12, PA12_RXD1

const uint16_t ROLAND_BAUD = 9600;


class Roland
{
	public:
		Roland(Platform* p);
		void Init();
		void Spin();
		bool ProcessHome();
		bool ProcessDwell(long milliseconds);
		bool ProcessG92(float v, size_t axis);
		bool ProcessSpindle(float rpm);
		bool RawWrite(const char* s);
		void GetCurrentRolandPosition(float moveBuffer[]);
		bool Active();
		void Activate();
		bool Deactivate();

	private:
		void ProcessMove();
		void Zero(bool feed);
		bool Busy();

		Platform* platform;
		float longWait;

		float move[DRIVES+1];
		float coordinates[AXES+1];
		float oldCoordinates[AXES+1];
		float offset[AXES+1];
		char buffer[ROLAND_BUFFER_SIZE];
		int bufferPointer;
		StringRef *sBuffer;
		bool active;
};

#endif

// vim: ts=4:sw=4 
