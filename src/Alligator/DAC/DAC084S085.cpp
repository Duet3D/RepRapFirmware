/*
 * Texas Instruments SPI DAC084S085
 *
 *  Created on: 14 May 2017
 *      Author: Marco Antonini
 */

#include "DAC084S085.h"

//***************************************************************************************************
// Texas Instruments SPI DAC084S085 class

DAC084S085::DAC084S085()
{
	initialized = false;
}

void DAC084S085::Init(uint8_t cs)
{
	if( cs != 0xFF && cs > 0 && !initialized)
	{
		pinMode(cs,OUTPUT_HIGH);
		device.csPin = cs;								// chip select pin
		device.csPolarity = false;						// active low chip select
		device.spiMode = SPI_MODE_1; 					// CPHA Clock phase
		device.clockFrequency = DAC084S085_MAX_FREQ;	// setup Clock Freq
		sspi_master_init(&device, 8);
		initialized = true;
	}
}

void DAC084S085::setChannel(uint8_t channel, unsigned short value)
{
    if(initialized)
    {
    	 uint8_t dacBuff[2] = { 0x10, 0x00 };

    	// Do not exceed the maximum value
    	if(value > 255)      // Dac is 8 bit
    	{
    		value=255;
    	}
    	if(channel < 0)  	 // Channel is [0-3]
    	{
    		channel = 0;
    	}
    	else if(channel > 3) // Channel is [0-3]
    	{
    		channel = 3;
    	}

    	dacBuff[1] |= (value << 4);
    	sspi_master_setup_device(&device);

    	// Select this DAC device
    	sspi_select_device(&device);
    	delayMicroseconds(1);

    	dacBuff[0] |= ( channel << 6);
    	dacBuff[0] |= (value >> 4);

    	// Write register and update the DAC output
    	sspi_write_packet(dacBuff, 2);

    	// Deselect this DAC device
    	delayMicroseconds(1);
    	sspi_deselect_device(&device);
    	delayMicroseconds(1);
    }
}


// End
