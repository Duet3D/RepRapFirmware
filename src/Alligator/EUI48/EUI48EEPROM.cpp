/*
 * SPI Microchip 25AA02E48 EUI48 Eeeprom
 *
 *  Created on: 14 May 2017
 *      Author: Marco Antonini
 */

#include "EUI48EEPROM.h"

//***************************************************************************************************
// SPI Microchip 25AA02E48 EUI48EEPROM class

EUI48EEPROM::EUI48EEPROM()
{
	initialized = false;
}

void EUI48EEPROM::Init(uint8_t cs)
{
	if( cs != 0xFF && cs > 0 && !initialized)
	{
		pinMode(cs,OUTPUT_HIGH);
		device.csPin = cs;								// chip select pin
		device.csPolarity = false;						// active low chip select
		device.spiMode = SPI_MODE_1; 					// CPHA Clock phase
		device.clockFrequency = EUI48EEPROM_MAX_FREQ;	// setup Clock Freq
		sspi_master_init(&device, 8);
		initialized = true;
	}
}

bool EUI48EEPROM::getEUI48(uint8_t *EUI48buff)
{
	bool ret = false;
	if( initialized )
	{
		uint8_t instBuff[2] = { READ_INSTRUCTION, EUI48_START_ADDRESS };

		// Select this device
		sspi_select_device(&device);
		delayMicroseconds(1);

		// Send EEPROM address and read the EUI48
		sspi_write_packet(instBuff, 2);
		delayMicroseconds(1);
		sspi_read_packet(EUI48buff, 6);

		// Deselect this device
		delayMicroseconds(1);
		sspi_deselect_device(&device);
		delayMicroseconds(1);

		//Check if the first 3 byte are Microchip OUIs
		if ( (EUI48buff[0] == 0x00 || EUI48buff[0] == 0xD8) &&
				(EUI48buff[1] == 0x04 || EUI48buff[1] == 0x1E || EUI48buff[1] == 0x80 ) &&
				(EUI48buff[2] == 0xA3 || EUI48buff[2] == 0xC0 || EUI48buff[2] == 0x39 ) )
		{
			ret=true;
		}
		else
		{
			ret=false;
		}
	}
	return ret;
}
// End
