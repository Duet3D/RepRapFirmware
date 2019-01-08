#include "MCP4461.h"
#include "RepRapFirmware.h"
#include "Tasks.h"

/*
Library to control the MCP4461 Digital Potentiometer over I2C.
http://ww1.microchip.com/downloads/en/DeviceDoc/22265a.pdf
This library does not fully implement the functionality of
the MCP4461 just the basics of changing the wiper values.
Note this is currently configured to use the second I2C bus
on the Due: Wire1
The master joins the bus with the default address of 0

No warranty given or implied, use at your own risk.
Tony@think3dprint3d.com
GPL v3
*/

#include <cstdio>
#include "Wire.h"
#include "Pins.h"

#define MCP_WIRE	I2C_IFACE

//ensure you call begin() before any other functions but note
//begin can only be called once for all MCP* objects as it initialises
//the local master through the Wire library
//if the MCP4461 does not have a default address, call set address before
//trying to communicate
MCP4461::MCP4461()
{
	_mcp4461_address = DEFAULT_ADDRESS;
}

//set the MCP4461 address
void MCP4461::setMCP4461Address(uint8_t mcp4461_addr)
{
	_mcp4461_address = mcp4461_addr;
}

void MCP4461::setVolatileWiper(uint8_t wiper, uint16_t wiper_value)
{
	if (wiper_value > 0xFF)
	{
		wiper_value = 0x100;
	}

	const uint8_t d_byte = (uint8_t)wiper_value;
	uint8_t c_byte = (uint8_t)(wiper_value >> 8);
	switch (wiper)
	{
	case 0:
		c_byte |= MCP4461_VW0;
		break;
	case 1:
		c_byte |= MCP4461_VW1;
		break;
	case 2:
		c_byte |= MCP4461_VW2;
		break;
	case 3:
		c_byte |= MCP4461_VW3;
		break;
	default:
		return;	// not a valid wiper
	}
	c_byte |= MCP4461_WRITE;

	uint8_t i2cBytes[2] = { c_byte, d_byte };
	MutexLocker lock(Tasks::GetI2CMutex());
	MCP_WIRE.Transfer(_mcp4461_address, i2cBytes, 2, 0);

	delayMicroseconds(5);				// bus needs to be free for 4.7us before the next command
}

void MCP4461::setNonVolatileWiper(uint8_t wiper, uint16_t wiper_value)
{
	if (wiper_value > 0xFF)
	{
		wiper_value = 0x100;
	}

	const uint8_t d_byte = (uint8_t)wiper_value;
	uint8_t c_byte = (uint8_t)(wiper_value >> 8);
	switch (wiper)
	{
	case 0:
		c_byte |= MCP4461_NVW0;
		break;
	case 1:
		c_byte |= MCP4461_NVW1;
		break;
	case 2:
		c_byte |= MCP4461_NVW2;
		break;
	case 3:
		c_byte |= MCP4461_NVW3;
		break;
	default:
		return;	// not a valid wiper
	}
	c_byte |= MCP4461_WRITE;

	uint8_t i2cBytes[2] = { c_byte, d_byte };
	MutexLocker lock(Tasks::GetI2CMutex());
	MCP_WIRE.Transfer(_mcp4461_address, i2cBytes, 2, 0);

	delay(20);						// writing to nonvolatile memory takes up to 10ms to complete
}
  
// End
