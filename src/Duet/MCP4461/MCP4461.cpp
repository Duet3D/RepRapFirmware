#include "MCP4461.h"
#include "RepRapFirmware.h"

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
MCP4461::MCP4461() {
	_mcp4461_address = DEFAULT_ADDRESS;
}

//set the MCP4461 address
void MCP4461::setMCP4461Address(uint8_t mcp4461_addr) {
	_mcp4461_address = mcp4461_addr;
}

void MCP4461::setVolatileWiper(uint8_t wiper, uint16_t wiper_value){
  uint16_t value = wiper_value;
  if (value > 0xFF) value = 0x100;
  uint8_t d_byte = (uint8_t)value;
  uint8_t c_byte;
  if (value > 0xFF)c_byte = 0x1; //the 8th data bit is 1
  else c_byte =0;
  switch (wiper) {
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
        break; //not a valid wiper
  } 
  c_byte |= MCP4461_WRITE;
  //send command byte
  MCP_WIRE.beginTransmission(_mcp4461_address);
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  MCP_WIRE.endTransmission(); //do not release bus
  }

void MCP4461::setNonVolatileWiper(uint8_t wiper, uint16_t wiper_value){
  uint16_t value = wiper_value;
  if (value > 0xFF) value = 0x100;
  uint8_t d_byte = (uint8_t)value;
  uint8_t c_byte;
  if (value > 0xFF)c_byte = 0x1; //the 8th data bit is 1
  else c_byte =0;
  switch (wiper) {
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
        break; //not a valid wiper
  } 
  c_byte |= MCP4461_WRITE;
  //send command byte
  MCP_WIRE.beginTransmission(_mcp4461_address);
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  MCP_WIRE.endTransmission(); //do not release bus
  delay(20); //allow the write to complete (this is wasteful - better to check if the write has completed)
  }
  
//set all the wipers in one transmission, more verbose but quicker than multiple calls to
//setVolatileWiper(uint8_t wiper, uint16_t wiper_value)
void MCP4461::setVolatileWipers(uint16_t wiper_value){
  uint16_t value = wiper_value;
  if (value > 0xFF) value = 0x100;
  uint8_t d_byte = (uint8_t)value;
  uint8_t c_byte;
  if (value > 0xFF)c_byte = 0x1; //the 8th data bit is 1
  else c_byte =0;
  MCP_WIRE.beginTransmission(_mcp4461_address);
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_VW0;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  if (value > 0xFF) c_byte = 0x1;
  else c_byte =0;
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_VW1;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  if (value > 0xFF) c_byte = 0x1;
  else c_byte =0;
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_VW2;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  if (value > 0xFF) c_byte = 0x1;
  else c_byte =0;
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_VW3;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  MCP_WIRE.endTransmission();
}

//set all the wipers in one transmission, more verbose but quicker than multiple calls to
//setNonVolatileWiper(uint8_t wiper, uint16_t wiper_value)
void MCP4461::setNonVolatileWipers(uint16_t wiper_value){
 uint16_t value = wiper_value;
  if (value > 0xFF) value = 0x100;
  uint8_t d_byte = (uint8_t)value;
  uint8_t c_byte;
  if (value > 0xFF)c_byte = 0x1; //the 8th data bit is 1
  else c_byte =0;
  MCP_WIRE.beginTransmission(_mcp4461_address);
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_NVW0;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  delay(20); //allow the write to complete (this is wasteful - better to check if the write has completed)
  if (value > 0xFF) c_byte = 0x1;
  else c_byte =0;
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_NVW1;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  delay(20);
  if (value > 0xFF) c_byte = 0x1;
  else c_byte =0;
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_NVW2;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  delay(20);
  if (value > 0xFF) c_byte = 0x1;
  else c_byte =0;
  c_byte |= MCP4461_WRITE;
  c_byte |= MCP4461_NVW3;
  MCP_WIRE.write(c_byte);
  MCP_WIRE.write(d_byte);
  MCP_WIRE.endTransmission();
  delay(20);
}

//return the value for a specific wiper
uint16_t MCP4461::getNonVolatileWiper(uint8_t wiper){
  uint16_t ret = 0;
  uint16_t c_byte =0;
  switch (wiper) {
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
        return 0; //not a valid wiper
  } 
  c_byte |= MCP4461_READ; 
  //send command byte
  MCP_WIRE.beginTransmission(_mcp4461_address);
  MCP_WIRE.write(c_byte);
  MCP_WIRE.endTransmission(false); //do not release bus
  MCP_WIRE.requestFrom((uint8_t)_mcp4461_address,(uint8_t)2);
  //read the register
  int i = 0;
  while(MCP_WIRE.available())
  { 
    ret |= MCP_WIRE.read();
    if (i==0) ret = ret<<8;
    i++;
  }
  return ret;
}
  
//return the volatile value for a specific wiper
uint16_t MCP4461::getVolatileWiper(uint8_t wiper){
  uint16_t ret = 0;
  uint16_t c_byte =0;
  switch (wiper) {
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
        return 0; //not a valid wiper
  } 
  c_byte |= MCP4461_READ; 
  //send command byte
  MCP_WIRE.beginTransmission(_mcp4461_address);
  MCP_WIRE.write(c_byte);
  MCP_WIRE.endTransmission(false); //do not release bus
  MCP_WIRE.requestFrom((uint8_t)_mcp4461_address,(uint8_t)2);
  //read the register
  int i = 0;
  while(MCP_WIRE.available())
  { 
    ret |= MCP_WIRE.read();
    if (i==0) ret = ret<<8;
    i++;
  }
  return ret;
} 


//return the status register
uint16_t MCP4461::getStatus(){
  uint16_t ret = 0;
  uint16_t c_byte =0;
  c_byte |= MCP4461_STATUS;
  c_byte |= MCP4461_READ; 
  //send command byte
  MCP_WIRE.beginTransmission(_mcp4461_address);
  MCP_WIRE.write(c_byte);
  MCP_WIRE.endTransmission(false); //do not release bus
  MCP_WIRE.requestFrom((uint8_t)_mcp4461_address, (uint8_t)2);
  //read the register
  int i = 0;
  while(MCP_WIRE.available())
  { 
    ret |= MCP_WIRE.read();
    if (i==0) ret = ret<<8;
    i++;
  }
  return ret;
}

//toggle a specific pot channel on and off
/*  //NOT YET IMPLEMENTED
void MCP4461::toggleWiper(uint8_t wiper){
  uint16_t tcon = 0;
  uint16_t c_byte =0;
  //read the specific TCONX register to get the current stae of the
  //pot connections
  if (wiper <=1) {
    c_byte |= MCP4461_TCON0;  
    c_byte |= MCP4461_READ;
  }
  else {
    c_byte |= MCP4461_TCON1;  
    c_byte |= MCP4461_READ;
  }
  //send command byte
  MCP_WIRE.beginTransmission(_mcp4461_address);
  MCP_WIRE.write(c_byte);
  MCP_WIRE.endTransmission(false); //do not release bus
  MCP_WIRE.requestFrom((uint8_t)_mcp4461_address,(uint8_t)2);
  //read the register
  int i = 0;
  while(MCP_WIRE.available())
  { 
    tcon |= MCP_WIRE.read();
    if (i==0) tcon = tcon<<8;
    i++;
  }
  SerialUSB.print(" TCON ");
  SerialUSB.print(tcon,BIN);
} */
