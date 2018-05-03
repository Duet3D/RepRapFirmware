#ifndef MCP4461_H
#define MCP4461_H

#include <cstdint>

#define DEFAULT_ADDRESS 0x2C  //With A0 and A1 grounded
#define DEFAULT_WIPER_VALUE 0x80  //Default to the wipers in midrange

// memory addresses (all shifted 4 bits left)
//For all the Wipers 0x100 = Full scale, 0x80 = mid scale, 0x0 = Zero scale
#define MCP4461_VW0 0x00
#define MCP4461_VW1 0x10
#define MCP4461_NVW0 0x20
#define MCP4461_NVW1 0x30
//TCON0: D8: Reserved D7:R1HW D6: R1A D5:R1W D4:R1B D3:R0HW D2:R0A D1:R0W D0: R0B
#define MCP4461_TCON0 0x40
//STATUS: D8:D7: Reserved D6: WL3 D5:WL2 D4:EEWA D3:WL1 D2:WL0 D1:Reserved D0: WP
#define MCP4461_STATUS 0x50
#define MCP4461_VW2 0x60
#define MCP4461_VW3 0x70
#define MCP4461_NVW2 0x80
#define MCP4461_NVW3 0x90
//TCON0: D8: Reserved D7:R3HW D6: R3A D5:R3W D4:R3B D3:R2HW D2:R2A D1:R2W D0: R2B
#define MCP4461_TCON1 0xA0

//control commands
#define MCP4461_WRITE 0x0
#define MCP4461_INCREMENT 0x4 //01 left shift by 2
#define MCP4461_DECREMENT 0x8 //10 left shift by 2
#define MCP4461_READ 0xC //11 left shift by 2

class MCP4461{
public:
  MCP4461();
  void setMCP4461Address(uint8_t);
  void setVolatileWiper(uint8_t, uint16_t);
  void setNonVolatileWiper(uint8_t, uint16_t);
  void setVolatileWipers(uint16_t);
  void setNonVolatileWipers(uint16_t);
  void toggleWiper(uint8_t); //NOT YET IMPLEMENTED
  uint16_t getVolatileWiper(uint8_t);
  uint16_t getNonVolatileWiper(uint8_t);
  uint16_t getStatus();

private:
  uint8_t _mcp4461_address;
  uint8_t _wiper;
  uint8_t _value;
};

#endif //MCP4461_H
