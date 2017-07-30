/*
 * SPI Microchip 25AA02E48 EUI48 EEPROM
 *
 *  Created on: 14 May 2017
 *      Author: Marco Antonini
 */

#ifndef EUI48EEPROM_H_
#define EUI48EEPROM_H_

#include <inttypes.h>
#include "Core.h"
#include "SharedSpi.h"


// 2.0 MHz standard clock frequency for Microchip 25AA02E48 @ 3.3V
#define EUI48EEPROM_MAX_FREQ 2000000u

// Microchip 25AA02E48 Addresses
#define READ_INSTRUCTION          0b00000011
#define WRITE_INSTRUCTION         0b00000010
#define READ_STATUS_INSTRUCTION   0b00000101
#define WRITE_STATUS_INSTRUCTION  0b00000001
#define EUI48_START_ADDRESS       0xFA

class EUI48EEPROM{

public:

    EUI48EEPROM();
    void Init(uint8_t cs);
    bool getEUI48(uint8_t *buffer);

private:

    bool initialized;
    struct sspi_device device;

};

#endif /* EUI48EEPROM_ */
