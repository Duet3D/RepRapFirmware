/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
Code from wiring-digital.c and wiring-analog.c from the arduino core.
See undefined.cpp file for more info
*/

#ifndef SAM_NON_DUE_PIN_H
#define SAM_NON_DUE_PIN_H

#include "Arduino.h"

// Number of pins defined in PinDescription array
#define PINS_C 25

//undefined pins constants so the undef pins can
//be refered to a Xn rather than n
static const uint8_t X0  = 0;
static const uint8_t X1  = 1;
static const uint8_t X2  = 2;
static const uint8_t X3  = 3;
static const uint8_t X4  = 4;
static const uint8_t X5  = 5;
static const uint8_t X6  = 6;
static const uint8_t X7  = 7;
static const uint8_t X8  = 8;
//HSMCI
static const uint8_t PIN_HSMCI_MCCDA_GPIO  = 9;
static const uint8_t PIN_HSMCI_MCCK_GPIO  = 10;
static const uint8_t PIN_HSMCI_MCDA0_GPIO  = 11;
static const uint8_t PIN_HSMCI_MCDA1_GPIO  = 12;
static const uint8_t PIN_HSMCI_MCDA2_GPIO  = 13;
static const uint8_t PIN_HSMCI_MCDA3_GPIO  = 14;
//EMAC
static const uint8_t PIN_EMAC_EREFCK_GPIO  = 15; //What is this one for?
static const uint8_t PIN_EMAC_EREFCK  = 15;
static const uint8_t PIN_EMAC_ETXEN  = 16;
static const uint8_t PIN_EMAC_ETX0  = 17;
static const uint8_t PIN_EMAC_ETX1  = 18;
static const uint8_t PIN_EMAC_ECRSDV  = 19;
static const uint8_t PIN_EMAC_ERX0  = 20;
static const uint8_t PIN_EMAC_ERX1  = 21;
static const uint8_t PIN_EMAC_ERXER  = 22;
static const uint8_t PIN_EMAC_EMDC  = 23;
static const uint8_t PIN_EMAC_EMDIO  = 24;

// struct used to hold the descriptions for the "non arduino" pins.
// from the Arduino.h files
extern const PinDescription nonDuePinDescription[] ;
extern void pinModeNonDue( uint32_t ulPin, uint32_t ulMode );
extern void digitalWriteNonDue( uint32_t ulPin, uint32_t ulVal );
extern int digitalReadNonDue( uint32_t ulPin);
extern void analogWriteNonDue(uint32_t ulPin, uint32_t ulValue);
extern void analogOutputNonDue();
extern void hsmciPinsinit();
extern void ethPinsInit();
#endif /* SAM_NON_DUE_PIN_H */

