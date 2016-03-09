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
See SamNonDuePin.cpp file for more info
*/

#ifndef SAM_NON_DUE_PIN_H
#define SAM_NON_DUE_PIN_H

#include "Arduino.h"

// Number of pins defined in PinDescription array
//#define PINS_C 28  //not used

// Undefined pins constants so the undef pins can be referred to a Xn rather than n
// Any pin numbers below X0 we assume are ordinary Due pin numbers
// Note: these must all be <=127 because pin numbers are held in int8_t in some places.
// There are 92 pins defined in the Arduino Due core as at version 1.5.4, so these must all be >=92
// 2015-07-08 Tony@t3p3 Added the additional pins for the Duet 0.8.5, changed the mapping to start at 93 (>=92) and
// finish at 126 (<=127).
static const uint8_t X0  = 93;
static const uint8_t X1  = 94;
static const uint8_t X2  = 95;
static const uint8_t X3  = 96;
static const uint8_t X4  = 97;
static const uint8_t X5  = 98;
static const uint8_t X6  = 99;
static const uint8_t X7  = 100;
static const uint8_t X8  = 101;
static const uint8_t X9  = 102;
static const uint8_t X10  = 103;
static const uint8_t X11  = 104;
static const uint8_t X12  = 105; //probe
static const uint8_t X13  = 106;
static const uint8_t X14  = 107;
static const uint8_t X15  = 108;
static const uint8_t X16  = 109;
static const uint8_t X17  = 110;
//HSMCI
static const uint8_t PIN_HSMCI_MCCDA_GPIO  = 111;
static const uint8_t PIN_HSMCI_MCCK_GPIO  = 112;
static const uint8_t PIN_HSMCI_MCDA0_GPIO  = 113;
static const uint8_t PIN_HSMCI_MCDA1_GPIO  = 114;
static const uint8_t PIN_HSMCI_MCDA2_GPIO  = 115;
static const uint8_t PIN_HSMCI_MCDA3_GPIO  = 116;
//EMAC
static const uint8_t PIN_EMAC_EREFCK  = 117;
static const uint8_t PIN_EMAC_ETXEN  = 118;
static const uint8_t PIN_EMAC_ETX0  = 119;
static const uint8_t PIN_EMAC_ETX1  = 120;
static const uint8_t PIN_EMAC_ECRSDV  = 121;
static const uint8_t PIN_EMAC_ERX0  = 122;
static const uint8_t PIN_EMAC_ERX1  = 123;
static const uint8_t PIN_EMAC_ERXER  = 124;
static const uint8_t PIN_EMAC_EMDC  = 125;
static const uint8_t PIN_EMAC_EMDIO  = 126;

// Class to give fast access to digital output pins for stepping
class OutputPin
{
	Pio *pPort;
	uint32_t ulPin;
public:
	explicit OutputPin(unsigned int pin);
	OutputPin() : pPort(PIOC), ulPin(1 << 31) {}		// default constructor needed for array init - accesses PC31 which isn't on the package, so safe
	void SetHigh() const { pPort->PIO_SODR = ulPin; }
	void SetLow() const { pPort->PIO_CODR = ulPin; }
};

extern const PinDescription nonDuePinDescription[];

inline const PinDescription& GetPinDescription(uint32_t ulPin)
{
	return (ulPin >= X0) ? nonDuePinDescription[ulPin - X0] : g_APinDescription[ulPin];
}

void pinModeNonDue(uint32_t ulPin, uint32_t ulMode, uint32_t debounceCutoff = 0);	// NB only one debounce cutoff frequency can be set per PIO
void digitalWriteNonDue(uint32_t ulPin, uint32_t ulVal);
int digitalReadNonDue(uint32_t ulPin);
OutputPin getPioPin(uint32_t ulPin);
void analogWriteNonDue(uint32_t ulPin, uint32_t ulValue, uint16_t freq);
void analogOutputNonDue();
void hsmciPinsinit();
void ethPinsInit();
adc_channel_num_t PinToAdcChannel(int pin);		// convert an analog pin number to an ADC channel

#endif /* SAM_NON_DUE_PIN_H */

