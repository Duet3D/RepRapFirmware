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
	return g_APinDescription[ulPin];
}

OutputPin getPioPin(uint32_t ulPin);
void hsmciPinsinit();
void ethPinsInit();

#endif /* SAM_NON_DUE_PIN_H */

