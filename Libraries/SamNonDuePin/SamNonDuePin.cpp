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
Code adapted from wiring-digital.c and from variant.cpp from the Arduino software
This allows access to the pins on the SAM3X8E that are not defined in the Arduino
pin description.
*/

#include "SamNonDuePin.h"

// Build a short-form pin descriptor for a IO pin
OutputPin::OutputPin(unsigned int pin)
{
	const PinDescription& pinDesc = GetPinDescription(pin);
	pPort = pinDesc.pPort;
	ulPin = pinDesc.ulPin;
}

static void ConfigurePin(const PinDescription& pinDesc)
{
	PIO_Configure(pinDesc.pPort, pinDesc.ulPinType, pinDesc.ulPin, pinDesc.ulPinConfiguration);
}

//initialise HSMCI pins
void hsmciPinsinit()
{
	ConfigurePin(g_APinDescription[PIN_HSMCI_MCCDA_GPIO]);
	ConfigurePin(g_APinDescription[PIN_HSMCI_MCCK_GPIO]);
	ConfigurePin(g_APinDescription[PIN_HSMCI_MCDA0_GPIO]);
	ConfigurePin(g_APinDescription[PIN_HSMCI_MCDA1_GPIO]);
	ConfigurePin(g_APinDescription[PIN_HSMCI_MCDA2_GPIO]);
	ConfigurePin(g_APinDescription[PIN_HSMCI_MCDA3_GPIO]);
}

//initialise ethernet pins
void ethPinsInit()
{
	ConfigurePin(g_APinDescription[PIN_EMAC_EREFCK]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ETXEN]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ETX0]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ETX1]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ECRSDV]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ERX0]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ERX1]);
	ConfigurePin(g_APinDescription[PIN_EMAC_ERXER]);
	ConfigurePin(g_APinDescription[PIN_EMAC_EMDC]);
	ConfigurePin(g_APinDescription[PIN_EMAC_EMDIO]);
}

// End
