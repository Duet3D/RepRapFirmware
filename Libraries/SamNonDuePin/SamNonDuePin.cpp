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

/*
pinModeNonDue
copied from the pinMode function within wiring-digital.c file, part of the arduino core.
Allows a non "Arduino Due" PIO pin to be setup.
*/
extern void pinModeNonDue(uint32_t ulPin, uint32_t ulMode, uint32_t debounceCutoff)
{
	if (ulPin > MaxPinNumber)
	{
		return;
	}

	const PinDescription& pinDesc = GetPinDescription(ulPin);
    if (pinDesc.ulPinType == PIO_NOT_A_PIN)
    {
        return;
    }

    switch (ulMode)
    {
        case INPUT:
            pmc_enable_periph_clk(pinDesc.ulPeripheralId);		// enable peripheral for clocking input
            PIO_Configure(
            	pinDesc.pPort,
            	PIO_INPUT,
            	pinDesc.ulPin,
            	(debounceCutoff == 0) ? 0 : PIO_DEBOUNCE );
            if (debounceCutoff != 0)
            {
            	PIO_SetDebounceFilter(pinDesc.pPort, pinDesc.ulPin, debounceCutoff);	// enable debounce filer with specified cutoff frequency
            }
            break;

        case INPUT_PULLUP:
            pmc_enable_periph_clk(pinDesc.ulPeripheralId);		// enable peripheral for clocking input
            PIO_Configure(
            	pinDesc.pPort,
            	PIO_INPUT,
            	pinDesc.ulPin,
            	(debounceCutoff == 0) ? PIO_PULLUP : PIO_PULLUP | PIO_DEBOUNCE );
            if (debounceCutoff != 0)
            {
            	PIO_SetDebounceFilter(pinDesc.pPort, pinDesc.ulPin, debounceCutoff);	// enable debounce filer with specified cutoff frequency
            }
            break;

        case OUTPUT:
            PIO_Configure(
            	pinDesc.pPort,
            	PIO_OUTPUT_1,
            	pinDesc.ulPin,
            	pinDesc.ulPinConfiguration );

            // If all pins are output, disable PIO Controller clocking, reduce power consumption
            if (pinDesc.pPort->PIO_OSR == 0xffffffff)
            {
                pmc_disable_periph_clk(pinDesc.ulPeripheralId);
            }
            break;

        default:
        	break ;
    }
}

/*
digitalWriteNonDue
copied from the digitalWrite function within wiring-digital.c file, part of the arduino core.
Allows digital write to a non "Arduino Due" PIO pin that has been setup as output with pinModeUndefined
This has now been optimised to speed up the generation of steps, so it may no longer be used to enable
the pullup resistor on an input pin (use mode INPUT_PULLUP instead).
*/

extern void digitalWriteNonDue(uint32_t ulPin, uint32_t ulVal)
{
	if (ulPin > MaxPinNumber)
	{
		return;
	}

	const PinDescription& pinDesc = GetPinDescription(ulPin);
	if (pinDesc.ulPinType != PIO_NOT_A_PIN)
	{
		if (ulVal)		// we make use of the fact that LOW is zero and HIGH is nonzero
		{
			pinDesc.pPort->PIO_SODR = pinDesc.ulPin;
		}
		else
		{
			pinDesc.pPort->PIO_CODR = pinDesc.ulPin;
		}
	}
}

/*
digitalReadNonDue
copied from the digitalRead function within wiring-digital.c file, part of the arduino core.
Allows digital read of a non "Arduino Due" PIO pin that has been setup as input with pinModeUndefined
*/
extern int digitalReadNonDue( uint32_t ulPin )
{
	if (ulPin > MaxPinNumber)
	{
		return LOW;
	}

	const PinDescription& pinDesc = GetPinDescription(ulPin);
	if (pinDesc.ulPinType == PIO_NOT_A_PIN)
    {
        return LOW ;
    }

	return (PIO_Get(pinDesc.pPort, PIO_INPUT, pinDesc.ulPin ) == 1) ? HIGH : LOW;
}

// Build a short-form pin descriptor for a IO pin
OutputPin::OutputPin(unsigned int pin)
{
	const PinDescription& pinDesc = GetPinDescription(pin);
	pPort = pinDesc.pPort;
	ulPin = pinDesc.ulPin;
}

static bool nonDuePWMEnabled = 0;
static uint16_t PWMChanFreq[8] = {0, 0, 0, 0, 0, 0, 0, 0};		// there are only 8 PWM channels
static uint16_t PWMChanPeriod[8];

// Version of PWMC_ConfigureChannel from Arduino core, mended to not mess up PWM channel 0 when another channel is programmed
static void PWMC_ConfigureChannel_fixed( Pwm* pPwm, uint32_t ul_channel, uint32_t prescaler, uint32_t alignment, uint32_t polarity )
{
    // Disable ul_channel (effective at the end of the current period)
    if ((pPwm->PWM_SR & (1 << ul_channel)) != 0)
    {
        pPwm->PWM_DIS = 1 << ul_channel;
        while ((pPwm->PWM_SR & (1 << ul_channel)) != 0);
    }

    // Configure ul_channel
    pPwm->PWM_CH_NUM[ul_channel].PWM_CMR = prescaler | alignment | polarity;
}

/*
analogWriteNonDue
copied from the analogWrite function within wiring-analog.c file, part of the arduino core.
Allows analog write to a non "Arduino Due" PWM pin. Note this does not support the other functions of
the arduino analog write function such as timer counters and the DAC. Any hardware PWM pin that is defined as such
within the unDefPinDescription[] struct should work, and non hardware PWM pin will default to digitalWriteUndefined
NOTE:
1. We must not pass on any PWM calls to the Arduino core analogWrite here, because it calls the buggy version of PWMC_ConfigureChannel
   which messes up channel 0.
2. The optional fastPwm parameter only takes effect on the first call to analogWriteNonDue for each PWM pin.
   If true on the first call then the PWM frequency will be set to 25kHz instead of 1kHz.
*/

void analogWriteNonDue(uint32_t ulPin, uint32_t ulValue, uint16_t freq)
{
	if (ulPin > MaxPinNumber)
	{
		return;
	}

	if (ulValue > 255)
	{
		ulValue = 255;
	}

	const PinDescription& pinDesc = GetPinDescription(ulPin);
	uint32_t attr = pinDesc.ulPinAttribute;
	if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
	{
		// It's a DAC pin, so we can pass it on (and it can't be an extended pin because none of our extended pins support DAC)
		analogWrite(ulPin, ulValue);
		return;
	}

	if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
	{
		uint32_t chan = pinDesc.ulPWMChannel;
		if (freq == 0)
		{
			PWMChanFreq[chan] = freq;
			pinModeNonDue(ulPin, OUTPUT);
			digitalWriteNonDue(ulPin, (ulValue < 128) ? LOW : HIGH);
		}
		else if (PWMChanFreq[chan] != freq)
		{
			if (!nonDuePWMEnabled)
			{
				// PWM Startup code
				pmc_enable_periph_clk(PWM_INTERFACE_ID);
				PWMC_ConfigureClocks(PwmSlowClock, PwmFastClock, VARIANT_MCK);	// clock A is slow clock, B is fast clock
				nonDuePWMEnabled = true;
			}

			bool useFastClock = (freq >= PwmFastClock/65535);
			uint16_t period = ((useFastClock) ? PwmFastClock : PwmSlowClock)/freq - 1;
			// Setup PWM for this PWM channel
			PIO_Configure(pinDesc.pPort,
					pinDesc.ulPinType,
					pinDesc.ulPin,
					pinDesc.ulPinConfiguration);
			PWMC_ConfigureChannel_fixed(PWM_INTERFACE, chan, (useFastClock) ? PWM_CMR_CPRE_CLKB : PWM_CMR_CPRE_CLKA, 0, 0);
			PWMC_SetPeriod(PWM_INTERFACE, chan, period);
			PWMC_SetDutyCycle(PWM_INTERFACE, chan, (ulValue * (uint32_t)period)/255);
			PWMC_EnableChannel(PWM_INTERFACE, chan);
			PWMChanFreq[chan] = freq;
			PWMChanPeriod[chan] = period;
		}
		else
		{
			PWMC_SetDutyCycle(PWM_INTERFACE, chan, (ulValue * (uint32_t)PWMChanPeriod[chan])/255);
		}
	}
	else
	{
		// Defaults to digital write
		pinModeNonDue(ulPin, OUTPUT);
		digitalWriteNonDue(ulPin, (ulValue < 128) ? LOW : HIGH);
	}
}


//initialise HSMCI pins
void hsmciPinsinit()
{
  PIO_Configure(g_APinDescription[PIN_HSMCI_MCCDA_GPIO].pPort,g_APinDescription[PIN_HSMCI_MCCDA_GPIO].ulPinType,g_APinDescription[PIN_HSMCI_MCCDA_GPIO].ulPin,g_APinDescription[PIN_HSMCI_MCCDA_GPIO].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_HSMCI_MCCK_GPIO].pPort,g_APinDescription[PIN_HSMCI_MCCK_GPIO].ulPinType,g_APinDescription[PIN_HSMCI_MCCK_GPIO].ulPin,g_APinDescription[PIN_HSMCI_MCCK_GPIO].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_HSMCI_MCDA0_GPIO].pPort,g_APinDescription[PIN_HSMCI_MCDA0_GPIO].ulPinType,g_APinDescription[PIN_HSMCI_MCDA0_GPIO].ulPin,g_APinDescription[PIN_HSMCI_MCDA0_GPIO].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_HSMCI_MCDA1_GPIO].pPort,g_APinDescription[PIN_HSMCI_MCDA1_GPIO].ulPinType,g_APinDescription[PIN_HSMCI_MCDA1_GPIO].ulPin,g_APinDescription[PIN_HSMCI_MCDA1_GPIO].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_HSMCI_MCDA2_GPIO].pPort,g_APinDescription[PIN_HSMCI_MCDA2_GPIO].ulPinType,g_APinDescription[PIN_HSMCI_MCDA2_GPIO].ulPin,g_APinDescription[PIN_HSMCI_MCDA2_GPIO].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_HSMCI_MCDA3_GPIO].pPort,g_APinDescription[PIN_HSMCI_MCDA3_GPIO].ulPinType,g_APinDescription[PIN_HSMCI_MCDA3_GPIO].ulPin,g_APinDescription[PIN_HSMCI_MCDA3_GPIO].ulPinConfiguration);
  //set pullups (not on clock!)
  digitalWriteNonDue(PIN_HSMCI_MCCDA_GPIO, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA0_GPIO, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA1_GPIO, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA2_GPIO, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA3_GPIO, HIGH);
}

//initialise ethernet pins
void ethPinsInit()
{
  PIO_Configure(g_APinDescription[PIN_EMAC_EREFCK].pPort,
		  g_APinDescription[PIN_EMAC_EREFCK].ulPinType,
		  g_APinDescription[PIN_EMAC_EREFCK].ulPin,
		  g_APinDescription[PIN_EMAC_EREFCK].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ETXEN].pPort,
		  g_APinDescription[PIN_EMAC_ETXEN].ulPinType,
		  g_APinDescription[PIN_EMAC_ETXEN].ulPin,
		  g_APinDescription[PIN_EMAC_ETXEN].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ETX0].pPort,
		  g_APinDescription[PIN_EMAC_ETX0].ulPinType,
		  g_APinDescription[PIN_EMAC_ETX0].ulPin,
		  g_APinDescription[PIN_EMAC_ETX0].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ETX1].pPort,
		  g_APinDescription[PIN_EMAC_ETX1].ulPinType,
		  g_APinDescription[PIN_EMAC_ETX1].ulPin,
		  g_APinDescription[PIN_EMAC_ETX1].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ECRSDV].pPort,
		  g_APinDescription[PIN_EMAC_ECRSDV].ulPinType,
		  g_APinDescription[PIN_EMAC_ECRSDV].ulPin,
		  g_APinDescription[PIN_EMAC_ECRSDV].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ERX0].pPort,
		  g_APinDescription[PIN_EMAC_ERX0].ulPinType,
		  g_APinDescription[PIN_EMAC_ERX0].ulPin,
		  g_APinDescription[PIN_EMAC_ERX0].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ERX1].pPort,
		  g_APinDescription[PIN_EMAC_ERX1].ulPinType,
		  g_APinDescription[PIN_EMAC_ERX1].ulPin,
		  g_APinDescription[PIN_EMAC_ERX1].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_ERXER].pPort,
		  g_APinDescription[PIN_EMAC_ERXER].ulPinType,
		  g_APinDescription[PIN_EMAC_ERXER].ulPin,
		  g_APinDescription[PIN_EMAC_ERXER].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_EMDC].pPort,
		  g_APinDescription[PIN_EMAC_EMDC].ulPinType,
		  g_APinDescription[PIN_EMAC_EMDC].ulPin,
		  g_APinDescription[PIN_EMAC_EMDC].ulPinConfiguration);
  PIO_Configure(g_APinDescription[PIN_EMAC_EMDIO].pPort,
		  g_APinDescription[PIN_EMAC_EMDIO].ulPinType,
		  g_APinDescription[PIN_EMAC_EMDIO].ulPin,
		  g_APinDescription[PIN_EMAC_EMDIO].ulPinConfiguration);
}

// End
