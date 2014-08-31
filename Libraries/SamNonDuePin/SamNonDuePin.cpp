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
Code from wiring-digital.c and from variant.cpp from the arduino software
This allows access to the pins on the SAM3X8E that are not defined in the Arduino
pin description.

At this point it only implements pinMode and digitalWrite on pin PA5 and PC27
(also on PA0,PA1,PA7 as a further example, ahtough these are defined by the Arduino software)
Note the pin numbers of "0" and "1"
*/

#include "SamNonDuePin.h"

//Example from the variant.cpp file
/*
 * DUET "undefined" pin  |  PORT  | Label
 * ----------------------+--------+-------
 *  0                    |  PA5   | "E0_EN"
 *  1                    |  PC27  | "Z_EN"


 */

/*
 * Pins descriptions
 */
extern const PinDescription nonDuePinDescription[]=
{
  { PIOA, PIO_PA5,         ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN X0
  { PIOC, PIO_PC27,        ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN X1
  { PIOA, PIO_PA0,         ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN X2
  { PIOA, PIO_PA1,         ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN X3
  { PIOC, PIO_PC11,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN X4
  { PIOC, PIO_PC8B_PWML3,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH3,     NOT_ON_TIMER }, // PWM X5
  { PIOC, PIO_PC2B_PWML0,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH0,     NOT_ON_TIMER }, // PWM X6
  { PIOC, PIO_PC6B_PWML2,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC, PWM_CH2,     NOT_ON_TIMER },  //PWM X7
  { PIOC, PIO_PC20,        ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },  //PWM X8
  // 9 .. 14
  { PIOA, PIO_PA20A_MCCDA,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCCDA_GPIO
  { PIOA, PIO_PA19A_MCCK,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCCK_GPIO
  { PIOA, PIO_PA21A_MCDA0,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA0_GPIO
  { PIOA, PIO_PA22A_MCDA1,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA1_GPIO
  { PIOA, PIO_PA23A_MCDA2,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA2_GPIO
  { PIOA, PIO_PA24A_MCDA3,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA3_GPIO
  // 15 .. 24 - ETHERNET MAC
  { PIOB, PIO_PB0A_ETXCK,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETXCK
  { PIOB, PIO_PB1A_ETXEN,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETXEN
  { PIOB, PIO_PB2A_ETX0,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETX0
  { PIOB, PIO_PB3A_ETX1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ETX1
  { PIOB, PIO_PB4A_ECRSDV,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ECRSDV
  { PIOB, PIO_PB5A_ERX0,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ERX0
  { PIOB, PIO_PB6A_ERX1,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ERX1
  { PIOB, PIO_PB7A_ERXER,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // ERXER
  { PIOB, PIO_PB8A_EMDC,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // EMDC
  { PIOB, PIO_PB9A_EMDIO,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // EMDIO

  // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;

// Fixed version of PIO_SetDebounceFilter
static void SetDebounceFilter( Pio* pPio, const uint32_t dwMask, const uint32_t dwCuttOff )
{
    pPio->PIO_IFER = dwMask; 		// enable input filtering, 0 bit field no effect
    pPio->PIO_DIFSR = dwMask; 		// select debouncing filter, 0 bit field no effect
    pPio->PIO_SCDR = ((32678/(2*(dwCuttOff))) - 1) & 0x3FFF; // the lowest 14 bits work
}

/*
pinModeNonDue
copied from the pinMode function within wiring-digital.c file, part of the arduino core.
Allows a non "Arduino Due" PIO pin to be setup.
*/
extern void pinModeNonDue( uint32_t ulPin, uint32_t ulMode, uint32_t debounceCutoff )
{
	const PinDescription& pinDesc = (ulPin >= X0) ? nonDuePinDescription[ulPin - X0] : g_APinDescription[ulPin];
    if ( pinDesc.ulPinType == PIO_NOT_A_PIN )
    {
        return;
    }

    switch ( ulMode )
    {
        case INPUT:
            /* Enable peripheral for clocking input */
            pmc_enable_periph_clk( pinDesc.ulPeripheralId ) ;
            PIO_Configure(
            	pinDesc.pPort,
            	PIO_INPUT,
            	pinDesc.ulPin,
            	0 ) ;
            if (debounceCutoff != 0)
            {
            	SetDebounceFilter(pinDesc.pPort, pinDesc.ulPin, debounceCutoff);	// enable debounce filer with specified cutoff frequency
            }
            break ;

        case INPUT_PULLUP:
            /* Enable peripheral for clocking input */
            pmc_enable_periph_clk( pinDesc.ulPeripheralId ) ;
            PIO_Configure(
            	pinDesc.pPort,
            	PIO_INPUT,
            	pinDesc.ulPin,
            	PIO_PULLUP ) ;
            if (debounceCutoff != 0)
            {
            	SetDebounceFilter(pinDesc.pPort, pinDesc.ulPin, debounceCutoff);	// enable debounce filer with specified cutoff frequency
            }
            break ;

        case OUTPUT:
            PIO_Configure(
            	pinDesc.pPort,
            	PIO_OUTPUT_1,
            	pinDesc.ulPin,
            	pinDesc.ulPinConfiguration ) ;

            /* if all pins are output, disable PIO Controller clocking, reduce power consumption */
            if ( pinDesc.pPort->PIO_OSR == 0xffffffff )
            {
                pmc_disable_periph_clk( pinDesc.ulPeripheralId ) ;
            }
            break ;

        default:
        	break ;
    }
}

/*
digitalWriteNonDue
copied from the digitalWrite function within wiring-digital.c file, part of the arduino core.
Allows digital write to a non "Arduino Due" PIO pin that has been setup as output with pinModeUndefined
*/

extern void digitalWriteNonDue( uint32_t ulPin, uint32_t ulVal )
{
	if (ulPin < X0)
	{
		digitalWrite(ulPin, ulVal);		// pass on to Arduino core
		return;
	}

	const PinDescription& pinDesc = nonDuePinDescription[ulPin - X0];

  /* Handle */
  if ( pinDesc.ulPinType == PIO_NOT_A_PIN )
  {
    return ;
  }

  if ( PIO_GetOutputDataStatus( pinDesc.pPort, pinDesc.ulPin ) == 0 )
  {
    PIO_PullUp( pinDesc.pPort, pinDesc.ulPin, ulVal ) ;
  }
  else
  {
    PIO_SetOutput( pinDesc.pPort, pinDesc.ulPin, ulVal, 0, PIO_PULLUP ) ;
  }
}

/*
digitalReadNonDue
copied from the digitalRead function within wiring-digital.c file, part of the arduino core.
Allows digital read of a non "Arduino Due" PIO pin that has been setup as input with pinModeUndefined
*/
extern int digitalReadNonDue( uint32_t ulPin )
{
	if (ulPin < X0)
	{
		return digitalRead(ulPin);		// pass on to Arduino core
	}

	const PinDescription& pinDesc = nonDuePinDescription[ulPin - X0];
	if ( pinDesc.ulPinType == PIO_NOT_A_PIN )
    {
        return LOW ;
    }

	if ( PIO_Get( pinDesc.pPort, PIO_INPUT, pinDesc.ulPin ) == 1 )
    {
        return HIGH ;
    }

	return LOW ;
}

static bool nonDuePWMEnabled = 0;
static bool PWMChanEnabled[8] = {false,false,false,false, false,false,false,false};		// there are only 8 PWM channels

// Version of PWMC_ConfigureChannel from Arduino core, fixed to not mess up PWM channel 0 when another channel is programmed
static void PWMC_ConfigureChannel_fixed( Pwm* pPwm, uint32_t ul_channel, uint32_t prescaler, uint32_t alignment, uint32_t polarity )
{
    /* Disable ul_channel (effective at the end of the current period) */
    if ((pPwm->PWM_SR & (1 << ul_channel)) != 0) {
        pPwm->PWM_DIS = 1 << ul_channel;
        while ((pPwm->PWM_SR & (1 << ul_channel)) != 0);
    }

    /* Configure ul_channel */
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
   which messes up channel 0..
2. The optional fastPwm parameter only takes effect on the first call to analogWriteNonDuet for each PWM pin.
   If true on the first call then the PWM frequency will be set to 25kHz instead of 1kHz.
*/

void analogWriteNonDue(uint32_t ulPin, uint32_t ulValue, bool fastPwm)
{
	const PinDescription& pinDesc = (ulPin >= X0) ? nonDuePinDescription[ulPin - X0] : g_APinDescription[ulPin];
	uint32_t attr = pinDesc.ulPinAttribute;
	if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
	{
		// It's a DAC pin, so we can pass it on (and it can't be an extended pin because none of our extended pins support DAC)
		analogWrite(ulPin, ulValue);
		return;
	}

	if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
	{
		if (!nonDuePWMEnabled)
		{
			// PWM Startup code
			pmc_enable_periph_clk(PWM_INTERFACE_ID);
			// Set clock A to give 1kHz PWM (the standard value for Arduino Due) and clock B to give 25kHz PWM
			PWMC_ConfigureClocks(PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE, pwmFastFrequency * PWM_MAX_DUTY_CYCLE, VARIANT_MCK);
			nonDuePWMEnabled = true;
		}

		uint32_t chan = pinDesc.ulPWMChannel;
		if (!PWMChanEnabled[chan])
		{
			// Setup PWM for this PWM channel
			PIO_Configure(pinDesc.pPort,
					pinDesc.ulPinType,
					pinDesc.ulPin,
					pinDesc.ulPinConfiguration);
			PWMC_ConfigureChannel_fixed(PWM_INTERFACE, chan, (fastPwm) ? PWM_CMR_CPRE_CLKB : PWM_CMR_CPRE_CLKA, 0, 0);
			PWMC_SetPeriod(PWM_INTERFACE, chan, PWM_MAX_DUTY_CYCLE);
			PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
			PWMC_EnableChannel(PWM_INTERFACE, chan);
			PWMChanEnabled[chan] = true;
		}

		PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue);
		return;
	}

	// Defaults to digital write
	pinModeNonDue(ulPin, OUTPUT);
	digitalWriteNonDue(ulPin, (ulValue < 128) ? LOW : HIGH);
}


//initialise HSMCI pins
void hsmciPinsinit()
{
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO].pPort,nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO].ulPinType,nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO].ulPin,nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCCK_GPIO].pPort,nonDuePinDescription[PIN_HSMCI_MCCK_GPIO].ulPinType,nonDuePinDescription[PIN_HSMCI_MCCK_GPIO].ulPin,nonDuePinDescription[PIN_HSMCI_MCCK_GPIO].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO].pPort,nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO].pPort,nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO].pPort,nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO].pPort,nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO].ulPinConfiguration);
  //set pullups (not on clock!)
  digitalWriteNonDue(PIN_HSMCI_MCCDA_GPIO+X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA0_GPIO+X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA1_GPIO+X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA2_GPIO+X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA3_GPIO+X0, HIGH);
}

//initialise ethernet pins
void ethPinsInit()
{
  PIO_Configure(nonDuePinDescription[PIN_EMAC_EREFCK].pPort,
			  nonDuePinDescription[PIN_EMAC_EREFCK].ulPinType,
			  nonDuePinDescription[PIN_EMAC_EREFCK].ulPin,
			  nonDuePinDescription[PIN_EMAC_EREFCK].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ETXEN].pPort,
		  nonDuePinDescription[PIN_EMAC_ETXEN].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ETXEN].ulPin,
		  nonDuePinDescription[PIN_EMAC_ETXEN].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ETX0].pPort,
		  nonDuePinDescription[PIN_EMAC_ETX0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ETX0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ETX0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ETX1].pPort,
		  nonDuePinDescription[PIN_EMAC_ETX1].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ETX1].ulPin,
		  nonDuePinDescription[PIN_EMAC_ETX1].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ECRSDV].pPort,
		  nonDuePinDescription[PIN_EMAC_ECRSDV].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ECRSDV].ulPin,
		  nonDuePinDescription[PIN_EMAC_ECRSDV].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ERX0].pPort,
		  nonDuePinDescription[PIN_EMAC_ERX0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ERX0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ERX0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ERX1].pPort,
		  nonDuePinDescription[PIN_EMAC_ERX1].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ERX1].ulPin,
		  nonDuePinDescription[PIN_EMAC_ERX1].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ERXER].pPort,
		  nonDuePinDescription[PIN_EMAC_ERXER].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ERXER].ulPin,
		  nonDuePinDescription[PIN_EMAC_ERXER].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_EMDC].pPort,
		  nonDuePinDescription[PIN_EMAC_EMDC].ulPinType,
		  nonDuePinDescription[PIN_EMAC_EMDC].ulPin,
		  nonDuePinDescription[PIN_EMAC_EMDC].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_EMDIO].pPort,
		  nonDuePinDescription[PIN_EMAC_EMDIO].ulPinType,
		  nonDuePinDescription[PIN_EMAC_EMDIO].ulPin,
		  nonDuePinDescription[PIN_EMAC_EMDIO].ulPinConfiguration);
}
