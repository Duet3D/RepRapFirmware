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
 * Pins descriptions
 */
extern const PinDescription nonDuePinDescription[]=
{
  { PIOA, PIO_PA5,			ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X0
  { PIOC, PIO_PC27,			ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X1
  { PIOA, PIO_PA0,			ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X2
  { PIOA, PIO_PA1,			ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X3
  { PIOC, PIO_PC11,         ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X4
  { PIOC, PIO_PC8B_PWML3,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH3,    NOT_ON_TIMER }, // PWM X5
  { PIOC, PIO_PC2B_PWML0,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH0,    NOT_ON_TIMER }, // PWM X6
  { PIOC, PIO_PC6B_PWML2,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH2,    NOT_ON_TIMER }, // PWM X7
  { PIOC, PIO_PC20,			ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X8
  { PIOD, PIO_PD9,			ID_PIOD, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X9
  //10-14
  { PIOC, PIO_PC29,			ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X10
  { PIOC, PIO_PC30,			ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X11
  { PIOC, PIO_PC10,			ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X12
  { PIOC, PIO_PC28,			ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X13
  { PIOB, PIO_PB22,			ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X14
  { PIOB, PIO_PB23,			ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X15
  { PIOB, PIO_PB24,			ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN X16
  { PIOC, PIO_PC4B_PWML1,   ID_PIOC, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),	NO_ADC, NO_ADC, PWM_CH1,    NOT_ON_TIMER }, // PWM X17
  // 18-23 - HSMCI
  { PIOA, PIO_PA20A_MCCDA,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,   				NO_ADC, NO_ADC, NOT_ON_PWM,	NOT_ON_TIMER }, // PIN_HSMCI_MCCDA_GPIO
  { PIOA, PIO_PA19A_MCCK,	ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCCK_GPIO
  { PIOA, PIO_PA21A_MCDA0,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA0_GPIO
  { PIOA, PIO_PA22A_MCDA1,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA1_GPIO
  { PIOA, PIO_PA23A_MCDA2,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA2_GPIO
  { PIOA, PIO_PA24A_MCDA3,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // PIN_HSMCI_MCDA3_GPIO
  // 24-33 - ETHERNET MAC
  { PIOB, PIO_PB0A_ETXCK,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ETXCK
  { PIOB, PIO_PB1A_ETXEN,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ETXEN
  { PIOB, PIO_PB2A_ETX0,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ETX0
  { PIOB, PIO_PB3A_ETX1,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ETX1
  { PIOB, PIO_PB4A_ECRSDV,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ECRSDV
  { PIOB, PIO_PB5A_ERX0,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ERX0
  { PIOB, PIO_PB6A_ERX1,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ERX1
  { PIOB, PIO_PB7A_ERXER,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // ERXER
  { PIOB, PIO_PB8A_EMDC,	ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // EMDC
  { PIOB, PIO_PB9A_EMDIO,   ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,					NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER } // EMDIO

};

const uint32_t MaxPinNumber = X17;

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

	const PinDescription& pinDesc = (ulPin >= X0) ? nonDuePinDescription[ulPin - X0] : g_APinDescription[ulPin];
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

	const PinDescription& pinDesc = (ulPin >= X0) ? nonDuePinDescription[ulPin - X0] : g_APinDescription[ulPin];
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

	const PinDescription& pinDesc = (ulPin >= X0) ? nonDuePinDescription[ulPin - X0] : g_APinDescription[ulPin];
	if (pinDesc.ulPinType == PIO_NOT_A_PIN)
    {
        return LOW ;
    }

	return (PIO_Get(pinDesc.pPort, PIO_INPUT, pinDesc.ulPin ) == 1) ? HIGH : LOW;
}

// Build a short-form pin descriptor for a IO pin
OutputPin::OutputPin(unsigned int pin)
{
	const PinDescription& pinDesc = (pin >= X0) ? nonDuePinDescription[pin - X0] : g_APinDescription[pin];
	pPort = pinDesc.pPort;
	ulPin = pinDesc.ulPin;
}

static bool nonDuePWMEnabled = 0;
static bool PWMChanEnabled[8] = {false,false,false,false, false,false,false,false};		// there are only 8 PWM channels

// Version of PWMC_ConfigureChannel from Arduino core, mended to not mess up PWM channel 0 when another channel is programmed
static void PWMC_ConfigureChannel_fixed( Pwm* pPwm, uint32_t ul_channel, uint32_t prescaler, uint32_t alignment, uint32_t polarity )
{
    /* Disable ul_channel (effective at the end of the current period) */
    if ((pPwm->PWM_SR & (1 << ul_channel)) != 0)
    {
        pPwm->PWM_DIS = 1 << ul_channel;
        while ((pPwm->PWM_SR & (1 << ul_channel)) != 0);
    }

    /* Configure ul_channel */
    pPwm->PWM_CH_NUM[ul_channel].PWM_CMR = prescaler | alignment | polarity;
}

// Convert an Arduino Due analog pin number to the corresponding ADC channel number
adc_channel_num_t PinToAdcChannel(int pin)
{
	if (pin < A0)
	{
		pin += A0;
	}
	return (adc_channel_num_t) (int) g_APinDescription[pin].ulADCChannelNumber;
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
2. The optional fastPwm parameter only takes effect on the first call to analogWriteNonDue for each PWM pin.
   If true on the first call then the PWM frequency will be set to 25kHz instead of 1kHz.
*/

void analogWriteNonDue(uint32_t ulPin, uint32_t ulValue, bool fastPwm)
{
	if (ulPin > MaxPinNumber)
	{
		return;
	}

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
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO - X0].pPort,nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO - X0].ulPinType,nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO - X0].ulPin,nonDuePinDescription[PIN_HSMCI_MCCDA_GPIO - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCCK_GPIO - X0].pPort,nonDuePinDescription[PIN_HSMCI_MCCK_GPIO - X0].ulPinType,nonDuePinDescription[PIN_HSMCI_MCCK_GPIO - X0].ulPin,nonDuePinDescription[PIN_HSMCI_MCCK_GPIO - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO - X0].pPort,nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO - X0].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO - X0].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA0_GPIO - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO - X0].pPort,nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO - X0].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO - X0].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA1_GPIO - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO - X0].pPort,nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO - X0].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO - X0].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA2_GPIO - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO - X0].pPort,nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO - X0].ulPinType,nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO - X0].ulPin,nonDuePinDescription[PIN_HSMCI_MCDA3_GPIO - X0].ulPinConfiguration);
  //set pullups (not on clock!)
  digitalWriteNonDue(PIN_HSMCI_MCCDA_GPIO - X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA0_GPIO - X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA1_GPIO - X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA2_GPIO - X0, HIGH);
  digitalWriteNonDue(PIN_HSMCI_MCDA3_GPIO - X0, HIGH);
}

//initialise ethernet pins
void ethPinsInit()
{
  PIO_Configure(nonDuePinDescription[PIN_EMAC_EREFCK - X0].pPort,
			  nonDuePinDescription[PIN_EMAC_EREFCK - X0].ulPinType,
			  nonDuePinDescription[PIN_EMAC_EREFCK - X0].ulPin,
			  nonDuePinDescription[PIN_EMAC_EREFCK - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ETXEN - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ETXEN - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ETXEN - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ETXEN - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ETX0 - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ETX0 - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ETX0 - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ETX0 - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ETX1 - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ETX1 - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ETX1 - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ETX1 - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ECRSDV - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ECRSDV - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ECRSDV - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ECRSDV - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ERX0 - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ERX0 - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ERX0 - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ERX0 - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ERX1 - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ERX1 - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ERX1 - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ERX1 - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_ERXER - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_ERXER - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_ERXER - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_ERXER - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_EMDC - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_EMDC - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_EMDC - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_EMDC - X0].ulPinConfiguration);
  PIO_Configure(nonDuePinDescription[PIN_EMAC_EMDIO - X0].pPort,
		  nonDuePinDescription[PIN_EMAC_EMDIO - X0].ulPinType,
		  nonDuePinDescription[PIN_EMAC_EMDIO - X0].ulPin,
		  nonDuePinDescription[PIN_EMAC_EMDIO - X0].ulPinConfiguration);
}

// End
