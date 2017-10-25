/*
 * TMC2660.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "TMC2660.h"

const float MaximumMotorCurrent = 2400.0;

static size_t numTmc2660Drivers;

static bool driversPowered = false;

// Pin assignments for the second prototype, using USART1 SPI
const Pin DriversClockPin = 15;								// PB15/TIOA1
const Pin DriversMosiPin = 22;								// PA13
const Pin DriversMisoPin = 21;								// PA22
const Pin DriversSclkPin = 23;								// PA23

const int ChopperControlRegisterMode = 999;					// mode passed to get/set microstepping to indicate we want the chopper control register

#define USART_EXT_DRV		USART1
#define ID_USART_EXT_DRV	ID_USART1
#define TMC_CLOCK_TC		TC0
#define TMC_CLOCK_CHAN		1
#define TMC_CLOCK_ID		ID_TC1							// this is channel 1 on TC0

const uint32_t DriversSpiClockFrequency = 4000000;			// 4MHz SPI clock
const int StallGuardThreshold = 1;							// Range is -64..63. Zero seems to be too sensitive. Higher values reduce sensitivity of stall detection.

// TMC2660 registers
const uint32_t TMC_REG_DRVCTRL = 0;
const uint32_t TMC_REG_CHOPCONF = 0x80000;
const uint32_t TMC_REG_SMARTEN = 0xA0000;
const uint32_t TMC_REG_SGCSCONF = 0xC0000;
const uint32_t TMC_REG_DRVCONF = 0xE0000;
const uint32_t TMC_DATA_MASK = 0x0001FFFF;

// DRVCONF register bits
const uint32_t TMC_DRVCONF_RDSEL_0 = 0 << 4;
const uint32_t TMC_DRVCONF_RDSEL_1 = 1 << 4;
const uint32_t TMC_DRVCONF_RDSEL_2 = 2 << 4;
const uint32_t TMC_DRVCONF_RDSEL_3 = 3 << 4;
const uint32_t TMC_DRVCONF_VSENSE = 1 << 6;
const uint32_t TMC_DRVCONF_SDOFF = 1 << 7;
const uint32_t TMC_DRVCONF_TS2G_3P2 = 0 << 8;
const uint32_t TMC_DRVCONF_TS2G_1P6 = 1 << 8;
const uint32_t TMC_DRVCONF_TS2G_1P2 = 2 << 8;
const uint32_t TMC_DRVCONF_TS2G_0P8 = 3 << 8;
const uint32_t TMC_DRVCONF_DISS2G = 1 << 10;
const uint32_t TMC_DRVCONF_SLPL_MIN = 0 << 12;
const uint32_t TMC_DRVCONF_SLPL_MED = 2 << 12;
const uint32_t TMC_DRVCONF_SLPL_MAX = 3 << 12;
const uint32_t TMC_DRVCONF_SLPH_MIN = 0 << 14;
const uint32_t TMC_DRVCONF_SLPH_MIN_TCOMP = 1 << 14;
const uint32_t TMC_DRVCONF_SLPH_MED_TCOMP = 2 << 14;
const uint32_t TMC_DRVCONF_SLPH_MAX = 3 << 14;
const uint32_t TMC_DRVCONF_TST = 1 << 16;

// Chopper control register bits
const uint32_t TMC_CHOPCONF_TOFF_MASK = 15;
#define TMC_CHOPCONF_TOFF(n)	((((uint32_t)n) & 15) << 0)
#define TMC_CHOPCONF_HSTRT(n)	((((uint32_t)n) & 7) << 4)
#define TMC_CHOPCONF_HEND(n)	((((uint32_t)n) & 15) << 7)
#define TMC_CHOPCONF_HDEC(n)	((((uint32_t)n) & 3) << 11)
const uint32_t TMC_CHOPCONF_RNDTF = 1 << 13;
const uint32_t TMC_CHOPCONF_CHM = 1 << 14;
#define TMC_CHOPCONF_TBL(n)	(((uint32_t)n & 3) << 15)

// Driver control register bits, when SDOFF=0
const uint32_t TMC_DRVCTRL_MRES_MASK = 0x0F;
const uint32_t TMC_DRVCTRL_MRES_SHIFT = 0;
const uint32_t TMC_DRVCTRL_MRES_16 = 0x04;
const uint32_t TMC_DRVCTRL_MRES_32 = 0x03;
const uint32_t TMC_DRVCTRL_MRES_64 = 0x02;
const uint32_t TMC_DRVCTRL_MRES_128 = 0x01;
const uint32_t TMC_DRVCTRL_MRES_256 = 0x00;
const uint32_t TMC_DRVCTRL_DEDGE = 1 << 8;
const uint32_t TMC_DRVCTRL_INTPOL = 1 << 9;

// stallGuard2 control register
const uint32_t TMC_SGCSCONF_CS_MASK = 31;
#define TMC_SGCSCONF_CS(n) ((((uint32_t)n) & 31) << 0)
const uint32_t TMC_SGCSCONF_SGT_MASK = 127 << 8;
const uint32_t TMC_SGCSCONF_SGT_SHIFT = 8;
#define TMC_SGCSCONF_SGT(n) ((((uint32_t)n) & 127) << 8)
const uint32_t TMC_SGCSCONF_SGT_SFILT = 1 << 16;

// coolStep control register
const uint32_t TMC_SMARTEN_SEMIN_MASK = 15;
const uint32_t TMC_SMARTEN_SEMIN_SHIFT = 0;
const uint32_t TMC_SMARTEN_SEUP_1 = 0 << 5;
const uint32_t TMC_SMARTEN_SEUP_2 = 1 << 5;
const uint32_t TMC_SMARTEN_SEUP_4 = 2 << 5;
const uint32_t TMC_SMARTEN_SEUP_8 = 3 << 5;
const uint32_t TMC_SMARTEN_SEMAX_MASK = 15;
const uint32_t TMC_SMARTEN_SEMAX_SHIFT = 8;
const uint32_t TMC_SMARTEN_SEDN_32 = 0 << 13;
const uint32_t TMC_SMARTEN_SEDN_8 = 1 << 13;
const uint32_t TMC_SMARTEN_SEDN_2 = 2 << 13;
const uint32_t TMC_SMARTEN_SEDN_1 = 3 << 13;
const uint32_t TMC_SMARTEN_SEIMIN_HALF = 0 << 15;
const uint32_t TMC_SMARTEN_SEIMIN_QTR = 1 << 15;

const unsigned int NumWriteRegisters = 5;

// Chopper control register defaults
// 0x901B4 as per datasheet example
// CHM bit not set, so uses spread cycle mode
const uint32_t defaultChopConfReg =
	  TMC_REG_CHOPCONF
	| TMC_CHOPCONF_TBL(2)				// blanking time 36 clocks which is about 2.4us typical (should maybe use 16 or 24 instead?)
	| TMC_CHOPCONF_HDEC(0)				// no hysteresis decrement
	| TMC_CHOPCONF_HEND(3)				// HEND = 0
	| TMC_CHOPCONF_HSTRT(3)				// HSTRT = 4
	| TMC_CHOPCONF_TOFF(4);				// TOFF = 9.2us

// StallGuard configuration register
const uint32_t defaultSgscConfReg =
	  TMC_REG_SGCSCONF
	| TMC_SGCSCONF_SGT(StallGuardThreshold);

// Driver configuration register
const uint32_t defaultDrvConfReg =
	TMC_REG_DRVCONF
	| TMC_DRVCONF_VSENSE				// use high sensitivity range
	| TMC_DRVCONF_TS2G_0P8				// fast short-to-ground detection
	| 0;

// Driver control register
const uint32_t defaultDrvCtrlReg =
	  TMC_REG_DRVCTRL
	| TMC_DRVCTRL_MRES_16
	| TMC_DRVCTRL_INTPOL;				// x16 microstepping with interpolation

// coolStep control register
const uint32_t defaultSmartEnReg =
	  TMC_REG_SMARTEN
	| 0;								// disable coolStep, it needs to be tuned ot the motor to work properly

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

struct TmcDriverState
{
	uint32_t drvCtrlReg;
	uint32_t chopConfReg;
	uint32_t smartEnReg;
	uint32_t sgcsConfReg;
	uint32_t drvConfReg;
	uint32_t lastReadValue;
	uint32_t pin;
	uint32_t configuredChopConfReg;

	void Init(uint32_t p_pin);
	void WriteAll();
	void SetChopConf(uint32_t newVal);
	void SetMicrostepping(uint32_t shift, bool interpolate);
	void SetCurrent(float current);
	void Enable(bool en);
	void SpiSendWord(uint32_t dataOut);
	uint32_t ReadStatus();
	void SetStallThreshold(int sgThreshold);
	void SetStallFilter(bool sgFilter);
	void SetCoolStep(uint16_t coolStepConfig);
	void AppendStallConfig(StringRef& reply);
};

// Initialise the state of the driver.
void TmcDriverState::Init(uint32_t p_pin)
pre(!driversPowered)
{
	drvCtrlReg = defaultDrvCtrlReg;
	configuredChopConfReg = defaultChopConfReg;
	chopConfReg = configuredChopConfReg & ~TMC_CHOPCONF_TOFF_MASK;		// disable driver at startup
	smartEnReg = defaultSmartEnReg;
	sgcsConfReg = defaultSgscConfReg;
	drvConfReg = defaultDrvConfReg;
	pin = p_pin;
	// No point in calling WriteAll() here because the drivers are assumed to be not powered up.
}

// Send an SPI control string. The drivers need 20 bits. We send and receive 24 because the USART only supports 5 to 9 bit transfers.
// If the drivers are not powered up, don't attempt a transaction, and return 0xFFFFFFFF
void TmcDriverState::SpiSendWord( uint32_t dataOut)
{
	if (driversPowered)
	{
		USART_EXT_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX;	// reset transmitter and receiver
		digitalWrite(pin, LOW);						// set CS low
		delayMicroseconds(1);						// allow some CS low setup time
		USART_EXT_DRV->US_CR = US_CR_RXEN | US_CR_TXEN;	// enable transmitter and receiver
		uint32_t dataIn = 0;
		for (int i = 0; i < 3; ++i)
		{
			USART_EXT_DRV->US_THR = (dataOut >> 16) & 0x000000FFu;
			dataOut <<= 8;
			dataIn <<= 8;
			for (int j = 0; j < 10000 && (USART_EXT_DRV->US_CSR & (US_CSR_RXRDY | US_CSR_TXRDY)) != (US_CSR_RXRDY | US_CSR_TXRDY); ++j)
			{
				// nothing
			}
			dataIn |= USART_EXT_DRV->US_RHR & 0x000000FF;
		}
		delayMicroseconds(1);
		digitalWrite(pin, HIGH);					// set CS high again
		USART_EXT_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;	// reset and disable transmitter and receiver
		delayMicroseconds(1);						// ensure it stays high for long enough before the next write
		lastReadValue = dataIn >> 4;
	}
}

// Write all registers, if the drivers are powered up
void TmcDriverState::WriteAll()
{
	SpiSendWord(chopConfReg);
	SpiSendWord(sgcsConfReg);
	SpiSendWord(drvConfReg);
	SpiSendWord(drvCtrlReg);
	SpiSendWord(smartEnReg);
}

// Set the chopper control register
void TmcDriverState::SetChopConf(uint32_t newVal)
{
	configuredChopConfReg = (newVal & 0x0001FFFF) | TMC_REG_CHOPCONF;		// save the new value
	Enable((chopConfReg & TMC_CHOPCONF_TOFF_MASK) != 0);					// send the new value, keeping the current Enable status
}

// Set the microstepping and microstep interpolation
void TmcDriverState::SetMicrostepping(uint32_t shift, bool interpolate)
{
	drvCtrlReg &= ~TMC_DRVCTRL_MRES_MASK;
	drvCtrlReg |= (shift << TMC_DRVCTRL_MRES_SHIFT) & TMC_DRVCTRL_MRES_MASK;
	if (interpolate)
	{
		drvCtrlReg |= TMC_DRVCTRL_INTPOL;
	}
	else
	{
		drvCtrlReg &= ~TMC_DRVCTRL_INTPOL;
	}
	SpiSendWord(drvCtrlReg);
}

// Set the motor current
void TmcDriverState::SetCurrent(float current)
{
	// The current sense resistor on the production Duet WiFi is 0.051 ohms.
	// This gives us a range of 101mA to 3.236A in 101mA steps in the high sensitivity range (VSENSE = 1)
	drvConfReg |= TMC_DRVCONF_VSENSE;							// this should always be set, but send it again just in case
	SpiSendWord(drvConfReg);

	const uint32_t iCurrent = static_cast<uint32_t>(constrain<float>(current, 100.0, MaximumMotorCurrent));
	const uint32_t csBits = (32 * iCurrent - 1600)/3236;		// formula checked by simulation on a spreadsheet
	sgcsConfReg &= ~TMC_SGCSCONF_CS_MASK;
	sgcsConfReg |= TMC_SGCSCONF_CS(csBits);
	SpiSendWord(sgcsConfReg);
}

// Enable or disable the driver. Also called from SetChopConf after the chopper control configuration has been changed.
void TmcDriverState::Enable(bool en)
{
	chopConfReg = (en) ? configuredChopConfReg : (configuredChopConfReg & ~TMC_CHOPCONF_TOFF_MASK);
	SpiSendWord(chopConfReg);
}

// Read the status
uint32_t TmcDriverState::ReadStatus()
{
	// We need to send a command in order to get up-to-date status
	SpiSendWord(smartEnReg);
	return lastReadValue & (TMC_RR_SG | TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST);
}

void TmcDriverState::SetStallThreshold(int sgThreshold)
{
	const uint32_t sgVal = ((uint32_t)constrain<int>(sgThreshold, -64, 63)) & 127;
	sgcsConfReg = (sgcsConfReg & ~TMC_SGCSCONF_SGT_MASK) | (sgVal << TMC_SGCSCONF_SGT_SHIFT);
	SpiSendWord(sgcsConfReg);
}

void TmcDriverState::SetStallFilter(bool sgFilter)
{
	if (sgFilter)
	{
		sgcsConfReg |= TMC_SGCSCONF_SGT_SFILT;
	}
	else
	{
		sgcsConfReg &= ~TMC_SGCSCONF_SGT_SFILT;
	}
	SpiSendWord(sgcsConfReg);
}

void TmcDriverState::SetCoolStep(uint16_t coolStepConfig)
{
	smartEnReg = TMC_REG_SMARTEN | coolStepConfig;
	SpiSendWord(smartEnReg);
}

void TmcDriverState::AppendStallConfig(StringRef& reply)
{
	const bool filtered = ((sgcsConfReg & TMC_SGCSCONF_SGT_SFILT) != 0);
	int threshold = (int)((sgcsConfReg & TMC_SGCSCONF_SGT_MASK) >> TMC_SGCSCONF_SGT_SHIFT);
	if (threshold >= 64)
	{
		threshold -= 128;
	}
	reply.catf("stall threshold %d, filter %s, coolstep %" PRIx32, threshold, ((filtered) ? "on" : "off"), smartEnReg & 0xFFFF);
}

static TmcDriverState driverStates[DRIVES];

//--------------------------- Public interface ---------------------------------

namespace SmartDrivers
{
	// Initialise the driver interface and the drivers, leaving each drive disabled.
	// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
	void Init(const Pin driverSelectPins[DRIVES], size_t numTmcDrivers)
	{
		numTmc2660Drivers = numTmcDrivers;

		// Make sure the ENN pins are high
		pinMode(GlobalTmcEnablePin, OUTPUT_HIGH);

		// Set up the SPI pins

#ifdef DUET_NG
		// The pins are already set up for SPI in the pins table
		ConfigurePin(GetPinDescription(DriversMosiPin));
		ConfigurePin(GetPinDescription(DriversMisoPin));
		ConfigurePin(GetPinDescription(DriversSclkPin));
#else
		// Pins AD0 and AD7 may have already be set up as an ADC pin by the Arduino core, so undo that here or we won't get a clock output
		ADC->ADC_CHDR = (1 << 7);

		const PinDescription& pin2 = GetPinDescription(DriversMosiPin);
		pio_configure(pin2.pPort, PIO_PERIPH_A, pin2.ulPin, PIO_DEFAULT);
		const PinDescription& pin3 = GetPinDescription(DriversMisoPin);
		pio_configure(pin3.pPort, PIO_PERIPH_A, pin3.ulPin, PIO_DEFAULT);
		const PinDescription& pin1 = GetPinDescription(DriversSclkPin);
		pio_configure(pin1.pPort, PIO_PERIPH_A, pin1.ulPin, PIO_DEFAULT);
#endif

		// Enable the clock to the USART
		pmc_enable_periph_clk(ID_USART_EXT_DRV);

		// Set up the CS pins and set them all high
		// When this becomes the standard code, we must set up the STEP and DIR pins here too.
		for (size_t drive = 0; drive < numTmc2660Drivers; ++drive)
		{
			pinMode(driverSelectPins[drive], OUTPUT_HIGH);
		}

		// Set USART_EXT_DRV in SPI mode, with data changing on the falling edge of the clock and captured on the rising edge
		USART_EXT_DRV->US_IDR = ~0u;
		USART_EXT_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		USART_EXT_DRV->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		USART_EXT_DRV->US_BRGR = VARIANT_MCK/DriversSpiClockFrequency;				// 1MHz SPI clock
		USART_EXT_DRV->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

		// We need a few microseconds of delay here for the USART to sort itself out,
		// otherwise the processor generates two short reset pulses on its own NRST pin, and resets itself.
		// 2016-07-07: removed this delay, because we no longer send commands to the TMC2660 drivers immediately.
		//delay(10);

		driversPowered = false;
		for (size_t drive = 0; drive < numTmc2660Drivers; ++drive)
		{
			driverStates[drive].Init(driverSelectPins[drive]);
		}
	}

	void SetCurrent(size_t drive, float current)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetCurrent(current);
		}
	}

	void EnableDrive(size_t drive, bool en)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].Enable(en);
		}
	}

	uint32_t GetStatus(size_t drive)
	{
		return (drive < numTmc2660Drivers) ? driverStates[drive].ReadStatus() : 0;
	}

	// Set microstepping or chopper control register
	bool SetMicrostepping(size_t drive, int microsteps, int mode)
	{
		if (drive < numTmc2660Drivers)
		{
			if (mode == ChopperControlRegisterMode && microsteps >= 0)
			{
				driverStates[drive].SetChopConf((uint32_t)microsteps);	// set the chopper control register
				return true;
			}
			else if (microsteps > 0 && (mode == 0 || mode == 1))
			{
				// Set the microstepping. We need to determine how many bits left to shift the desired microstepping to reach 256.
				unsigned int shift = 0;
				unsigned int uSteps = (unsigned int)microsteps;
				while (uSteps < 256)
				{
					uSteps <<= 1;
					++shift;
				}
				if (uSteps == 256)
				{
					driverStates[drive].SetMicrostepping(shift, mode != 0);
					return true;
				}
			}
		}
		return false;
	}

	// Get microstepping or chopper control register
	unsigned int GetMicrostepping(size_t drive, int mode, bool& interpolation)
	{
		if (drive < numTmc2660Drivers)
		{
			const uint32_t drvCtrl = driverStates[drive].drvCtrlReg;
			interpolation = (drvCtrl & TMC_DRVCTRL_INTPOL) != 0;
			if (mode == ChopperControlRegisterMode)
			{
				return driverStates[drive].configuredChopConfReg & TMC_DATA_MASK;
			}
			else
			{
				const uint32_t mresBits = (drvCtrl & TMC_DRVCTRL_MRES_MASK) >> TMC_DRVCTRL_MRES_SHIFT;
				return 256 >> mresBits;
			}
		}
		return 1;
	}

	// Flag the the drivers have been powered up.
	// Important notes:
	// 1. Before the first call to this function with powered true, you must call Init().
	// 2. This may be called by the tick ISR with powered false, possibly while another call (with powered either true or false) is being executed
	void SetDriversPowered(bool powered)
	{
		const bool wasPowered = driversPowered;
		driversPowered = powered;
		if (powered && !wasPowered)
		{
			// Power to the drivers has been provided or restored, so we need to enable and re-initialise them
			digitalWrite(GlobalTmcEnablePin, LOW);
			delayMicroseconds(10);

			for (size_t drive = 0; drive < numTmc2660Drivers; ++drive)
			{
				driverStates[drive].WriteAll();
			}
		}
		else if (!powered && wasPowered)
		{
			digitalWrite(GlobalTmcEnablePin, HIGH);			// disable the drivers
		}
	}

	void SetStallThreshold(size_t drive, int sgThreshold)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetStallThreshold(sgThreshold);
		}
	}

	void SetStallFilter(size_t drive, bool sgFilter)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetStallFilter(sgFilter);
		}
	}

	void SetCoolStep(size_t drive, uint16_t coolStepConfig)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].SetCoolStep(coolStepConfig);
		}
	}

	void AppendStallConfig(size_t drive, StringRef& reply)
	{
		if (drive < numTmc2660Drivers)
		{
			driverStates[drive].AppendStallConfig(reply);
		}
	}

};	// end namespace

// End




