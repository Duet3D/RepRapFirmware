/*
 * ExternalDrivers.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#ifdef EXTERNAL_DRIVERS

#include "Platform.h"			// for typedefs uint8_t etc.

#define NUM_EXTERNAL_DRIVERS	(DRIVES - FIRST_EXTERNAL_DRIVE)

// Connections between Duet and TMC2660-EVAL board:

// Driver signal name  	Eval board pin	Our signal name   	Duet 0.6 expansion connector pin #
// SDI                 	29				MOSI              	11 (TXD1)
// SDO                  28 				MISO               	12 (RXD1)
// SCK                  27 				SCLK               	33 (AD7/PA16)
// /CS                  24				/CS                	17 (PC5_PWMH1/E1_EN)
// GND					2,3,43,44		GND					2  (GND)
// INT_STEP				17				E1_STEP				15 (PC9_PWMH3)
// INT_DIR				18				E1_DIR				16 (PC3_PWMH0)
// ENN					8				connect to ground	2  (GND)
// CLK					23				connect to ground	2  (GND
// 5V_USB				5				+3.3V				3  (+3.3V)

const Pin MosiPin = 16;								// PA13
const Pin MisoPin = 17;								// PA12
const Pin SclkPin = 54;								// PA16
const Pin SelectPins[NUM_EXTERNAL_DRIVERS] = {37, X8, 50, 47, X13};

const uint32_t SpiClockFrequency = 1000000;			// 1MHz SPI clock

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

// Read response. The microstep counter can also be read, but we don't include that here.
const uint32_t TMC_RR_SG = 1 << 0;		// stall detected
const uint32_t TMC_RR_OT = 1 << 1;		// over temperature shutdown
const uint32_t TMC_RR_OTPW = 1 << 2;	// over temperature warning
const uint32_t TMC_RR_S2G = 3 << 3;		// short to ground counter (2 bits)
const uint32_t TMC_RR_OLA = 1 << 5;		// open load A
const uint32_t TMC_RR_OLB = 1 << 6;		// open load B
const uint32_t TMC_RR_STST = 1 << 7;	// standstill detected

const unsigned int NumWriteRegisters = 5;

// Chopper control register defaults
const uint32_t defaultChopConfReg =
	  TMC_REG_CHOPCONF
	| TMC_CHOPCONF_TBL(2)
	| TMC_CHOPCONF_HDEC(0)
	| TMC_CHOPCONF_HEND(3)
	| TMC_CHOPCONF_HSTRT(3)
	| TMC_CHOPCONF_TOFF(0);				// 0x901B4 as per datasheet example except TOFF set to zero to disable driver

const uint32_t defaultChopConfToff = 4;	// default value for TOFF when drive is enabled

// StallGuard configuration register
const uint32_t defaultSgscConfReg =
	  TMC_REG_SGCSCONF
	| 0;								// minimum current until user has set it

// Driver configuration register
const uint32_t defaultDrvConfReg =
	TMC_REG_DRVCONF
	| 0;

// Driver control register
const uint32_t defaultDrvCtrlReg =
	  TMC_REG_DRVCTRL
	| TMC_DRVCTRL_MRES_16
	| TMC_DRVCTRL_INTPOL;				// x16 microstepping with interpolation

// coolStep control register
const uint32_t defaultSmartEnReg =
	  TMC_REG_SMARTEN
	| 0;								// disable coolStep, we already do this in the main firmware

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

// Send an SPI control string. The drivers need 20 bits. We send and receive 24 because the USART only supports 5 to 9 bit transfers.
uint32_t SpiSendWord(uint32_t pin, uint32_t dataOut)
{
	USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX;	// reset transmitter and receiver
	digitalWriteNonDue(pin, LOW);				// set CS low
	delayMicroseconds(1);						// allow some CS low setup time
	USART1->US_CR = US_CR_RXEN | US_CR_TXEN;	// enable transmitter and receiver
	uint32_t dataIn = 0;
	for (int i = 0; i < 3; ++i)
	{
		USART1->US_THR = (dataOut >> 16) & 0x000000FFu;
		dataOut <<= 8;
		dataIn <<= 8;
		for (int j = 0; j < 10000 && (USART1->US_CSR & (US_CSR_RXRDY | US_CSR_TXRDY)) != (US_CSR_RXRDY | US_CSR_TXRDY); ++j)
		{
			// nothing
		}
		dataIn |= USART1->US_RHR & 0x000000FF;
	}
	delayMicroseconds(1);
	digitalWriteNonDue(pin, HIGH);				// set CS high again
	delayMicroseconds(1);						// ensure it stays high for long enough before the next write
	USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;	// reset and disable transmitter and receiver
	return (dataIn >> 4) & 0x000FFFFF;
}

struct TmcDriverState
{
	uint32_t drvCtrlReg;
	uint32_t chopConfReg;
	uint32_t smartEnReg;
	uint32_t sgcsConfReg;
	uint32_t drvConfReg;
	uint32_t lastReadValue;
	uint32_t pin;

	void Init(uint32_t p_pin);
	void WriteAll();
	void SetChopConf(uint32_t newVal);
	void SetMicrostepping(uint32_t shift, bool interpolate);
	void SetCurrent(float current);
	void Enable(bool en);
	uint32_t GetStatus() const;
};

void TmcDriverState::Init(uint32_t p_pin)
{
	drvCtrlReg = defaultDrvCtrlReg;
	chopConfReg = defaultChopConfReg;
	smartEnReg = defaultSmartEnReg;
	sgcsConfReg = defaultSgscConfReg;
	drvConfReg = defaultDrvConfReg;
	pin = p_pin;
	WriteAll();
}

void TmcDriverState::WriteAll()
{
	SpiSendWord(pin, chopConfReg);
	SpiSendWord(pin, sgcsConfReg);
	SpiSendWord(pin, drvConfReg);
	SpiSendWord(pin, drvCtrlReg);
	SpiSendWord(pin, smartEnReg);
}

void TmcDriverState::SetChopConf(uint32_t newVal)
{
	chopConfReg = (newVal & 0x0001FFFF) | TMC_REG_CHOPCONF;
	SpiSendWord(pin, chopConfReg);
}

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
	SpiSendWord(pin, drvCtrlReg);
}

void TmcDriverState::SetCurrent(float current)
{
	// I am assuming that the current sense resistor is 0.1 ohms as on the evaluation board.
	// This gives us a range of 95mA to 3.05A in 95mA steps when VSENSE is high (but max allowed RMS current is 2A),
	// or 52mA to 1.65A in 52mA steps when VSENSE is low.

	if (current > 1650.0)
	{
		// Need VSENSE = 1, but set up the current first to avoid temporarily exceeding the 2A rating
		const uint32_t iCurrent = (current > 2000.0) ? 2000 : (uint32_t)current;
		const uint32_t csBits = (uint32_t)((32 * iCurrent - 1500)/3050);	// formula checked by simulation on a spreadsheet
		sgcsConfReg &= ~TMC_SGCSCONF_CS_MASK;
		sgcsConfReg |= TMC_SGCSCONF_CS(csBits);
		SpiSendWord(pin, sgcsConfReg);

		drvConfReg |= TMC_DRVCONF_VSENSE;
		SpiSendWord(pin, drvConfReg);
	}
	else
	{
		// Use VSENSE = 0
		drvConfReg &= ~TMC_DRVCONF_VSENSE;
		SpiSendWord(pin, drvConfReg);

		const uint32_t iCurrent = (current < 50) ? 50 : (uint32_t)current;
		const uint32_t csBits = (uint32_t)((32 * iCurrent - 800)/1650);		// formula checked by simulation on a spreadsheet
		sgcsConfReg &= ~TMC_SGCSCONF_CS_MASK;
		sgcsConfReg |= TMC_SGCSCONF_CS(csBits);
		SpiSendWord(pin, sgcsConfReg);
	}
}

void TmcDriverState::Enable(bool en)
{
	chopConfReg &= ~TMC_CHOPCONF_TOFF_MASK;
	if (en)
	{
		chopConfReg |= TMC_CHOPCONF_TOFF(defaultChopConfToff);
	}
	SpiSendWord(pin, chopConfReg);
}

uint32_t TmcDriverState::GetStatus() const
{
	return SpiSendWord(pin, smartEnReg) & (TMC_RR_SG | TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST);
}

static TmcDriverState driverStates[NUM_EXTERNAL_DRIVERS];

//--------------------------- Public interface ---------------------------------

namespace ExternalDrivers
{
	// Initialise the driver interface and the drivers, leaving each drive disabled
	void Init()
	{
		// Set up the SPI pins
		// PinS AD0 and AD7 may have already be set up as an ADC pin by the Arduino core, so undo that here or we won't get a clock output
		ADC->ADC_CHDR = (1 << 7);

		const PinDescription& pin2 = GetPinDescription(MosiPin);
		PIO_Configure(pin2.pPort, PIO_PERIPH_A, pin2.ulPin, PIO_DEFAULT);
		const PinDescription& pin3 = GetPinDescription(MisoPin);
		PIO_Configure(pin3.pPort, PIO_PERIPH_A, pin3.ulPin, PIO_DEFAULT);
		const PinDescription& pin1 = GetPinDescription(SclkPin);
		PIO_Configure(pin1.pPort, PIO_PERIPH_A, pin1.ulPin, PIO_DEFAULT);

		// Set up the CS pins and set them all high
		// When this becomes the standard code, we must set up the STEP and DIR pins here too.
		for (size_t drive = 0; drive < NUM_EXTERNAL_DRIVERS; ++drive)
		{
			digitalWriteNonDue(SelectPins[drive], HIGH);
			pinModeNonDue(SelectPins[drive], OUTPUT);
		}

		// Set USART1 in SPI mode, with data changing on the falling edge of the clock and captured on the rising edge
		pmc_enable_periph_clk(ID_USART1);
		USART1->US_IDR = ~0u;
		USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		USART1->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		USART1->US_BRGR = F_CPU/SpiClockFrequency;				// 1MHz SPI clock
		USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

		for (size_t drive = 0; drive < NUM_EXTERNAL_DRIVERS; ++drive)
		{
			driverStates[drive].Init(SelectPins[drive]);
		}
	}

	void SetCurrent(size_t drive, float current)
	{
		driverStates[drive].SetCurrent(current);
	}

	void EnableDrive(size_t drive, bool en)
	{
		driverStates[drive].Enable(en);
	}

	uint32_t GetStatus(size_t drive)
	{
		return driverStates[drive].GetStatus();
	}

	bool SetMicrostepping(size_t drive, int microsteps, int mode)
	{
		if (mode == 999 && microsteps >= 0)
		{
			driverStates[drive].SetChopConf((uint32_t)microsteps);	// set the chopper control register
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
		return false;
	}

	unsigned int GetMicrostepping(size_t drive, bool& interpolation)
	{
		const uint32_t drvCtrl = driverStates[drive].drvCtrlReg;
		interpolation = (drvCtrl & TMC_DRVCTRL_INTPOL) != 0;
		const uint32_t mresBits = (drvCtrl & TMC_DRVCTRL_MRES_MASK) >> TMC_DRVCTRL_MRES_SHIFT;
		return 256 >> mresBits;
	}
};

#endif

// End




