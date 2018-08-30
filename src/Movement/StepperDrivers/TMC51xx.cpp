/*
 * TMC51xx.cpp
 *
 *  Created on: 26 Aug 2018
 *      Author: David
 *  Purpose:
 *  	Support for TMC5130 and TMC5160 stepper drivers
 */

#include "RepRapFirmware.h"

#if SUPPORT_TMC51xx

#include "TMC51xx.h"

constexpr uint32_t DefaultTpwmthrsReg = 2000;							// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle

// The SPI clock speed is a compromise:
// - too high and polling the driver chips take too much of the CPU time
// - too low and we won't detect stalls quickly enough
// With a 4MHz SPI clock:
// - polling the drivers makes calculations take ??% longer, so it is taking about ??% of the CPU time
// - we poll all ?? drivers in about ??us
// With a 2MHz SPI clock:
// - polling the drivers makes calculations take ??% longer, so it is taking about ??% of the CPU time
// - we poll all ?? drivers in about ??us
const uint32_t DriversSpiClockFrequency = 2000000;			// 2MHz SPI clock
const uint32_t TransferTimeout = 2;							// any transfer should complete within 2 ticks @ 1ms/tick

// GCONF register (0x00, RW)
constexpr uint8_t REGNUM_GCONF = 0x00;

constexpr uint32_t GCONF_5130_USE_VREF = 1 << 0;			// use external VRef
constexpr uint32_t GCONF_5130_INT_RSENSE = 1 << 1;			// use internal sense resistors
constexpr uint32_t GCONF_5130_END_COMMUTATION = 1 << 3;		// Enable commutation by full step encoder (DCIN_CFG5 = ENC_A, DCEN_CFG4 = ENC_B)

constexpr uint32_t GCONF_5160_RECAL = 1 << 0;				// Zero crossing recalibration during driver disable (via ENN or via TOFF setting)
constexpr uint32_t GCONF_5160_FASTSTANDSTILL = 1 << 1;		// Timeout for step execution until standstill detection: 1: Short time: 2^18 clocks, 0: Normal time: 2^20 clocks
constexpr uint32_t GCONF_5160_MULTISTEP_FILT = 1 << 3;		// Enable step input filtering for stealthChop optimization with external step source (default=1)

constexpr uint32_t GCONF_SPREAD_CYCLE = 1 << 2;				// use spread cycle mode (else stealthchop mode)
constexpr uint32_t GCONF_REV_DIR = 1 << 4;					// reverse motor direction
constexpr uint32_t GCONF_DIAG0_ERROR = 1 << 5;				// Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
															// DIAG0 always shows the reset-status, i.e. is active low during reset condition.
constexpr uint32_t GCONF_DIAG0_OTPW = 1 << 6;				// Enable DIAG0 active on driver over temperature prewarning (otpw)
constexpr uint32_t GCONF_DIAG0_STALL = 1 << 7;				// Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
constexpr uint32_t GCONF_DIAG1_STALL = 1 << 8;				// Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
constexpr uint32_t GCONF_DIAG1_INDEX = 1 << 9;				// Enable DIAG1 active on index position (microstep look up table position 0)
constexpr uint32_t GCONF_DIAG1_ONSTATE = 1 << 10;			// Enable DIAG1 active when chopper is on (for the coil which is in the second half of the fullstep)
constexpr uint32_t GCONF_DIAG1_STEPS_SKIPPED = 1 << 11;		// Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS). Do not enable in conjunction with other DIAG1 options.
constexpr uint32_t GCONF_DIAG0_PUSHPULL = 1 << 12;			// 0: SWN_DIAG0 is open collector output (active low), 1: Enable SWN_DIAG0 push pull output (active high)
constexpr uint32_t GCONF_DIAG1_PUSHPULL = 1 << 13;			// 0: SWN_DIAG1 is open collector output (active low), 1: Enable SWN_DIAG1 push pull output (active high)
constexpr uint32_t GCONF_SMALL_HYSTERESIS = 1 << 14;		// 0: Hysteresis for step frequency comparison is 1/16, 1: Hysteresis for step frequency comparison is 1/32
constexpr uint32_t GCONF_STOP_ENABLE = 1 << 15;				// 0: Normal operation, 1: Emergency stop: ENCA_DCIN stops the sequencer when tied high (no steps become executed by the sequencer, motor goes to standstill state)
constexpr uint32_t GCONF_DIRECT_MODE = 1 << 16;				// 0: Normal operation, 1: Motor coil currents and polarity directly programmed via serial interface:
															// Register XTARGET (0x2D) specifies signed coil A current (bits 8..0) and coil B current (bits 24..16).
															// In this mode, the current is scaled by IHOLD setting. Velocity based current regulation of stealthChop
															// is not available in this mode. The automatic stealthChop current regulation will work only for low stepper motor velocities.
constexpr uint32_t GCONF_TEST_MODE = 1 << 17;				// 0: Normal operation, 1: Enable analog test output on pin ENCN_DCO. IHOLD[1..0] selects the function of ENCN_DCO: 0…2: T120, DAC, VDDH

constexpr uint32_t DefaultGConfReg_5130 = 0;
constexpr uint32_t DefaultGConfReg_5160 = GCONF_5160_MULTISTEP_FILT;

// General configuration and status registers

// GSTAT register (0x01, RW). Write 1 bits to clear the flags.
constexpr uint8_t REGNUM_GSTAT = 0x01;
constexpr uint32_t GSTAT_RESET = 1 << 0;					// driver has been reset since last read
constexpr uint32_t GSTAT_DRV_ERR = 1 << 1;					// driver has been shut down due to over temp or short circuit
constexpr uint32_t GSTAT_UV_CP = 1 << 2;					// undervoltage on charge pump, driver disabled. Not latched so does not need to be cleared.

// IFCOUNT register (0x02, RO) is not used in SPI mode
// SLAVECONF register (0x03, WO) is not used in SPI mode
// IOIN register (0x04, RO) reads the state of all input pins. We don't use it.
// OUTPUT register (0x04, WO) is not used in SPI mode
// X_COMPARE register (0x05, WO) allows us to get a pulse on DIAG1 when an index is passed. We don't use it.
// OTP_PROG register (0x06, WO, 5160 only) is not used in this firmware
// OTP_READ register (0x07, RO, 5160 only) is not used in this firmware
// FACTORY_CONF register (0x08, RW, 5160 only) trims the clock frequency and is preset for 12MHz

// SHORT_CONF register
constexpr uint8_t REGNUM_5160_SHORTCONF = 0x09;

constexpr uint32_t SHORTCONF_S2VS_LEVEL_SHIFT = 0;
constexpr uint32_t SHORTCONF_S2VS_LEVEL_MASK = 15;			// Short to VS detector level for lowside FETs. Checks for voltage drop in LS MOSFET and sense resistor.
															// 4 (highest sensitivity) … 15 (lowest sensitivity); 10 recommended for normal operation (Reset default 12 via OTP)
															// Hint: Settings from 1 to 3 will trigger during normal operation due to voltage drop on sense resistor.
constexpr uint32_t SHORTCONF_S2G_LEVEL_SHIFT = 8;
constexpr uint32_t SHORTCONF_S2G_LEVEL_MASK = (15 << 8);	// Short to GND detector level for highside FETs. Checks for voltage drop on high side MOSFET
															// 2 (highest sensitivity) … 15 (lowest sensitivity) 6 to 10 recommended (Reset Default: 12 via OTP)
constexpr uint32_t SHORTCONF_FILTER_SHIFT = 16;
constexpr uint32_t SHORTCONF_FILTER_MASK = (3 << 16);		// Spike filtering bandwidth for short detection 0 (lowest, 100ns), 1 (1us), 2 (2us) 3 (3us)
															// Hint: A good PCB layout will allow using setting 0. Increase value, if erroneous short detection occurs. Reset Default = 1
constexpr uint32_t SHORTCONF_DELAY = (1 << 18);				// Short detection delay 0=750ns: normal, 1=1500ns: high The short detection delay shall cover the bridge switching time.
															// 0 will work for most applications. (Reset Default = 0)

// DRV_CONF register
constexpr uint8_t REGNUM_5160_DRVCONF = 0x0A;
constexpr uint32_t DRVCONF_BBMTIME_SHIFT = 0;
constexpr uint32_t DRVCONF_BBMTIME_MASK = 31;				// Break-Before make delay 0=shortest (100ns) … 16 (200ns) … 24=longest (375ns) >24 not recommended, use BBMCLKS instead
															// Hint: 0 recommended due to fast switching MOSFETs (Reset Default = 0)
constexpr uint32_t DRVCONF_BBMCLKS_SHIFT = 8;
constexpr uint32_t DRVCONF_BBMCLKS_MASK = (15 << 8);		// Digital BBM time in clock cycles (typ. 83ns). The longer setting rules (BBMTIME vs. BBMCLKS).
															// Reset Default: 2 via OTP. Hint: 2, or down to 0 recommended due to fast switching MOSFETs
constexpr uint32_t DRVCONF_OTSELECT_SHIFT = 16;
constexpr uint32_t DRVCONF_OTSELECT_MASK = (3 << 16);		// Selection of over temperature level for bridge disable, switch on after cool down to 120°C / OTPW level. Reset Default = 0.
															// 00: 150°C (not recommended – MOSFET might overheat); 01: 143°C 10: 136°C (Recommended); 11: 120°C (not recommended, no hysteresis)
															// Hint: Adapt overtemperature threshold as required to protect the MOSFETs or other components on the PCB.
constexpr uint32_t DRVCONF_STRENGTH_SHIFT = 18;
constexpr uint32_t DRVCONF_STRENGTH_MASK = (3 << 18);		// Selection of gate driver current. Adapts the gate driver current to the gate charge of the external MOSFETs.
															// 00: Normal slope (Recommended) 01: Normal+TC (medium above OTPW level) 10: Fast slope. Reset Default = 10.
constexpr uint32_t DRVCONF_FILT_ISENSE_SHIFT = 20;
constexpr uint32_t DRVCONF_FILT_ISENSE_MASK = (3 << 20);	// Filter time constant of sense amplifier to suppress ringing and coupling from second coil operation
															// 00: low – 100ns 01: – 200ns 10: – 300ns 11: high – 400ns
															// Hint: Increase setting if motor chopper noise occurs due to cross-coupling of both coils. Reset Default = 0.

constexpr uint8_t REGNUM_5160_GLOBAL_SCALER = 0x0B;			// Global scaling of Motor current. This value is multiplied to the current scaling in order to adapt a drive to a
															// certain motor type. This value should be chosen before tuning other settings, because it also influences chopper hysteresis.
															// 0: Full Scale (or write 256) 1 … 31: Not allowed for operation 32 … 255: 32/256 … 255/256 of maximum current.
															// Hint: Values >128 recommended for best results. Reset Default 0.

constexpr uint8_t REGNUM_5160_OFFSET_READ = 0x0B;			// Bits 8..15: Offset calibration result phase A (signed). Bits 0..7: Offset calibration result phase B (signed).

// Velocity dependent control registers

// IHOLD_IRUN register (WO)
constexpr uint8_t REGNUM_IHOLDIRUN = 0x10;
constexpr uint32_t IHOLDIRUN_IHOLD_SHIFT = 0;				// standstill current
constexpr uint32_t IHOLDIRUN_IHOLD_MASK = 0x1F << IHOLDIRUN_IHOLD_SHIFT;
constexpr uint32_t IHOLDIRUN_IRUN_SHIFT = 8;
constexpr uint32_t IHOLDIRUN_IRUN_MASK = 0x1F << IHOLDIRUN_IRUN_SHIFT;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_SHIFT = 16;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_MASK = 0x0F << IHOLDIRUN_IHOLDDELAY_SHIFT;

constexpr uint32_t DefaultIholdIrunReg = (0 << IHOLDIRUN_IHOLD_SHIFT) | (0 << IHOLDIRUN_IRUN_SHIFT) | (2 << IHOLDIRUN_IHOLDDELAY_SHIFT);
															// approx. 0.5 sec motor current reduction to half power

constexpr uint8_t REGNUM_TPOWER_DOWN = 0x11;
constexpr uint8_t REGNUM_TSTEP = 0x12;
constexpr uint8_t REGNUM_TPWMTHRS = 0x13;
constexpr uint8_t REGNUM_VACTUAL = 0x22;

// Sequencer registers (read only)
constexpr uint8_t REGNUM_MSCNT = 0x6A;
constexpr uint8_t REGNUM_MSCURACT = 0x6B;

// Chopper control registers

// CHOPCONF register
constexpr uint8_t REGNUM_CHOPCONF = 0x6C;
constexpr uint32_t CHOPCONF_TOFF_SHIFT = 0;					// off time setting, 0 = disable driver
constexpr uint32_t CHOPCONF_TOFF_MASK = 0x0F << CHOPCONF_TOFF_SHIFT;
constexpr uint32_t CHOPCONF_HSTRT_SHIFT = 4;				// hysteresis start
constexpr uint32_t CHOPCONF_HSTRT_MASK = 0x07 << CHOPCONF_HSTRT_SHIFT;
constexpr uint32_t CHOPCONF_HEND_SHIFT = 7;					// hysteresis end
constexpr uint32_t CHOPCONF_HEND_MASK = 0x0F << CHOPCONF_HEND_SHIFT;
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;					// blanking time
constexpr uint32_t CHOPCONF_TBL_MASK = 0x03 << CHOPCONF_TBL_SHIFT;
constexpr uint32_t CHOPCONF_VSENSE_HIGH = 1 << 17;			// use high sensitivity current scaling
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;				// microstep resolution
constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F << CHOPCONF_MRES_SHIFT;
constexpr uint32_t CHOPCONF_INTPOL = 1 << 28;				// use interpolation
constexpr uint32_t CHOPCONF_DEDGE = 1 << 29;				// step on both edges
constexpr uint32_t CHOPCONF_DISS2G = 1 << 30;				// disable short to ground protection
constexpr uint32_t CHOPCONF_DISS2VS = 1 << 31;				// disable low side short protection

constexpr uint32_t DefaultChopConfReg = 0x10000053 | CHOPCONF_VSENSE_HIGH;	// this is the reset default + CHOPCONF_VSENSE_HIGH - try it until we find something better

// DRV_STATUS register. See the .h file for the bit definitions.
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC10D0024;			// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

// Common data
static size_t numTmc51xxDrivers;

enum class DriversState : uint8_t
{
	noPower = 0,
	initialising,
	ready
};

static DriversState driversState = DriversState::noPower;

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class TmcDriverState
{
public:
	void Init(uint32_t p_driverNumber, Pin p_pin);
	void SetAxisNumber(size_t p_axisNumber);
	uint32_t GetAxisNumber() const { return axisNumber; }
	void WriteAll();
	bool SetMicrostepping(uint32_t shift, bool interpolate);
	unsigned int GetMicrostepping(bool& interpolation) const;		// Get microstepping
	bool SetDriverMode(unsigned int mode);
	DriverMode GetDriverMode() const;
	void SetCurrent(float current);
	void Enable(bool en);
	void AppendDriverStatus(const StringRef& reply);
	uint8_t GetDriverNumber() const { return driverNumber; }
	bool UpdatePending() const { return registersToUpdate != 0; }
	void SetStallDetectThreshold(int sgThreshold);
	void SetStallDetectFilter(bool sgFilter);
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond);
	void AppendStallConfig(const StringRef& reply) const;

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal);
	uint32_t GetRegister(SmartDriverRegister reg) const;

	float GetStandstillCurrentPercent() const;
	void SetStandstillCurrentPercent(float percent);

	void TransferDone() __attribute__ ((hot));				// called by the ISR when the SPI transfer has completed
	void StartTransfer() __attribute__ ((hot));				// called to start a transfer
	void TransferTimedOut() { ++numTimeouts; }
	void AbortTransfer();

	uint32_t ReadLiveStatus() const;
	uint32_t ReadAccumulatedStatus(uint32_t bitsToKeep);

	// Variables used by the ISR
	static TmcDriverState * volatile currentDriver;			// volatile because the ISR changes it
	static uint32_t transferStartedTime;

	void UartTmcHandler();									// core of the ISR for this driver

private:
	void UpdateRegister(size_t regIndex, uint32_t regVal);
	void UpdateChopConfRegister();							// calculate the chopper control register and flag it for sending
	void UpdateCurrent();

#if TMC22xx_HAS_MUX
	void SetUartMux();
	static void SetupDMASend(uint8_t regnum, uint32_t outVal, uint8_t crc) __attribute__ ((hot));	// set up the PDC to send a register
	static void SetupDMAReceive(uint8_t regnum, uint8_t crc) __attribute__ ((hot));					// set up the PDC to receive a register
#else
	void SetupDMASend(uint8_t regnum, uint32_t outVal, uint8_t crc) __attribute__ ((hot));			// set up the PDC to send a register
	void SetupDMAReceive(uint8_t regnum, uint8_t crc) __attribute__ ((hot));						// set up the PDC to receive a register
#endif

	static constexpr unsigned int NumWriteRegisters = 6;	// the number of registers that we write to
	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteGConf = 0;			// microstepping
	static constexpr unsigned int WriteSlaveConf = 1;		// read response timing
	static constexpr unsigned int WriteChopConf = 2;		// enable/disable and microstep setting
	static constexpr unsigned int WriteIholdIrun = 3;		// current setting
	static constexpr unsigned int WritePwmConf = 4;			// read register select, sense voltage high/low sensitivity
	static constexpr unsigned int WriteTpwmthrs = 5;		// upper step rate limit for stealthchop

	static constexpr unsigned int NumReadRegisters = 2;		// the number of registers that we read from
	static const uint8_t ReadRegNumbers[NumReadRegisters];	// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadGStat = 0;
	static constexpr unsigned int ReadDrvStat = 1;

	volatile uint32_t writeRegisters[NumWriteRegisters];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters];		// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedReadRegisters[NumReadRegisters];

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	volatile uint32_t registersToUpdate;					// bitmap of register indices whose values need to be sent to the driver chip
	volatile uint32_t registerBeingUpdated;					// which register we are sending

	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	uint32_t motorCurrent;									// the configured motor current

#if TMC22xx_HAS_MUX
	static Uart * const uart;								// the UART that controls all drivers
#else
	Uart *uart;												// the UART that controls this driver
#endif

	// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
	// This is the message we send - volatile because we care about when it is written
	static volatile uint8_t sendData[12];

	// Buffer for the message we receive when reading data. The first 4 or 12 bytes bytes are our own transmitted data.
	static volatile uint8_t receiveData[20];

	uint16_t readErrors;									// how many read errors we had
	uint16_t writeErrors;									// how many write errors we had
	uint16_t numReads;										// how many successful reads we had
	uint16_t numTimeouts;									// how many times a transfer timed out

	Pin enablePin;											// the enable pin of this driver, if it has its own
	uint8_t driverNumber;									// the number of this driver as addressed by the UART multiplexer
	uint8_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction
	uint8_t registerToRead;									// the next register we need to read
	uint8_t lastIfCount;									// the value of the IFCNT register last time we read it
	volatile uint8_t writeRegCRCs[NumWriteRegisters];		// CRCs of the messages needed to update the registers
	static const uint8_t ReadRegCRCs[NumReadRegisters];		// CRCs of the messages needed to read the registers
	bool enabled;											// true if driver is enabled
};

// State structures for all drivers
static TmcDriverState driverStates[MaxSmartDrivers];

namespace SmartDrivers
{
	// Initialise the driver interface and the drivers, leaving each drive disabled.
	// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
	void Init(const Pin driverSelectPins[DRIVES], size_t numTmcDrivers)
	{
		numTmc51xxDrivers = min<size_t>(numTmcDrivers, MaxSmartDrivers);

		// Make sure the ENN pins are high
		pinMode(GlobalTmc51xxEnablePin, OUTPUT_HIGH);

		// The pins are already set up for SPI in the pins table
		ConfigurePin(GetPinDescription(TMC51xxMosiPin));
		ConfigurePin(GetPinDescription(TMC51xxMisoPin));
		ConfigurePin(GetPinDescription(TMC51xxSclkPin));

		// Enable the clock to the USART or SPI
		pmc_enable_periph_clk(ID_TMC51xx_SPI);

#if TMC51xx_USES_USART
		// Set USART_EXT_DRV in SPI mode, with data changing on the falling edge of the clock and captured on the rising edge
		USART_TMC51xx->US_IDR = ~0u;
		USART_TMC51xx->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		USART_TMC51xx->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		USART_TMC51xx->US_BRGR = VARIANT_MCK/DriversSpiClockFrequency;		// set SPI clock frequency
		USART_TMC51xx->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

		// We need a few microseconds of delay here for the USART to sort itself out before we send any data,
		// otherwise the processor generates two short reset pulses on its own NRST pin, and resets itself.
		// 2016-07-07: removed this delay, because we no longer send commands to the TMC2660 drivers immediately.
		//delay(10);
#else
		// Set up the SPI interface with data changing on the falling edge of the clock and captured on the rising edge
		spi_reset(SPI_TMC51xx);										// this clears the transmit and receive registers and puts the SPI into slave mode
		SPI_TMC51xx->SPI_MR = SPI_MR_MSTR							// master mode
						| SPI_MR_MODFDIS							// disable fault detection
						| SPI_MR_PCS(0);							// fixed peripheral select

		// Set SPI mode, clock frequency, CS active after transfer, delay between transfers
		const uint16_t baud_div = (uint16_t)spi_calc_baudrate_div(DriversSpiClockFrequency, SystemCoreClock);
		const uint32_t csr = SPI_CSR_SCBR(baud_div)					// Baud rate
						| SPI_CSR_BITS_8_BIT						// Transfer bit width
						| SPI_CSR_DLYBCT(0)      					// Transfer delay
						| SPI_CSR_CSAAT								// Keep CS low after transfer in case we are slow in writing the next byte
						| SPI_CSR_CPOL;								// clock high between transfers
		SPI_TMC51xx->SPI_CSR[0] = csr;
#endif

		driversState = DriversState::noPower;
		for (size_t driver = 0; driver < numTmc51xxDrivers; ++driver)
		{
			driverStates[driver].Init(driver, driverSelectPins[driver]);		// axes are mapped straight through to drivers initially
		}

#if SAME70
		pmc_enable_periph_clk(ID_XDMAC);
#endif
	}

	void SetAxisNumber(size_t driver, uint32_t axisNumber)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].SetAxisNumber(axisNumber);
		}
	}

	uint32_t GetAxisNumber(size_t drive)
	{
		return (drive < numTmc51xxDrivers) ? driverStates[drive].GetAxisNumber() : 0;
	}

	void SetCurrent(size_t driver, float current)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].SetCurrent(current);
		}
	}

	void EnableDrive(size_t driver, bool en)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].Enable(en);
		}
	}

	uint32_t GetLiveStatus(size_t driver)
	{
		return (driver < numTmc51xxDrivers) ? driverStates[driver].ReadLiveStatus() : 0;
	}

	uint32_t GetAccumulatedStatus(size_t driver, uint32_t bitsToKeep)
	{
		return (driver < numTmc51xxDrivers) ? driverStates[driver].ReadAccumulatedStatus(bitsToKeep) : 0;
	}

	// Set microstepping and microstep interpolation
	bool SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate)
	{
		if (driver < numTmc51xxDrivers && microsteps > 0)
		{
			// Set the microstepping. We need to determine how many bits right to shift the desired microstepping to reach 1.
			unsigned int shift = 0;
			unsigned int uSteps = (unsigned int)microsteps;
			while ((uSteps & 1) == 0)
			{
				uSteps >>= 1;
				++shift;
			}
			if (uSteps == 1 && shift <= 8)
			{
				driverStates[driver].SetMicrostepping(shift, interpolate);
				return true;
			}
		}
		return false;
	}

	// Get microstepping and interpolation
	unsigned int GetMicrostepping(size_t driver, bool& interpolation)
	{
		if (driver < numTmc51xxDrivers)
		{
			return driverStates[driver].GetMicrostepping(interpolation);
		}
		interpolation = false;
		return 1;
	}

	bool SetDriverMode(size_t driver, unsigned int mode)
	{
		return driver < numTmc51xxDrivers && driverStates[driver].SetDriverMode(mode);
	}

	DriverMode GetDriverMode(size_t driver)
	{
		return (driver < numTmc51xxDrivers) ? driverStates[driver].GetDriverMode() : DriverMode::unknown;
	}

	// Flag that the the drivers have been powered up or down and handle any timeouts
	// Before the first call to this function with 'powered' true, you must call Init()
	void Spin(bool powered)
	{
		if (driversState == DriversState::noPower)
		{
			if (powered)
			{
				// Power to the drivers has been provided or restored, so we need to enable and re-initialise them
				for (size_t drive = 0; drive < numTmc51xxDrivers; ++drive)
				{
					driverStates[drive].WriteAll();
				}
				driversState = DriversState::initialising;
			}
		}
		else if (powered)
		{
			// If no transfer is in progress, kick one off.
			// If a transfer has timed out, abort it.
			if (TmcDriverState::currentDriver == nullptr)
			{
				// No transfer in progress, so start one
				if (numTmc51xxDrivers != 0)
				{
					// Kick off the first transfer
					driverStates[0].StartTransfer();
				}
			}
			else if (millis() - TmcDriverState::transferStartedTime > TransferTimeout)
			{
				// A UART transfer was started but has timed out
				TmcDriverState::currentDriver->TransferTimedOut();
				TmcDriverState::currentDriver->AbortTransfer();
				uint8_t driverNum = TmcDriverState::currentDriver->GetDriverNumber();
				TmcDriverState::currentDriver = nullptr;

				++driverNum;
				if (driverNum >= numTmc51xxDrivers)
				{
					driverNum = 0;
				}
				driverStates[driverNum].StartTransfer();
			}

			if (driversState == DriversState::initialising)
			{
				// If all drivers that share the global enable have been initialised, set the global enable
				bool allInitialised = true;
				for (size_t i = 0; i < numTmc51xxDrivers; ++i)
				{
					if (driverStates[i].UpdatePending())
					{
						allInitialised = false;
						break;
					}
				}

				if (allInitialised)
				{
					digitalWrite(GlobalTmc51xxEnablePin, LOW);
					driversState = DriversState::ready;
				}
			}
		}
		else
		{
			// We had power but we lost it
			digitalWrite(GlobalTmc51xxEnablePin, HIGH);			// disable the drivers
			if (TmcDriverState::currentDriver == nullptr)
			{
				TmcDriverState::currentDriver->AbortTransfer();
				TmcDriverState::currentDriver = nullptr;
			}
			driversState = DriversState::noPower;
		}
	}


	// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
	void TurnDriversOff()
	{
		digitalWrite(GlobalTmc51xxEnablePin, HIGH);				// disable the drivers
		driversState = DriversState::noPower;
	}

	void SetStallThreshold(size_t driver, int sgThreshold)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].SetStallDetectThreshold(sgThreshold);
		}
	}

	void SetStallFilter(size_t driver, bool sgFilter)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].SetStallDetectFilter(sgFilter);
		}
	}

	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].SetStallMinimumStepsPerSecond(stepsPerSecond);
		}
	}

	void AppendStallConfig(size_t driver, const StringRef& reply)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].AppendStallConfig(reply);
		}
	}

	void AppendDriverStatus(size_t driver, const StringRef& reply)
	{
		if (driver < numTmc51xxDrivers)
		{
			driverStates[driver].AppendDriverStatus(reply);
		}
	}

	float GetStandstillCurrentPercent(size_t driver)
	{
		return 100.0;			// not supported
	}

	void SetStandstillCurrentPercent(size_t driver, float percent)
	{
		// not supported so nothing to see here
	}

	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal)
	{
		return (driver < numTmc51xxDrivers) && driverStates[driver].SetRegister(reg, regVal);
	}

	uint32_t GetRegister(size_t driver, SmartDriverRegister reg)
	{
		return (driver < numTmc51xxDrivers) ? driverStates[driver].GetRegister(reg) : 0;
	}

};	// end namespace

#endif

// End
