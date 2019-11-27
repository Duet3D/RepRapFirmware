/*
 * TMC51xx.cpp
 *
 *  Created on: 26 Aug 2018
 *      Author: David
 *  Purpose:
 *  	Support for TMC5130 and TMC5160 stepper drivers
 */

#include "TMC51xx.h"

#if SUPPORT_TMC51xx

#error This file has not been kept up to date since Duet 3 development moved to RRF3. In particular, it does not allow for interactons between DMA and cache.

#include "RTOSIface/RTOSIface.h"
#include "Movement/Move.h"

#ifdef SAME51
# include "HAL/IoPorts.h"
# include "HAL/DmacManager.h"
# include "peripheral_clk_config.h"
# include "HAL/SAME5x.h"
#elif SAME70
# include "Hardware/DmacManager.h"
#endif

//#define TMC_TYPE	5130
#define TMC_TYPE	5160

constexpr float MinimumMotorCurrent = 50.0;
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
const int DefaultStallDetectThreshold = 1;
const bool DefaultStallDetectFiltered = false;
const unsigned int DefaultMinimumStepsPerSecond = 200;		// for stall detection: 1 rev per second assuming 1.8deg/step, as per the TMC2660 datasheet
const uint32_t DefaultTcoolthrs = 2000;						// max interval between 1/256 microsteps for stall detection to be enabled
const uint32_t DefaultThigh = 200;

#if TMC_TYPE == 5130
constexpr float MaximumMotorCurrent = 1600.0;
constexpr float SenseResistor = 0.11;						// 0.082R external + 0.03 internal
#elif TMC_TYPE == 5160
constexpr float MaximumMotorCurrent = 3200.0;				// depends on sense resistor power rating
constexpr float SenseResistor = 0.051;						// assume same as we use for TMC2660
constexpr float FullScaleCurrent = 325.0/SenseResistor;		// full scale current in mA
#endif

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

constexpr uint32_t GCONF_STEALTHCHOP = 1 << 2;				// use stealthchop mode (else spread cycle mode)
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

#if TMC_TYPE == 5130
constexpr uint32_t DefaultGConfReg = GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;
#elif TMC_TYPE == 5160
constexpr uint32_t DefaultGConfReg = GCONF_5160_RECAL | GCONF_5160_MULTISTEP_FILT | GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;
#endif

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

#if TMC_TYPE == 5160

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
constexpr uint32_t DefaultShortConfReg = (10 << SHORTCONF_S2VS_LEVEL_SHIFT) | (6 << SHORTCONF_S2G_LEVEL_SHIFT) | (0 << SHORTCONF_FILTER_SHIFT);

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
															// 00: Normal slope (Recommended), 01: Normal+TC (medium above OTPW level), 10: Fast slope. Reset Default = 10.
constexpr uint32_t DRVCONF_FILT_ISENSE_SHIFT = 20;
constexpr uint32_t DRVCONF_FILT_ISENSE_MASK = (3 << 20);	// Filter time constant of sense amplifier to suppress ringing and coupling from second coil operation
															// 00: low – 100ns 01: – 200ns 10: – 300ns 11: high – 400ns
															// Hint: Increase setting if motor chopper noise occurs due to cross-coupling of both coils. Reset Default = 0.
constexpr uint32_t DefaultDrvConfReg = (2 << DRVCONF_BBMCLKS_SHIFT) | (2 << DRVCONF_OTSELECT_SHIFT);

constexpr uint8_t REGNUM_5160_GLOBAL_SCALER = 0x0B;			// Global scaling of Motor current. This value is multiplied to the current scaling in order to adapt a drive to a
															// certain motor type. This value should be chosen before tuning other settings, because it also influences chopper hysteresis.
															// 0: Full Scale (or write 256) 1 … 31: Not allowed for operation 32 … 255: 32/256 … 255/256 of maximum current.
															// Hint: Values >128 recommended for best results. Reset Default 0.
constexpr uint32_t DefaultGlobalScalerReg = 0;				// until we use it as part of the current setting

constexpr uint8_t REGNUM_5160_OFFSET_READ = 0x0B;			// Bits 8..15: Offset calibration result phase A (signed). Bits 0..7: Offset calibration result phase B (signed).

#endif

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

constexpr uint8_t REGNUM_TCOOLTHRS = 0x14;
const uint32_t DefaultTcoolthrsReg = DefaultTcoolthrs;

constexpr uint8_t REGNUM_THIGH = 0x15;
const uint32_t DefaultThighReg = DefaultThigh;

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
constexpr uint32_t CHOPCONF_5130_RNDTOFF = 1 << 13;			// random off time
constexpr uint32_t CHOPCONF_CHM = 1 << 14;					// fixed off time
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;					// blanking time
constexpr uint32_t CHOPCONF_TBL_MASK = 0x03 << CHOPCONF_TBL_SHIFT;
constexpr uint32_t CHOPCONF_5130_VSENSE_HIGH = 1 << 17;		// use high sensitivity current scaling
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;				// microstep resolution
constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F << CHOPCONF_MRES_SHIFT;
constexpr uint32_t CHOPCONF_INTPOL = 1 << 28;				// use interpolation
constexpr uint32_t CHOPCONF_DEDGE = 1 << 29;				// step on both edges
constexpr uint32_t CHOPCONF_DISS2G = 1 << 30;				// disable short to ground protection
constexpr uint32_t CHOPCONF_DISS2VS = 1 << 31;				// disable low side short protection

#if TMC_TYPE == 5130
constexpr uint32_t DefaultChopConfReg = (1 << CHOPCONF_TBL_SHIFT) | (3 << CHOPCONF_TOFF_SHIFT) | (5 << CHOPCONF_HSTRT_SHIFT) | CHOPCONF_5130_VSENSE_HIGH;
#elif TMC_TYPE == 5160
constexpr uint32_t DefaultChopConfReg = (1 << CHOPCONF_TBL_SHIFT) | (3 << CHOPCONF_TOFF_SHIFT) | (5 << CHOPCONF_HSTRT_SHIFT);
#endif

constexpr uint8_t REGNUM_COOLCONF = 0x6D;
constexpr uint32_t COOLCONF_SGFILT = 1 << 24;				// set to update stallGuard status every 4 full steps instead of every full step
constexpr uint32_t COOLCONF_SGT_SHIFT = 16;
constexpr uint32_t COOLCONF_SGT_MASK = 128 << COOLCONF_SGT_SHIFT;	// stallguard threshold (signed)

constexpr uint32_t DefaultCoolConfReg = 0;

// DRV_STATUS register. See the .h file for the bit definitions.
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC10D0024;			// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

// Common data
static constexpr size_t numTmc51xxDrivers = MaxSmartDrivers;

enum class DriversState : uint8_t
{
	noPower = 0,			// no VIN power
	notInitialised,			// have VIN power but not started initialising drivers
	initialising,			// in the process of initialising the drivers
	ready					// drivers are initialised and ready
};

static DriversState driversState = DriversState::noPower;

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class TmcDriverState
{
public:
	void Init(uint32_t p_driverNumber);
	void SetAxisNumber(size_t p_axisNumber);
	uint32_t GetAxisNumber() const { return axisNumber; }
	void WriteAll();
	bool SetMicrostepping(uint32_t shift, bool interpolate);
	unsigned int GetMicrostepping(bool& interpolation) const;		// Get microstepping
	bool SetDriverMode(unsigned int mode);
	DriverMode GetDriverMode() const;
	void SetCurrent(float current);
	void Enable(bool en);
	void AppendDriverStatus(const StringRef& reply, bool clearGlobalStats);
	uint8_t GetDriverNumber() const { return driverNumber; }
	bool UpdatePending() const { return (registersToUpdate | newRegistersToUpdate) != 0; }
	void SetStallDetectThreshold(int sgThreshold);
	void SetStallDetectFilter(bool sgFilter);
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond);
	void AppendStallConfig(const StringRef& reply) const;

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal);
	uint32_t GetRegister(SmartDriverRegister reg) const;

	float GetStandstillCurrentPercent() const;
	void SetStandstillCurrentPercent(float percent);

	static void TransferTimedOut() { ++numTimeouts; }

	uint32_t ReadLiveStatus() const;
	uint32_t ReadAccumulatedStatus(uint32_t bitsToKeep);

	void GetSpiCommand(uint8_t *sendDdataBlock);
	void TransferSucceeded(const uint8_t *rcvDataBlock);
	void TransferFailed();

private:
	bool SetChopConf(uint32_t newVal);
	void UpdateRegister(size_t regIndex, uint32_t regVal);
	void UpdateChopConfRegister();							// calculate the chopper control register and flag it for sending
	void UpdateCurrent();

	void ResetLoadRegisters()
	{
		minSgLoadRegister = 1023;
		maxSgLoadRegister = 0;
	}

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteGConf = 0;			// microstepping
	static constexpr unsigned int WriteIholdIrun = 1;		// current setting
	static constexpr unsigned int WriteTpwmthrs = 2;		// upper step rate limit for stealthchop
	static constexpr unsigned int WriteTcoolthrs = 3;		// lower velocity for coolStep and stallGuard
	static constexpr unsigned int WriteThigh = 4;			// upper velocity for coolStep and stealthChop
	static constexpr unsigned int WriteChopConf = 5;		// chopper control
	static constexpr unsigned int WriteCoolConf = 6;		// coolstep control
	static constexpr unsigned int WritePwmConf = 7;			// stealthchop and freewheel control
#if TMC_TYPE == 5160
	static constexpr unsigned int Write5160ShortConf = 8;	// short circuit detection configuration
	static constexpr unsigned int Write5160DrvConf = 9;		// driver timing
	static constexpr unsigned int Write5160GlobalScaler = 10; // motor current scaling

	static constexpr unsigned int NumWriteRegisters = 11;	// the number of registers that we write to
#else
	static constexpr unsigned int NumWriteRegisters = 8;	// the number of registers that we write to
#endif

	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	static constexpr unsigned int NumReadRegisters = 2;		// the number of registers that we read from
	static const uint8_t ReadRegNumbers[NumReadRegisters];	// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadGStat = 0;
	static constexpr unsigned int ReadDrvStat = 1;

	volatile uint32_t writeRegisters[NumWriteRegisters];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters];		// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedReadRegisters[NumReadRegisters];

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	uint32_t maxStallStepInterval;							// maximum interval between full steps to take any notice of stall detection
	uint32_t minSgLoadRegister;								// the minimum value of the StallGuard bits we read
	uint32_t maxSgLoadRegister;								// the maximum value of the StallGuard bits we read

	volatile uint32_t newRegistersToUpdate;					// bitmap of register indices whose values need to be sent to the driver chip
	uint32_t registersToUpdate;								// bitmap of register indices whose values need to be sent to the driver chip

	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	uint32_t motorCurrent;									// the configured motor current in mA

	uint16_t numReads, numWrites;							// how many successful reads and writes we had
	static uint16_t numTimeouts;							// how many times a transfer timed out

	uint8_t driverNumber;									// the number of this driver as addressed by the UART multiplexer
	uint8_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction
	uint8_t regIndexBeingUpdated;							// which register we are sending
	uint8_t regIndexRequested;								// the register we asked to read in the previous transaction, or 0xFF
	uint8_t previousRegIndexRequested;						// the register we asked to read in the previous transaction, or 0xFF
	bool enabled;											// true if driver is enabled
};

const uint8_t TmcDriverState::WriteRegNumbers[NumWriteRegisters] =
{
	REGNUM_GCONF,
	REGNUM_IHOLDIRUN,
	REGNUM_TPWMTHRS,
	REGNUM_TCOOLTHRS,
	REGNUM_THIGH,
	REGNUM_CHOPCONF,
	REGNUM_COOLCONF,
	REGNUM_PWMCONF,
#if TMC_TYPE == 5160
	REGNUM_5160_SHORTCONF,
	REGNUM_5160_DRVCONF,
	REGNUM_5160_GLOBAL_SCALER
#endif
};

const uint8_t TmcDriverState::ReadRegNumbers[NumReadRegisters] =
{
	REGNUM_GSTAT,
	REGNUM_DRV_STATUS
};

uint16_t TmcDriverState::numTimeouts = 0;							// how many times a transfer timed out

// Initialise the state of the driver and its CS pin
void TmcDriverState::Init(uint32_t p_axisNumber)
pre(!driversPowered)
{
	axisNumber = p_axisNumber;
	enabled = false;
	registersToUpdate = newRegistersToUpdate = 0;
	motorCurrent = 0;
	standstillCurrentFraction = (256 * 3)/4;							// default to 75%

	// Set default values for all registers and flag them to be updated
	UpdateRegister(WriteGConf, DefaultGConfReg);
#if TMC_TYPE == 5160
	UpdateRegister(Write5160ShortConf, DefaultShortConfReg);
	UpdateRegister(Write5160DrvConf, DefaultDrvConfReg);
	UpdateRegister(Write5160GlobalScaler, DefaultGlobalScalerReg);
#endif
	UpdateRegister(WriteIholdIrun, DefaultIholdIrunReg);
	UpdateRegister(WriteTpwmthrs, DefaultTpwmthrsReg);
	UpdateRegister(WriteTcoolthrs, DefaultTcoolthrsReg);
	UpdateRegister(WriteThigh, DefaultThighReg);
	configuredChopConfReg = DefaultChopConfReg;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);	// this also updates the chopper control register
	writeRegisters[WriteCoolConf] = DefaultCoolConfReg;
	SetStallDetectThreshold(DefaultStallDetectThreshold);				// this also updates the CoolConf register
	SetStallMinimumStepsPerSecond(DefaultMinimumStepsPerSecond);
	UpdateRegister(WritePwmConf, DefaultPwmConfReg);

	for (size_t i = 0; i < NumReadRegisters; ++i)
	{
		accumulatedReadRegisters[i] = readRegisters[i] = 0;
	}

	regIndexBeingUpdated = regIndexRequested = previousRegIndexRequested = 0xFF;
	numReads = numWrites = 0;
}

// Set a register value and flag it for updating
void TmcDriverState::UpdateRegister(size_t regIndex, uint32_t regVal)
{
	writeRegisters[regIndex] = regVal;
	newRegistersToUpdate |= (1u << regIndex);								// flag it for sending
}

// Calculate the chopper control register and flag it for sending
void TmcDriverState::UpdateChopConfRegister()
{
	UpdateRegister(WriteChopConf, (enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK);
}

void TmcDriverState::SetStallDetectThreshold(int sgThreshold)
{
	const uint32_t sgVal = ((uint32_t)constrain<int>(sgThreshold, -64, 63)) & 127;
	writeRegisters[WriteCoolConf] = (writeRegisters[WriteCoolConf] & ~COOLCONF_SGT_MASK) | (sgVal << COOLCONF_SGT_SHIFT);
	newRegistersToUpdate |= 1u << WriteCoolConf;
}

inline void TmcDriverState::SetAxisNumber(size_t p_axisNumber)
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void TmcDriverState::WriteAll()
{
	newRegistersToUpdate = (1u << NumWriteRegisters) - 1;
}

float TmcDriverState::GetStandstillCurrentPercent() const
{
	return (float)(standstillCurrentFraction * 100)/256;
}

void TmcDriverState::SetStandstillCurrentPercent(float percent)
{
	standstillCurrentFraction = (uint8_t)constrain<long>(lrintf((percent * 256)/100), 0, 255);
	UpdateCurrent();
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift).
bool TmcDriverState::SetMicrostepping(uint32_t shift, bool interpolate)
{
	microstepShiftFactor = shift;
	configuredChopConfReg = (configuredChopConfReg & ~(CHOPCONF_MRES_MASK | CHOPCONF_INTPOL)) | ((8 - shift) << CHOPCONF_MRES_SHIFT);
	if (interpolate)
	{
		configuredChopConfReg |= CHOPCONF_INTPOL;
	}
	UpdateChopConfRegister();
	return true;
}

// Get microstepping or chopper control register
unsigned int TmcDriverState::GetMicrostepping(bool& interpolation) const
{
	interpolation = (writeRegisters[WriteChopConf] & CHOPCONF_INTPOL) != 0;
	return 1u << microstepShiftFactor;
}

bool TmcDriverState::SetRegister(SmartDriverRegister reg, uint32_t regVal)
{
	switch(reg)
	{
	case SmartDriverRegister::chopperControl:
		return SetChopConf(regVal);

	case SmartDriverRegister::toff:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_TOFF_MASK) | ((regVal << CHOPCONF_TOFF_SHIFT) & CHOPCONF_TOFF_MASK));

	case SmartDriverRegister::tblank:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_TBL_MASK) | ((regVal << CHOPCONF_TBL_SHIFT) & CHOPCONF_TBL_MASK));

	case SmartDriverRegister::hstart:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_HSTRT_MASK) | ((regVal << CHOPCONF_HSTRT_SHIFT) & CHOPCONF_HSTRT_MASK));

	case SmartDriverRegister::hend:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_HEND_MASK) | ((regVal << CHOPCONF_HEND_SHIFT) & CHOPCONF_HEND_MASK));

	case SmartDriverRegister::tpwmthrs:
		UpdateRegister(WriteTpwmthrs, regVal & ((1u << 20) - 1));
		return true;

	case SmartDriverRegister::thigh:
		UpdateRegister (WriteThigh, regVal & ((1u << 20) - 1));
		return true;

	case SmartDriverRegister::coolStep:
		UpdateRegister (WriteTcoolthrs, regVal & ((1u << 20) - 1));
		return true;

	case SmartDriverRegister::hdec:
	default:
		return false;
	}
}

uint32_t TmcDriverState::GetRegister(SmartDriverRegister reg) const
{
	switch(reg)
	{
	case SmartDriverRegister::chopperControl:
		return configuredChopConfReg & 0x01FFFF;

	case SmartDriverRegister::toff:
		return (configuredChopConfReg & CHOPCONF_TOFF_MASK) >> CHOPCONF_TOFF_SHIFT;

	case SmartDriverRegister::tblank:
		return (configuredChopConfReg & CHOPCONF_TBL_MASK) >> CHOPCONF_TBL_SHIFT;

	case SmartDriverRegister::hstart:
		return (configuredChopConfReg & CHOPCONF_HSTRT_MASK) >> CHOPCONF_HSTRT_SHIFT;

	case SmartDriverRegister::hend:
		return (configuredChopConfReg & CHOPCONF_HEND_MASK) >> CHOPCONF_HEND_SHIFT;

	case SmartDriverRegister::tpwmthrs:
		return writeRegisters[WriteTpwmthrs];

	case SmartDriverRegister::thigh:
		return writeRegisters[WriteThigh];

	case SmartDriverRegister::coolStep:
		return writeRegisters[WriteTcoolthrs];

	case SmartDriverRegister::hdec:
	default:
		return 0;
	}
}

// Set the chopper control register to the settings provided by the user. We allow only the lowest 17 bits to be set.
bool TmcDriverState::SetChopConf(uint32_t newVal)
{
	const uint32_t offTime = (newVal & CHOPCONF_TOFF_MASK) >> CHOPCONF_TOFF_SHIFT;
	if (offTime == 0 || (offTime == 1 && (newVal & CHOPCONF_TBL_MASK) < (2 << CHOPCONF_TBL_SHIFT)))
	{
		return false;
	}
	const uint32_t hstrt = (newVal & CHOPCONF_HSTRT_MASK) >> CHOPCONF_HSTRT_SHIFT;
	const uint32_t hend = (newVal & CHOPCONF_HEND_MASK) >> CHOPCONF_HEND_SHIFT;
	if (hstrt + hend > 16)
	{
		return false;
	}
	const uint32_t userMask = CHOPCONF_TBL_MASK | CHOPCONF_HSTRT_MASK | CHOPCONF_HEND_MASK | CHOPCONF_TOFF_MASK;	// mask of bits the user is allowed to change
	configuredChopConfReg = (configuredChopConfReg & ~userMask) | (newVal & userMask);
	UpdateChopConfRegister();
	return true;
}

// Set the driver mode
bool TmcDriverState::SetDriverMode(unsigned int mode)
{
	switch (mode)
	{
	case (unsigned int)DriverMode::spreadCycle:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_STEALTHCHOP);
		return true;

	case (unsigned int)DriverMode::stealthChop:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] | GCONF_STEALTHCHOP);
		return true;

	case (unsigned int)DriverMode::constantOffTime:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_STEALTHCHOP);
		UpdateRegister(WriteChopConf,
#if TMC_TYPE == 5130
			(writeRegisters[WriteChopConf] | CHOPCONF_CHM) & ~CHOPCONF_5130_RNDTOFF
#else
			writeRegisters[WriteChopConf] | CHOPCONF_CHM
#endif
			);
		return true;

#if TMC_TYPE == 5130
	case (unsigned int)DriverMode::randomOffTime:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_STEALTHCHOP);
		UpdateRegister(WriteChopConf, writeRegisters[WriteChopConf] | CHOPCONF_CHM | CHOPCONF_5130_RNDTOFF);
		return true;
#endif

	default:
		return false;
	}
}

// Get the driver mode
DriverMode TmcDriverState::GetDriverMode() const
{
	return ((writeRegisters[WriteGConf] & GCONF_STEALTHCHOP) != 0) ? DriverMode::stealthChop
		: ((writeRegisters[WriteChopConf] & CHOPCONF_CHM) == 0) ? DriverMode::spreadCycle
#if TMC_TYPE == 5130
			: ((writeRegisters[WriteChopConf] & CHOPCONF_5130_RNDTOFF) != 0) ? DriverMode::randomOffTime
#endif
				: DriverMode::constantOffTime;
}

// Set the motor current
void TmcDriverState::SetCurrent(float current)
{
	motorCurrent = static_cast<uint32_t>(constrain<float>(current, MinimumMotorCurrent, MaximumMotorCurrent));
	UpdateCurrent();
}

void TmcDriverState::UpdateCurrent()
{
#if TMC_TYPE == 5130
	// Assume a current sense resistor of 0.082 ohms, to which we must add 0.025 ohms internal resistance.
	// Full scale peak motor current in the high sensitivity range is give by I = 0.18/(R+0.03) = 0.18/0.105 ~= 1.6A
	// This gives us a range of 50mA to 1.6A in 50mA steps in the high sensitivity range (VSENSE = 1)
	const uint32_t iRunCsBits = (32 * motorCurrent - 800)/1615;		// formula checked by simulation on a spreadsheet
	const uint32_t iHoldCurrent = (motorCurrent * standstillCurrentFraction)/256;	// set standstill current
	const uint32_t iHoldCsBits = (32 * iHoldCurrent - 800)/1615;	// formula checked by simulation on a spreadsheet
	UpdateRegister(WriteIholdIrun,
					(writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRunCsBits << IHOLDIRUN_IRUN_SHIFT) | (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
#elif TMC_TYPE == 5160
	// See if we can set IRUN to 31 and do the current adjustment in the global scaler
	uint32_t gs = lrintf((motorCurrent * 256)/FullScaleCurrent);
	uint32_t iRun = 31;
	if (gs >= 256)
	{
		gs = 0;
	}
	else if (gs < 32)
	{
		// We can't regulate the current just through the global scaler because it has a minimum value of 32
		iRun = gs - 1;
		gs = 32;
	}
	const uint32_t iHold = (iRun * standstillCurrentFraction)/256;
	UpdateRegister(WriteIholdIrun,
					(writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRun << IHOLDIRUN_IRUN_SHIFT) | (iHold << IHOLDIRUN_IHOLD_SHIFT));
	UpdateRegister(Write5160GlobalScaler, gs);
#else
# error unknown device
#endif
}

// Enable or disable the driver
void TmcDriverState::Enable(bool en)
{
	if (enabled != en)
	{
		enabled = en;
		UpdateChopConfRegister();
	}
}

// Read the status
uint32_t TmcDriverState::ReadLiveStatus() const
{
	return readRegisters[ReadDrvStat] & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST);
}

// Read the status
uint32_t TmcDriverState::ReadAccumulatedStatus(uint32_t bitsToKeep)
{
	TaskCriticalSectionLocker lock;
	const uint32_t status = accumulatedReadRegisters[ReadDrvStat];
	accumulatedReadRegisters[ReadDrvStat] = (status & bitsToKeep) | readRegisters[ReadDrvStat];		// so that the next call to ReadAccumulatedStatus isn't missing some bits
	return status & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST);
}

// Append the driver status to a string, and reset the min/max load values
void TmcDriverState::AppendDriverStatus(const StringRef& reply, bool clearGlobalStats)
{
	const uint32_t lastReadStatus = readRegisters[ReadDrvStat];
	if (lastReadStatus & TMC_RR_OT)
	{
		reply.cat(" temperature-shutdown!");
	}
	else if (lastReadStatus & TMC_RR_OTPW)
	{
		reply.cat(" temperature-warning");
	}
	if (lastReadStatus & TMC_RR_S2G)
	{
		reply.cat(" short-to-ground");
	}
	if ((lastReadStatus & TMC_RR_OLA) && !(lastReadStatus & TMC_RR_STST))
	{
		reply.cat(" open-load-A");
	}
	if ((lastReadStatus & TMC_RR_OLB) && !(lastReadStatus & TMC_RR_STST))
	{
		reply.cat(" open-load-B");
	}
	if (lastReadStatus & TMC_RR_STST)
	{
		reply.cat(" standstill");
	}
	else if ((lastReadStatus & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB)) == 0)
	{
		reply.cat(" ok");
	}

	reply.catf(", reads %u, writes %u timeouts %u", numReads, numWrites, numTimeouts);
	numReads = numWrites = 0;
	if (clearGlobalStats)
	{
		numTimeouts = 0;
	}

	if (minSgLoadRegister <= maxSgLoadRegister)
	{
		reply.catf(", SG min/max %" PRIu32 "/%" PRIu32, minSgLoadRegister, maxSgLoadRegister);
	}
	else
	{
		reply.cat(", SG min/max not available");
	}
	ResetLoadRegisters();
}

void TmcDriverState::SetStallDetectFilter(bool sgFilter)
{
	if (sgFilter)
	{
		writeRegisters[WriteCoolConf] |= COOLCONF_SGFILT;
	}
	else
	{
		writeRegisters[WriteCoolConf] &= ~COOLCONF_SGFILT;
	}
	newRegistersToUpdate |= 1u << WriteCoolConf;
}

void TmcDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond)
{
	//TODO use hardware facility instead
	maxStallStepInterval = StepTimer::StepClockRate/max<unsigned int>(stepsPerSecond, 1);
}

void TmcDriverState::AppendStallConfig(const StringRef& reply) const
{
	const bool filtered = ((writeRegisters[WriteCoolConf] & COOLCONF_SGFILT) != 0);
	int threshold = (int)((writeRegisters[WriteCoolConf] & COOLCONF_SGT_MASK) >> COOLCONF_SGT_SHIFT);
	if (threshold >= 64)
	{
		threshold -= 128;
	}
	reply.catf("stall threshold %d, filter %s, steps/sec %" PRIu32 ", coolstep %" PRIx32,
				threshold, ((filtered) ? "on" : "off"), StepTimer::StepClockRate/maxStallStepInterval, writeRegisters[WriteCoolConf] & 0xFFFF);
}

void TmcDriverState::GetSpiCommand(uint8_t *sendDataBlock)
{
	// Find which register to send. The common case is when no registers need to be updated.
	{
		TaskCriticalSectionLocker lock;
		registersToUpdate |= newRegistersToUpdate;
		newRegistersToUpdate = 0;
	}

	if (registersToUpdate == 0)
	{
		// Read a register
		regIndexBeingUpdated = 0xFF;
		regIndexRequested = (regIndexRequested >= NumReadRegisters - 1) ? 0 : regIndexRequested + 1;
		sendDataBlock[0] = ReadRegNumbers[regIndexRequested];
		sendDataBlock[1] = 0;
		sendDataBlock[2] = 0;
		sendDataBlock[3] = 0;
		sendDataBlock[4] = 0;
	}
	else
	{
		// Write a register
		size_t regNum = 0;
		uint32_t mask = 1;
		do
		{
			if ((registersToUpdate & mask) != 0)
			{
				break;
			}
			++regNum;
			mask <<= 1;
		} while (regNum < NumWriteRegisters - 1);

		// Kick off a transfer for that register
		regIndexBeingUpdated = regNum;
		sendDataBlock[0] = WriteRegNumbers[regNum] | 0x80;
		sendDataBlock[1] = (uint8_t)(writeRegisters[regNum] >> 24);
		sendDataBlock[2] = (uint8_t)(writeRegisters[regNum] >> 16);
		sendDataBlock[3] = (uint8_t)(writeRegisters[regNum] >> 8);
		sendDataBlock[4] = (uint8_t)(writeRegisters[regNum]);
	}
}

void TmcDriverState::TransferSucceeded(const uint8_t *rcvDataBlock)
{
	// If we wrote a register, mark it up to date
	if (regIndexBeingUpdated < NumWriteRegisters)
	{
		registersToUpdate &= ~(1 << regIndexBeingUpdated);
		++numWrites;
	}

	// If we read a register, update our copy
	if (previousRegIndexRequested < NumReadRegisters)
	{
		++numReads;
		uint32_t regVal = ((uint32_t)rcvDataBlock[1] << 24) | ((uint32_t)rcvDataBlock[2] << 16) | ((uint32_t)rcvDataBlock[3] << 8) | ((uint32_t)rcvDataBlock[4]);
		if (previousRegIndexRequested == ReadDrvStat)
		{
			// We treat the DRV_STATUS register separately
			if ((regVal & (TMC_RR_OLA | TMC_RR_OLB)) != 0)
			{
				uint32_t interval;
				if (   (regVal & TMC_RR_STST) != 0
#ifdef SAME51
					|| (interval = moveInstance->GetStepInterval(axisNumber, microstepShiftFactor)) == 0		// get the full step interval
#else
					|| (interval = reprap.GetMove().GetStepInterval(axisNumber, microstepShiftFactor)) == 0		// get the full step interval
#endif
					|| interval > StepTimer::StepClockRate/MinimumOpenLoadFullStepsPerSec
				   )
				{
					regVal &= ~(TMC_RR_OLA | TMC_RR_OLB);				// open load bits are unreliable at standstill and low speeds
				}
			}
			// Only add bits to the accumulator if they appear in 2 successive samples. This is to avoid seeing transient S2G, S2VS, STST and open load errors.
			const uint32_t oldDrvStat = accumulatedReadRegisters[previousRegIndexRequested];
			readRegisters[previousRegIndexRequested] = regVal;
			regVal &= oldDrvStat;
			accumulatedReadRegisters[previousRegIndexRequested] |= regVal;
		}
		else
		{
			readRegisters[previousRegIndexRequested] = regVal;
			accumulatedReadRegisters[previousRegIndexRequested] |= regVal;
		}
	}

	if ((rcvDataBlock[0] & (1u << 2)) != 0)							// check the stall status
	{
		readRegisters[ReadDrvStat] |= TMC_RR_SG;
	}
	else
	{
		readRegisters[ReadDrvStat] &= ~TMC_RR_SG;
	}
	previousRegIndexRequested = regIndexRequested;
}

void TmcDriverState::TransferFailed()
{
	regIndexRequested = previousRegIndexRequested = 0xFF;
}

// State structures for all drivers
static TmcDriverState driverStates[MaxSmartDrivers];

// TMC51xx management task
constexpr size_t TMCTaskStackWords = 200;
static Task<TMCTaskStackWords> tmcTask;

static uint8_t sendData[5 * MaxSmartDrivers];
static uint8_t rcvData[5 * MaxSmartDrivers];

// Set up the PDC or DMAC to send a register and receive the status, but don't enable it yet
static void SetupDMA()
{
#if SAME70
	/* From the data sheet:
	 * Single Block Transfer With Single Microblock
		1. Read the XDMAC Global Channel Status Register (XDMAC_GS) to select a free channel. [we use fixed channel numbers instead.]
		2. Clear the pending Interrupt Status bit(s) by reading the selected XDMAC Channel x Interrupt Status
		Register (XDMAC_CISx).
		3. Write the XDMAC Channel x Source Address Register (XDMAC_CSAx) for channel x.
		4. Write the XDMAC Channel x Destination Address Register (XDMAC_CDAx) for channel x.
		5. Program field UBLEN in the XDMAC Channel x Microblock Control Register (XDMAC_CUBCx) with
		the number of data.
		6. Program the XDMAC Channel x Configuration Register (XDMAC_CCx):
		6.1. Clear XDMAC_CCx.TYPE for a memory-to-memory transfer, otherwise set this bit.
		6.2. Configure XDMAC_CCx.MBSIZE to the memory burst size used.
		6.3. Configure XDMAC_CCx.SAM and DAM to Memory Addressing mode.
		6.4. Configure XDMAC_CCx.DSYNC to select the peripheral transfer direction.
		6.5. Configure XDMAC_CCx.CSIZE to configure the channel chunk size (only relevant for
		peripheral synchronized transfer).
		6.6. Configure XDMAC_CCx.DWIDTH to configure the transfer data width.
		6.7. Configure XDMAC_CCx.SIF, XDMAC_CCx.DIF to configure the master interface used to
		read data and write data, respectively.
		6.8. Configure XDMAC_CCx.PERID to select the active hardware request line (only relevant for
		a peripheral synchronized transfer).
		6.9. Set XDMAC_CCx.SWREQ to use a software request (only relevant for a peripheral
		synchronized transfer).
		7. Clear the following five registers:
		– XDMAC Channel x Next Descriptor Control Register (XDMAC_CNDCx)
		– XDMAC Channel x Block Control Register (XDMAC_CBCx)
		– XDMAC Channel x Data Stride Memory Set Pattern Register (XDMAC_CDS_MSPx)
		– XDMAC Channel x Source Microblock Stride Register (XDMAC_CSUSx)
		– XDMAC Channel x Destination Microblock Stride Register (XDMAC_CDUSx)
		This indicates that the linked list is disabled, there is only one block and striding is disabled.
		8. Enable the Microblock interrupt by writing a ‘1’ to bit BIE in the XDMAC Channel x Interrupt Enable
		Register (XDMAC_CIEx). Enable the Channel x Interrupt Enable bit by writing a ‘1’ to bit IEx in the
		XDMAC Global Interrupt Enable Register (XDMAC_GIE).
		9. Enable channel x by writing a ‘1’ to bit ENx in the XDMAC Global Channel Enable Register
		(XDMAC_GE). XDMAC_GS.STx (XDMAC Channel x Status bit) is set by hardware.
		10. Once completed, the DMA channel sets XDMAC_CISx.BIS (End of Block Interrupt Status bit) and
		generates an interrupt. XDMAC_GS.STx is cleared by hardware. The software can either wait for
		an interrupt or poll the channel status bit.

		The following code is adapted from the code in the HSMCI driver instead.
	*/

	// Receive
	{
		xdmac_channel_disable(XDMAC, DmacChanTmcRx);
		xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
		p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_PER2MEM
						| XDMAC_CC_CSIZE_CHK_1
						| XDMAC_CC_DWIDTH_BYTE
						| XDMAC_CC_SIF_AHB_IF1
						| XDMAC_CC_DIF_AHB_IF0
						| XDMAC_CC_SAM_FIXED_AM
						| XDMAC_CC_DAM_INCREMENTED_AM
						| XDMAC_CC_PERID(TMC51xx_DmaRxPerid);
		p_cfg.mbr_ubc = ARRAY_SIZE(rcvData);
		p_cfg.mbr_sa = reinterpret_cast<uint32_t>(&(USART_TMC51xx->US_RHR));
		p_cfg.mbr_da = reinterpret_cast<uint32_t>(rcvData);
		xdmac_configure_transfer(XDMAC, DmacChanTmcRx, &p_cfg);
	}

	// Transmit
	{
		xdmac_channel_disable(XDMAC, DmacChanTmcTx);
		xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
		p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN
						| XDMAC_CC_MBSIZE_SINGLE
						| XDMAC_CC_DSYNC_MEM2PER
						| XDMAC_CC_CSIZE_CHK_1
						| XDMAC_CC_DWIDTH_BYTE
						| XDMAC_CC_SIF_AHB_IF0
						| XDMAC_CC_DIF_AHB_IF1
						| XDMAC_CC_SAM_INCREMENTED_AM
						| XDMAC_CC_DAM_FIXED_AM
						| XDMAC_CC_PERID(TMC51xx_DmaTxPerid);
		p_cfg.mbr_ubc = ARRAY_SIZE(sendData);
		p_cfg.mbr_sa = reinterpret_cast<uint32_t>(sendData);
		p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(USART_TMC51xx->US_THR));
		xdmac_configure_transfer(XDMAC, DmacChanTmcTx, &p_cfg);
	}

#elif SAME51
	// Receive
	DMAC->Channel[TmcRxDmaChannel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC((uint8_t)DmaTrigSource::sercom0_rx) | DMAC_CHCTRLA_TRIGACT_BURST
													| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;

	// Transmit
	DMAC->Channel[TmcTxDmaChannel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC((uint8_t)DmaTrigSource::sercom0_tx) | DMAC_CHCTRLA_TRIGACT_BURST
													| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#else
	spiPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);		// disable the PDC

	spiPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(sendData);
	spiPdc->PERIPH_TCR = ARRAY_SIZE(sendData);

	spiPdc->PERIPH_RPR = reinterpret_cast<uint32_t>(rcvData);
	spiPdc->PERIPH_RCR = ARRAY_SIZE(rcvData);
#endif
}

static inline void EnableDma()
{
#if SAME70
	xdmac_channel_enable(XDMAC, DmacChanTmcRx);
	xdmac_channel_enable(XDMAC, DmacChanTmcTx);
#elif SAME51
	DMAC->Channel[TmcRxDmaChannel].CHCTRLA.bit.ENABLE = 1;
	DMAC->Channel[TmcTxDmaChannel].CHCTRLA.bit.ENABLE = 1;
#else
	spiPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);			// enable the PDC
#endif
}

static inline void DisableDma()
{
#if SAME70
	xdmac_channel_disable(XDMAC, DmacChanTmcTx);
	xdmac_channel_disable(XDMAC, DmacChanTmcRx);
#elif SAME51
	DMAC->Channel[TmcTxDmaChannel].CHCTRLA.bit.ENABLE = 0;
	DMAC->Channel[TmcRxDmaChannel].CHCTRLA.bit.ENABLE = 0;
#else
	spiPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);		// disable the PDC
#endif
}

static inline void ResetSpi()
{
#if TMC51xx_USES_SERCOM
	// Don't disable the whole SPI between transmissions because that causes the clock output to go high impedance
	SERCOM_TMC51xx->SPI.CTRLB.bit.RXEN = 0;
#elif TMC51xx_USES_USART
	USART_TMC51xx->US_CR = US_CR_RSTRX | US_CR_RSTTX;	// reset transmitter and receiver
#else
	SPI_TMC51xx->SPI_CR = SPI_CR_SPIDIS;				// disable the SPI
	(void)SPI_TMC51xx->SPI_RDR;							// clear the receive buffer
#endif
}

static inline void EnableSpi()
{
#if TMC51xx_USES_SERCOM
	SERCOM_TMC51xx->SPI.CTRLB.bit.RXEN = 1;
#elif TMC51xx_USES_USART
	USART_TMC51xx->US_CR = US_CR_RXEN | US_CR_TXEN;		// enable transmitter and receiver
#else
	SPI_TMC51xx->SPI_CR = SPI_CR_SPIEN;					// enable SPI
#endif
}

static inline void DisableEndOfTransferInterrupt()
{
#if SAME70
	xdmac_channel_disable_interrupt(XDMAC, DmacChanTmcRx, XDMAC_CIE_BIE);
#elif TMC51xx_USES_SERCOM
	DmacDisableCompletedInterrupt(TmcRxDmaChannel);
#elif TMC51xx_USES_USART
	USART_TMC51xx->US_IDR = US_IDR_ENDRX;				// enable end-of-transfer interrupt
#else
	SPI_TMC51xx->SPI_IDR = SPI_IDR_ENDRX;				// enable end-of-transfer interrupt
#endif
}

static inline void EnableEndOfTransferInterrupt()
{
#if SAME70
	xdmac_channel_enable_interrupt(XDMAC, DmacChanTmcRx, XDMAC_CIE_BIE);
#elif TMC51xx_USES_SERCOM
	DmacEnableCompletedInterrupt(TmcRxDmaChannel);
#elif TMC51xx_USES_USART
	USART_TMC51xx->US_IER = US_IER_ENDRX;				// enable end-of-transfer interrupt
#else
	SPI_TMC51xx->SPI_IER = SPI_IER_ENDRX;				// enable end-of-transfer interrupt
#endif
}

// DMA complete callback
void RxDmaCompleteCallback(CallbackParameter param)
{
#if SAME70
	xdmac_channel_disable_interrupt(XDMAC, DmacChanTmcRx, 0xFFFFFFFF);
#endif
	fastDigitalWriteHigh(GlobalTmc51xxCSPin);			// set CS high
	tmcTask.GiveFromISR();
}

extern "C" void TmcLoop(void *)
{
	bool timedOut = true;
	for (;;)
	{
		if (driversState == DriversState::noPower)
		{
			TaskBase::Take(Mutex::TimeoutUnlimited);
		}
		else
		{
			if (driversState == DriversState::notInitialised)
			{
				for (size_t drive = 0; drive < numTmc51xxDrivers; ++drive)
				{
					driverStates[drive].WriteAll();
				}
				driversState = DriversState::initialising;
			}
			else if (!timedOut)
			{
				// Handle the read response - data comes out of the drivers in reverse driver order
				const uint8_t *readPtr = rcvData + 5 * numTmc51xxDrivers;
				for (size_t drive = 0; drive < numTmc51xxDrivers; ++drive)
				{
					readPtr -= 5;
					driverStates[drive].TransferSucceeded(readPtr);
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
						fastDigitalWriteLow(GlobalTmc51xxEnablePin);
						driversState = DriversState::ready;
					}
				}
			}

			// Set up data to write. Driver 0 is the first in the SPI chain so we must write them in reverse order.
			uint8_t *writeBufPtr = sendData + 5 * numTmc51xxDrivers;
			for (size_t i = 0; i < numTmc51xxDrivers; ++i)
			{
				writeBufPtr -= 5;
				driverStates[i].GetSpiCommand(writeBufPtr);
			}

			// Kick off a transfer
			{
				TaskCriticalSectionLocker lock;
				ResetSpi();

				fastDigitalWriteLow(GlobalTmc51xxCSPin);			// set CS low

				// On the SAME51 the order of doing the rest is critical, else we sometimes don't get the end-of-DMA interrupt
				SetupDMA();											// set up the PDC or DMAC

				// Enable the interrupt
				EnableEndOfTransferInterrupt();

				// Enable the transfer
				EnableSpi();
				EnableDma();
			}

			// Wait for the end-of-transfer interrupt
			timedOut = TaskBase::Take(TransferTimeout) == 0;
			if (timedOut)
			{
				TmcDriverState::TransferTimedOut();
				// If the transfer was interrupted then we will have written dud data to the drivers. So we should re-initialise them all.
				// Unfortunately registers that we don't normally write to may have changed too.
				fastDigitalWriteHigh(GlobalTmc51xxEnablePin);
				fastDigitalWriteHigh(GlobalTmc51xxCSPin);			// set CS high
				driversState = DriversState::notInitialised;
				DisableEndOfTransferInterrupt();
				DisableDma();
				for (size_t drive = 0; drive < numTmc51xxDrivers; ++drive)
				{
					driverStates[drive].TransferFailed();
				}
			}
		}
	}
}

namespace SmartDrivers
{
	// Initialise the driver interface and the drivers, leaving each drive disabled.
	// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
	void Init()
	{
		// Make sure the ENN and CS pins are high
		pinMode(GlobalTmc51xxEnablePin, OUTPUT_HIGH);
		pinMode(GlobalTmc51xxCSPin, OUTPUT_HIGH);

#ifndef SAME51
		// The pins are already set up for SPI in the pins table
		ConfigurePin(TMC51xxMosiPin);
		ConfigurePin(TMC51xxMisoPin);
		ConfigurePin(TMC51xxSclkPin);

		// Enable the clock to the USART or SPI
		pmc_enable_periph_clk(ID_TMC51xx_SPI);
#endif

#if TMC51xx_USES_SERCOM
		// Temporary fixed pin assignment
		gpio_set_pin_function(PortBPin(24), PINMUX_PB24C_SERCOM0_PAD0);		// MOSI
		gpio_set_pin_function(PortBPin(25), PINMUX_PB25C_SERCOM0_PAD1);		// SCLK
		gpio_set_pin_function(PortCPin(25), PINMUX_PC25C_SERCOM0_PAD3);		// MISO

		// Enable the clock
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
		hri_gclk_write_PCHCTRL_reg(GCLK, SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
		hri_mclk_set_APBAMASK_SERCOM0_bit(MCLK);

		// Set up the SERCOM
		const uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0)
								| SERCOM_SPI_CTRLA_CPOL | SERCOM_SPI_CTRLA_CPHA;
		const uint32_t regCtrlB = 0;											// 8 bits, slave select disabled, receiver disabled for now
		const uint32_t regCtrlC = 0;											// not 32-bit mode

		if (!hri_sercomusart_is_syncing(SERCOM_TMC51xx, SERCOM_USART_SYNCBUSY_SWRST))
		{
			uint32_t mode = regCtrlA & SERCOM_USART_CTRLA_MODE_Msk;
			if (hri_sercomusart_get_CTRLA_reg(SERCOM_TMC51xx, SERCOM_USART_CTRLA_ENABLE))
			{
				hri_sercomusart_clear_CTRLA_ENABLE_bit(SERCOM_TMC51xx);
				hri_sercomusart_wait_for_sync(SERCOM_TMC51xx, SERCOM_USART_SYNCBUSY_ENABLE);
			}
			hri_sercomusart_write_CTRLA_reg(SERCOM_TMC51xx, SERCOM_USART_CTRLA_SWRST | mode);
		}
		hri_sercomusart_wait_for_sync(SERCOM_TMC51xx, SERCOM_USART_SYNCBUSY_SWRST);

		hri_sercomusart_write_CTRLA_reg(SERCOM_TMC51xx, regCtrlA);
		hri_sercomusart_write_CTRLB_reg(SERCOM_TMC51xx, regCtrlB);
		hri_sercomusart_write_CTRLC_reg(SERCOM_TMC51xx, regCtrlC);
		hri_sercomusart_write_BAUD_reg(SERCOM_TMC51xx, SERCOM_SPI_BAUD_BAUD(CONF_GCLK_SERCOM0_CORE_FREQUENCY/(2 * DriversSpiClockFrequency) - 1));
		hri_sercomusart_write_DBGCTRL_reg(SERCOM_TMC51xx, SERCOM_I2CM_DBGCTRL_DBGSTOP);			// baud rate generator is stopped when CPU halted by debugger

		// Set up the DMA descriptors
		// We use separate write-back descriptors, so we only need to set this up once, but it must be in SRAM
		DmacSetBtctrl(TmcRxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
									| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
		DmacSetSourceAddress(TmcRxDmaChannel, &(SERCOM_TMC51xx->SPI.DATA.reg));
		DmacSetDestinationAddress(TmcRxDmaChannel, rcvData);
		DmacSetDataLength(TmcRxDmaChannel, ARRAY_SIZE(rcvData));

		DmacSetBtctrl(TmcTxDmaChannel, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
									| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
		DmacSetSourceAddress(TmcTxDmaChannel, sendData);
		DmacSetDestinationAddress(TmcTxDmaChannel, &(SERCOM_TMC51xx->SPI.DATA.reg));
		DmacSetDataLength(TmcTxDmaChannel, ARRAY_SIZE(sendData));

		DmacSetInterruptCallbacks(TmcRxDmaChannel, RxDmaCompleteCallback, nullptr, 0U);

		SERCOM_TMC51xx->SPI.CTRLA.bit.ENABLE = 1;		// keep the SPI enabled all the time so that the SPCLK line is driven

#elif TMC51xx_USES_USART
		// Set USART_EXT_DRV in SPI mode, with data changing on the falling edge of the clock and captured on the rising edge
		USART_TMC51xx->US_IDR = ~0u;
		USART_TMC51xx->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
		USART_TMC51xx->US_MR = US_MR_USART_MODE_SPI_MASTER
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_CHMODE_NORMAL
						| US_MR_CPOL
						| US_MR_CLKO;
		USART_TMC51xx->US_BRGR = SystemPeripheralClock()/DriversSpiClockFrequency;		// set SPI clock frequency
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
			driverStates[driver].Init(driver);						// axes are mapped straight through to drivers initially
		}

#if SAME70
		xdmac_channel_disable_interrupt(XDMAC, DmacChanTmcRx, 0xFFFFFFFF);
		DmacManager::SetInterruptCallback(DmacChanTmcRx, RxDmaCompleteCallback, CallbackParameter());				// set up DMA receive complete callback
		xdmac_enable_interrupt(XDMAC, DmacChanTmcRx);
#endif

		tmcTask.Create(TmcLoop, "TMC", nullptr, TaskPriority::TmcPriority);
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

	// Flag that the the drivers have been powered up or down
	// Before the first call to this function with 'powered' true, you must call Init()
	void Spin(bool powered)
	{
		TaskCriticalSectionLocker lock;

		if (powered)
		{
			if (driversState == DriversState::noPower)
			{
				driversState = DriversState::notInitialised;
				tmcTask.Give();									// wake up the TMC task because the drivers need to be initialised
			}
		}
		else
		{
			driversState = DriversState::noPower;				// flag that there is no power to the drivers
			fastDigitalWriteHigh(GlobalTmc51xxEnablePin);		// disable the drivers
		}
	}

	// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
	void TurnDriversOff()
	{
		digitalWrite(GlobalTmc51xxEnablePin, true);				// disable the drivers
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
			driverStates[driver].AppendDriverStatus(reply, driver + 1 == numTmc51xxDrivers);
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
