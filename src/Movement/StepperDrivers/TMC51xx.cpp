/*
 * TMC51xx.cpp
 *
 *  Created on: 26 Aug 2018
 *      Author: David
 *  Purpose:
 *  	Support for TMC5160 and TMC2160 stepper drivers, and TMC2140 drivers controlled over SPI
 */

#include "TMC51xx.h"

#if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI

#include <RTOSIface/RTOSIface.h>
#include <Platform/Platform.h>
#include <Movement/Move.h>
#include <DmacManager.h>
#include <Platform/TaskPriorities.h>
#include <General/Portability.h>
#include <AppNotifyIndices.h>

#if HAS_STALL_DETECT && SUPPORT_REMOTE_COMMANDS
# include <CAN/CanInterface.h>
#endif

#if defined(DUET3_MB6HC) || defined(DUET3MINI)

#include <Platform/RepRap.h>
#include <Endstops/Endstop.h>

static inline Move& GetMoveInstance() noexcept { return reprap.GetMove(); }

#elif defined(EXP3HC) || defined(EXP1HCL) || defined(M23CL) || defined(TOOLINDX)

static inline Move& GetMoveInstance() noexcept { return *moveInstance; }

#else
# error cannot define GetMoveInstance
#endif

#if SAME5x || SAMC21

# include <Serial.h>

# if SAME5x
#  include <hri_sercom_e54.h>
# elif SAMC21
#  include <hri_sercom_c21.h>
# endif

#elif SAME70

# include <pmc/pmc.h>
# include <xdmac/xdmac.h>
# define TMC_USES_SERCOM	0

#endif

#if SUPPORT_TMC51xx
# define TMC_TYPE	5160
#elif SUPPORT_TMC2240_SPI
# define TMC_TYPE	2240
#endif

constexpr float MinimumMotorCurrent = 50.0;
constexpr float MinimumOpenLoadMotorCurrent = 500;			// minimum current in mA for the open load status to be taken seriously
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
constexpr int DefaultStallDetectThreshold = 1;
constexpr bool DefaultStallDetectFiltered = false;
constexpr unsigned int DefaultMinimumStepsPerSecond = 200;	// for stall detection: 1 rev per second assuming 1.8deg/step, as per the TMC5160 datasheet
constexpr uint32_t DefaultTcoolthrs = 2000;					// max interval between 1/256 microsteps for stall detection to be enabled
constexpr uint32_t DefaultThigh = 200;
constexpr uint32_t LowestTmcClockSpeed =  11500000;			// the lowest speed at which the TMC driver is clocked internally
constexpr uint32_t NominalTmcClockSpeed = 12000000;			// the nominal speed at which the TMC driver is clocked internally
constexpr uint32_t HighestTmcClockSpeed = 12600000;			// the highest speed at which the TMC driver is clocked internally

#if SUPPORT_CLOSED_LOOP
constexpr size_t TmcTaskStackWords = 430;					// we need extra stack to handle closed loop tuning and writing to NVM
#elif SUPPORT_PHASE_STEPPING
constexpr size_t TmcTaskStackWords = 430;					// we need extra stack to handle phase stepping (amount not calculated yet, just taken from 1HCL)
#else
constexpr size_t TmcTaskStackWords = 140;					// with 100 stack words, deckingman's M122 on the main board after a major axis shift showed just 10 words left
#endif

#if TMC_TYPE == 5160
// We define MaxMotorCurrent and Tmc5160SenseResistor in the board configuration file because they vary between boards
constexpr float RecipFullScaleCurrent = Tmc5160SenseResistor/325.0;		// 1.0 divided by full scale current in mA
#elif TMC_TYPE == 2240
// We define MaxMotorCurrent and DriverFullScaleCurrent in the board configuration file
constexpr float RecipFullScaleCurrent = 1.0/DriverFullScaleCurrent;
#endif

constexpr float MaxStandstillCurrent = MaxMotorCurrent * 0.707;

// The SPI clock speed is a compromise:
// - too high and polling the driver chips takes too much of the CPU time
// - too low and we won't detect stalls quickly enough
// There are 40 bits to send per driver
#if SUPPORT_PHASE_STEPPING
constexpr uint32_t DriversSpiClockFrequency = 4000000;		// 4MHz SPI clock, this is the maximum rate the TMC5160/2160 support using the internal clock
constexpr uint32_t DefaultSpiSleepMicroseconds = 500;		// Sleep time used for tmcTask when not phase stepping
constexpr uint32_t PhaseStepSpiSleepMicroseconds = 125;		// Sleep time used for tmcTask when phase stepping

constexpr uint32_t DefaultSpiSleepClocks = (StepClockRate * DefaultSpiSleepMicroseconds)/1000000;
constexpr uint32_t PhaseStepSpiSleepClocks = (StepClockRate * PhaseStepSpiSleepMicroseconds)/1000000;

static uint32_t DriversDirectSleepClocks = DefaultSpiSleepClocks;	// how long the phase stepping task sleeps for in each cycle. Max SPI message frequency is ~16.7 kHz
															// there is 1 write + 1 read/write per motor current setting.
#else
constexpr uint32_t DriversSpiClockFrequency = 2000000;		// 2MHz SPI clock
#endif

constexpr uint32_t TransferTimeout = 3;						// any transfer should complete within 2 ticks @ 1ms/tick. Need to allow one more in case a tick is about to happen.

// GCONF register (0x00, RW)
constexpr uint8_t REGNUM_GCONF = 0x00;

constexpr uint32_t GCONF_5130_USE_VREF = 1 << 0;			// use external VRef
constexpr uint32_t GCONF_5130_INT_RSENSE = 1 << 1;			// use internal sense resistors
constexpr uint32_t GCONF_5130_END_COMMUTATION = 1 << 3;		// Enable commutation by full step encoder (DCIN_CFG5 = ENC_A, DCEN_CFG4 = ENC_B)

constexpr uint32_t GCONF_5160_RECAL = 1 << 0;				// Zero crossing recalibration during driver disable (via ENN or via TOFF setting)
constexpr uint32_t GCONF_FASTSTANDSTILL = 1 << 1;			// Timeout for step execution until standstill detection: 1: Short time: 2^18 clocks, 0: Normal time: 2^20 clocks
constexpr uint32_t GCONF_STEALTHCHOP = 1 << 2;				// use stealthchop mode (else spread cycle mode)
constexpr uint32_t GCONF_MULTISTEP_FILT = 1 << 3;			// Enable step input filtering for stealthChop optimization with external step source (default=1)

constexpr uint32_t GCONF_REV_DIR = 1 << 4;					// reverse motor direction
constexpr uint32_t GCONF_DIAG0_ERROR = 1 << 5;				// Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
															// DIAG0 always shows the reset-status, i.e. is active low during reset condition.
constexpr uint32_t GCONF_DIAG0_OTPW = 1 << 6;				// Enable DIAG0 active on driver over temperature prewarning (otpw)
constexpr uint32_t GCONF_DIAG0_STALL = 1 << 7;				// Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
constexpr uint32_t GCONF_DIAG1_STALL = 1 << 8;				// Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
constexpr uint32_t GCONF_DIAG1_INDEX = 1 << 9;				// Enable DIAG1 active on index position (microstep look up table position 0)
constexpr uint32_t GCONF_DIAG1_ONSTATE = 1 << 10;			// Enable DIAG1 active when chopper is on (for the coil which is in the second half of the fullstep)
constexpr uint32_t GCONF_5160_DIAG1_STEPS_SKIPPED = 1 << 11; // Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS). Do not enable in conjunction with other DIAG1 options.
constexpr uint32_t GCONF_DIAG0_PUSHPULL = 1 << 12;			// 0: SWN_DIAG0 is open collector output (active low), 1: Enable SWN_DIAG0 push pull output (active high)
constexpr uint32_t GCONF_DIAG1_PUSHPULL = 1 << 13;			// 0: SWN_DIAG1 is open collector output (active low), 1: Enable SWN_DIAG1 push pull output (active high)
constexpr uint32_t GCONF_SMALL_HYSTERESIS = 1 << 14;		// 0: Hysteresis for step frequency comparison is 1/16, 1: Hysteresis for step frequency comparison is 1/32
constexpr uint32_t GCONF_STOP_ENABLE = 1 << 15;				// 0: Normal operation, 1: Emergency stop: ENCA_DCIN stops the sequencer when tied high (no steps become executed by the sequencer, motor goes to standstill state)
constexpr uint32_t GCONF_DIRECT_MODE = 1 << 16;				// 0: Normal operation, 1: Motor coil currents and polarity directly programmed via serial interface:
															// Register XTARGET (0x2D) specifies signed coil A current (bits 8..0) and coil B current (bits 24..16).
															// In this mode, the current is scaled by IHOLD setting. Velocity based current regulation of stealthChop
															// is not available in this mode. The automatic stealthChop current regulation will work only for low stepper motor velocities.
constexpr uint32_t GCONF_5160_TEST_MODE = 1 << 17;			// 0: Normal operation, 1: Enable analog test output on pin ENCN_DCO. IHOLD[1..0] selects the function of ENCN_DCO: 0…2: T120, DAC, VDDH

#if TMC_TYPE == 5160
constexpr uint32_t DefaultGConfReg = GCONF_5160_RECAL | GCONF_MULTISTEP_FILT | GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;
#elif TMC_TYPE == 2240
constexpr uint32_t DefaultGConfReg = GCONF_MULTISTEP_FILT | GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;
#endif

// General configuration and status registers

// GSTAT register (0x01, RW). Write 1 bits to clear the flags.
constexpr uint8_t REGNUM_GSTAT = 0x01;
constexpr uint32_t GSTAT_RESET = 1u << 0;					// driver has been reset since last read
constexpr uint32_t GSTAT_DRV_ERR = 1u << 1;					// driver has been shut down due to over temp or short circuit
constexpr uint32_t GSTAT_UV_CP = 1u << 2;					// undervoltage on charge pump, driver disabled while it persists. This bit is latched for information. Will be active after initial boot.
constexpr uint32_t GSTAT_2240_REGISTER_RESET = 1u << 3;		// register map has been reset, active after initial boot
constexpr uint32_t GSTAT_2240_VM_UVLO = 1u << 4;			// Undervoltage has occurred, will be active after initial boot

constexpr uint32_t DefaultGstatReg = 0x07;					// this value clear all bits

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

#endif

// DRV_CONF register. This is completely different between TMC5160 and TMC2240.
constexpr uint8_t REGNUM_DRVCONF = 0x0A;

#if TMC_TYPE == 5160
constexpr uint32_t DRVCONF_5160_BBMTIME_SHIFT = 0;
constexpr uint32_t DRVCONF_5160_BBMTIME_MASK = 31;			// Break-Before make delay 0=shortest (100ns) … 16 (200ns) … 24=longest (375ns) >24 not recommended, use BBMCLKS instead
															// Hint: 0 recommended due to fast switching MOSFETs (Reset Default = 0)
constexpr uint32_t DRVCONF_5160_BBMCLKS_SHIFT = 8;
constexpr uint32_t DRVCONF_5160_BBMCLKS_MASK = (15 << 8);	// Digital BBM time in clock cycles (typ. 83ns). The longer setting rules (BBMTIME vs. BBMCLKS).
															// Reset Default: 2 via OTP. Hint: 2, or down to 0 recommended due to fast switching MOSFETs
constexpr uint32_t DRVCONF_5160_OTSELECT_SHIFT = 16;
constexpr uint32_t DRVCONF_5160_OTSELECT_MASK = (3 << 16);	// Selection of over temperature level for bridge disable, switch on after cool down to 120°C / OTPW level. Reset Default = 0.
															// 00: 150°C (not recommended – MOSFET might overheat); 01: 143°C 10: 136°C (Recommended); 11: 120°C (not recommended, no hysteresis)
															// Hint: Adapt overtemperature threshold as required to protect the MOSFETs or other components on the PCB.
constexpr uint32_t DRVCONF_5160_STRENGTH_SHIFT = 18;
constexpr uint32_t DRVCONF_5160_STRENGTH_MASK = (3 << 18);	// Selection of gate driver current. Adapts the gate driver current to the gate charge of the external MOSFETs.
															// 00: Normal slope (Recommended), 01: Normal+TC (medium above OTPW level), 10: Fast slope. Reset Default = 10.
constexpr uint32_t DRVCONF_5160_FILT_ISENSE_SHIFT = 20;
constexpr uint32_t DRVCONF_5160_FILT_ISENSE_MASK = (3 << 20);	// Filter time constant of sense amplifier to suppress ringing and coupling from second coil operation
															// 00: low – 100ns 01: – 200ns 10: – 300ns 11: high – 400ns
															// Hint: Increase setting if motor chopper noise occurs due to cross-coupling of both coils. Reset Default = 0.
constexpr uint32_t DefaultDrvConfReg = (2 << DRVCONF_5160_BBMCLKS_SHIFT) | (2 << DRVCONF_5160_OTSELECT_SHIFT);
#elif TMC_TYPE == 2240
constexpr unsigned int DRVCONF_2240_CURRENT_RANGE_SHIFT = 0;
constexpr uint32_t DRVCONF_2240_SLOPE_CONTROL_SHIFT = 4;
constexpr uint32_t DRVCONF_2240_SLOPE_CONTROL_MASK = 0x03 << DRVCONF_2240_SLOPE_CONTROL_SHIFT;		// 0 = 100V/us, 1 = 200V/us, 2 = 400V/us, 3 = 800V/us

constexpr uint32_t DefaultDrvConfReg = (Tmc2240CurrentRange << DRVCONF_2240_CURRENT_RANGE_SHIFT) | (Tmc2240SlopeControl << DRVCONF_2240_SLOPE_CONTROL_SHIFT);	// set current range and slope control
#endif

// Global Scaler register
constexpr uint8_t REGNUM_GLOBAL_SCALER = 0x0B;				// Global scaling of Motor current. This value is multiplied to the current scaling in order to adapt a drive to a
															// certain motor type. This value should be chosen before tuning other settings, because it also influences chopper hysteresis.
															// 0: Full Scale (or write 256) 1 … 31: Not allowed for operation 32 … 255: 32/256 … 255/256 of maximum current.
															// Hint: Values >128 recommended for best results. Reset Default 0.
constexpr uint32_t DefaultGlobalScalerReg = 0;				// until we use it as part of the current setting

constexpr uint8_t REGNUM_5160_OFFSET_READ = 0x0C;			// Bits 8..15: Offset calibration result phase A (signed). Bits 0..7: Offset calibration result phase B (signed).

constexpr uint8_t REGNUM_X_DIRECT = 0x2D;					// Coil currents for direct mode. Bits 8..0: signed coil A current. Bits 24..16: signed coil B current.
															// A maximal value of 255 in this register corresponds to a current of IHOLD
															// Note: Reg GCONF bit 16 (direct_mode) must be set to use this register

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
constexpr uint32_t CHOPCONF_2240_FD3 = 1u << 11;			// MSB of fast decay time setting TFD
constexpr uint32_t CHOPCONF_2240_DISFDCC = 1u << 12;		// disables fast decay mode when CHM = 1
constexpr uint32_t CHOPCONF_CHM = 1u << 14;					// fixed off time
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;					// blanking time
constexpr uint32_t CHOPCONF_TBL_MASK = 0x03 << CHOPCONF_TBL_SHIFT;
constexpr uint32_t CHOPCONF_2240_VHIGHFS = 1u << 18;		// high velocity fullstep selection
constexpr uint32_t CHOPCONF_2240_VHIGHCHM = 1u << 19;		// high velocity chopper mode
constexpr uint32_t CHOPCONF_2240_TPFD_SHIFT = 20;			// Passive fast decay time, allows dampening of motor mid-range resonances
constexpr uint32_t CHOPCONF_2240_TPFD_MASK = 0x0F;
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;				// microstep resolution
constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F << CHOPCONF_MRES_SHIFT;
constexpr uint32_t CHOPCONF_INTPOL = 1u << 28;				// use interpolation
constexpr uint32_t CHOPCONF_DEDGE = 1u << 29;				// step on both edges
constexpr uint32_t CHOPCONF_DISS2G = 1u << 30;				// disable short to ground protection
constexpr uint32_t CHOPCONF_DISS2VS = 1u << 31;				// disable low side short protection

#if TMC_TYPE == 5160
constexpr uint32_t DefaultChopConfReg = (1 << CHOPCONF_TBL_SHIFT) | (3 << CHOPCONF_TOFF_SHIFT) | (5 << CHOPCONF_HSTRT_SHIFT);
#elif TMC_TYPE == 2240
constexpr uint32_t DefaultChopConfReg = (2 << CHOPCONF_TBL_SHIFT) | (3 << CHOPCONF_TOFF_SHIFT) | (5 << CHOPCONF_HSTRT_SHIFT) | (2 << CHOPCONF_HEND_SHIFT);
#endif

constexpr uint8_t REGNUM_COOLCONF = 0x6D;
constexpr uint32_t COOLCONF_SGFILT = 1 << 24;				// set to update stallGuard status every 4 full steps instead of every full step
constexpr uint32_t COOLCONF_SGT_SHIFT = 16;
constexpr uint32_t COOLCONF_SGT_MASK = 127 << COOLCONF_SGT_SHIFT;	// stallguard threshold (signed)
constexpr uint32_t COOLCONF_COOL_MASK = (1u << 16) - 1;

constexpr uint32_t DefaultCoolConfReg = 0;

// DRV_STATUS register
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;
constexpr uint32_t TMC_RR_S2VS = 3 << 12;				// short to VS indicator (1 bit for each phase)
constexpr uint32_t TMC_RR_SG = 1 << 24;					// stall detected
constexpr uint32_t TMC_RR_OT = 1 << 25;					// over temperature shutdown
constexpr uint32_t TMC_RR_OTPW = 1 << 26;				// over temperature warning
constexpr uint32_t TMC_RR_S2G = 3 << 27;				// short to ground indicator (1 bit for each phase)
constexpr uint32_t TMC_RR_OL = 3 << 29;					// open load (1 bit for each phase)
constexpr uint32_t TMC_RR_STST = 1 << 31;				// standstill detected
constexpr uint32_t TMC_RR_SGRESULT = 0x3FF;				// 10-bit stallGuard2 result

constexpr unsigned int TMC_RR_S2VS_BIT_POS = 12;
constexpr unsigned int TMC_RR_SG_BIT_POS = 24;
constexpr unsigned int TMC_RR_OT_BIT_POS = 25;
constexpr unsigned int TMC_RR_OTPW_BIT_POS = 26;
constexpr unsigned int TMC_RR_S2G_BIT_POS = 27;
constexpr unsigned int TMC_RR_OL_BIT_POS = 29;
constexpr unsigned int TMC_RR_STST_BIT_POS = 31;

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC40C001E;			// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

// Common data
static constexpr size_t numTmcDrivers = MaxSmartDrivers;

static constexpr uint32_t MaxValidSgLoadRegister = 1023;
static constexpr uint32_t InvalidSgLoadRegister = 1024;

inline uint32_t GetHighestTmcClockSpeed() noexcept
{
	return HighestTmcClockSpeed;
}

inline uint32_t GetLowestTmcClockSpeed() noexcept
{
	return LowestTmcClockSpeed;
}

uint32_t SmartDrivers::GetDriverMinClockFrequency() noexcept
{
	return LowestTmcClockSpeed;
}

uint32_t SmartDrivers::GetDriverNominalClockFrequency() noexcept
{
	return NominalTmcClockSpeed;
}

uint32_t SmartDrivers::GetDriverMaxClockFrequency() noexcept
{
	return HighestTmcClockSpeed;
}

enum class DriversState : uint8_t
{
	shutDown = 0,
	noPower,				// no VIN power
	notInitialised,			// have VIN power but not started initialising drivers
	initialising,			// in the process of initialising the drivers
	ready					// drivers are initialised and ready
};

static DriversState driversState = DriversState::shutDown;

#if SUPPORT_REMOTE_COMMANDS
static LocalDriversBitmap stallEndstopsEnabled;
std::atomic<uint16_t> SmartDrivers::driverStallsToNotify(0);
#endif

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class TmcDriverState
{
public:
	void Init(uint32_t p_driverNumber) noexcept;
	void SetAxisNumber(size_t p_axisNumber) noexcept;
	uint32_t GetAxisNumber() const noexcept { return axisNumber; }
	void WriteAll() noexcept;
	bool SetMicrostepping(uint32_t shift, bool interpolate) noexcept;
	unsigned int GetMicrostepping(bool& interpolation) const noexcept;
#if SUPPORT_CLOSED_LOOP || SUPPORT_PHASE_STEPPING
	unsigned int GetMicrostepShift() const noexcept { return microstepShiftFactor; }
	uint16_t GetMicrostepPosition() const noexcept { return readRegisters[ReadMsCnt] & 1023; }
	bool SetXdirect(uint32_t regVal) noexcept;
	uint32_t GetPhaseToSet() const noexcept { return phaseToSet; }
	float GetCurrent() const noexcept { return (float)motorCurrent; }
#endif
#if SUPPORT_PHASE_STEPPING
	bool EnablePhaseStepping(bool enable) noexcept;
	bool IsPhaseSteppingEnabled() const noexcept { return phaseStepEnabled; }
#endif
	bool SetDriverMode(unsigned int mode) noexcept;
	DriverMode GetDriverMode() const noexcept;
	void SetCurrent(float current) noexcept;
	void Enable(bool en) noexcept;
	bool UpdatePending() const noexcept { return (registersToUpdate.load() | newRegistersToUpdate.load()) != 0; }
	void SetStallDetectThreshold(int sgThreshold) noexcept;
	void SetStallDetectFilter(bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept;
	StandardDriverStatus GetStatus(bool accumulated, bool clearAccumulated) noexcept;
	void AppendStallConfig(const StringRef& reply) const noexcept;
	void AppendDriverStatus(const StringRef& reply, bool clearGlobalStats) noexcept;

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(SmartDriverRegister reg) const noexcept;
	GCodeResult GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;

	float GetStandstillCurrentPercent() const noexcept;
	void SetStandstillCurrentPercent(float percent) noexcept;

	const char *_ecv_array _ecv_null CheckStallDetectionEnabled(float speed) const noexcept;

	int8_t GetCurrentScaler() const noexcept { return currentScaler; }
	bool SetCurrentScaler(int8_t cs) noexcept;
	uint8_t GetIRun() const noexcept { return (writeRegisters[WriteIholdIrun] & IHOLDIRUN_IRUN_MASK) >> IHOLDIRUN_IRUN_SHIFT; }
	uint8_t GetIHold() const noexcept { return (writeRegisters[WriteIholdIrun] & IHOLDIRUN_IHOLD_MASK) >> IHOLDIRUN_IHOLD_SHIFT; }
	uint32_t GetGlobalScaler() const noexcept { return writeRegisters[WriteGlobalScaler]; }
	float CalculateCurrent() const noexcept;				// calculate what current the driver is actually using based on register values

	static void TransferTimedOut() noexcept { ++numTimeouts; }

	void GetSpiCommand(uint8_t *sendDataBlock) noexcept;
	void GetSpiReadCommand(uint8_t *sendDataBlock) noexcept;
	void TransferSucceeded(const uint8_t *rcvDataBlock) noexcept;
	void TransferFailed() noexcept;

private:
	bool SetChopConf(uint32_t newVal) noexcept;
	void UpdateRegister(size_t regIndex, uint32_t regVal) noexcept;
	void UpdateChopConfRegister() noexcept;					// calculate the chopper control register and flag it for sending
	void UpdateCurrent() noexcept;

	void ResetLoadRegisters() noexcept
	{
		minSgLoadRegister = InvalidSgLoadRegister;			// value InvalidSgLoadRegister indicates that it hasn't been read
	}

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteGConf = 0;			// microstepping and direct mode
	static constexpr unsigned int WriteIholdIrun = 1;		// current setting
	static constexpr unsigned int WriteTpwmthrs = 2;		// upper step rate limit for stealthchop
	static constexpr unsigned int WriteTcoolthrs = 3;		// lower velocity for coolStep and stallGuard
	static constexpr unsigned int WriteThigh = 4;			// upper velocity for coolStep and stealthChop
	static constexpr unsigned int WriteChopConf = 5;		// chopper control
	static constexpr unsigned int WriteCoolConf = 6;		// coolstep control
	static constexpr unsigned int WritePwmConf = 7;			// stealthchop and freewheel control
	static constexpr unsigned int WriteGstat = 8;			// global status register (writing it resets status bits)
#if TMC_TYPE == 5160
	static constexpr unsigned int Write5160ShortConf = 9;	// short circuit detection configuration
	static constexpr unsigned int WriteDrvConf = 10;		// driver timing
	static constexpr unsigned int WriteGlobalScaler = 11;	// motor current scaling

	static constexpr unsigned int NumWriteRegisters = 12; 	// the number of registers that we write to
#elif TMC_TYPE == 2240
	static constexpr unsigned int WriteDrvConf = 9;			// driver timing
	static constexpr unsigned int WriteGlobalScaler = 10;	// motor current scaling
	static constexpr unsigned int NumWriteRegisters = 11;	// the number of registers that we write to
#endif
	static constexpr unsigned int WriteSpecial = NumWriteRegisters;

	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	static constexpr unsigned int NumReadRegisters = 5;		// the number of registers that we read from
	static const uint8_t ReadRegNumbers[NumReadRegisters];	// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadGStat = 0;
	static constexpr unsigned int ReadDrvStat = 1;
	static constexpr unsigned int ReadMsCnt = 2;
	static constexpr unsigned int ReadPwmScale = 3;
	static constexpr unsigned int ReadPwmAuto = 4;
	static constexpr unsigned int ReadSpecial = NumReadRegisters;

	static constexpr uint8_t NoRegIndex = 0xFF;				// this means no register updated, or no register requested

	volatile uint32_t writeRegisters[NumWriteRegisters + 1];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters + 1];		// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedDriveStatus;				// the accumulated drive status bits

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	uint32_t maxStallStepInterval;							// maximum interval between full steps to take any notice of stall detection

	std::atomic<uint32_t> newRegistersToUpdate;				// bitmap of register indices whose values need to be sent to the driver chip
	std::atomic<uint32_t> registersToUpdate;				// bitmap of register indices whose values need to be sent to the driver chip
	float motorCurrent;										// the configured motor current in mA

#if SUPPORT_CLOSED_LOOP || SUPPORT_PHASE_STEPPING
	uint32_t phaseToSet;									// phase value to be written to the XDIRECT register, only read/written by the TMC task
#endif

	LocalDriversBitmap driverBit;							// a bitmap containing just this driver number
	uint16_t minSgLoadRegister;								// the minimum value of the StallGuard bits we read
	uint16_t numReads, numWrites;							// how many successful reads and writes we had
	uint16_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction

	static uint16_t numTimeouts;							// how many times a transfer timed out

	uint8_t driverNumber;
	uint8_t axisNumber;										// the axis number of this driver as used to index the DriveMovements in the DDA
	uint8_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping

	int8_t currentScaler = -1;								// CS if manually specified, otherwise -1 to indicate auto calculate
	uint8_t regIndexBeingUpdated;							// which register we are sending
	uint8_t regIndexRequested;								// the register we asked to read in the previous transaction, or 0xFF
	uint8_t regIndexJustRequested;							// the register index we requested in the previous transaction, or 0xFF
	uint8_t previousRegIndexRequested;						// the register we asked to read in the previous transaction, or 0xFF
	volatile uint8_t specialReadRegisterNumber;
	volatile uint8_t specialWriteRegisterNumber;
	bool enabled;											// true if driver is enabled

#if SUPPORT_PHASE_STEPPING
	bool phaseStepEnabled = false;
	DriverMode currentMode;									// stepper driver mode if not using phase stepping
#endif
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
	REGNUM_GSTAT,
#if TMC_TYPE == 5160
	REGNUM_5160_SHORTCONF,
#endif
	REGNUM_DRVCONF,
	REGNUM_GLOBAL_SCALER,
};

const uint8_t TmcDriverState::ReadRegNumbers[NumReadRegisters] =
{
	REGNUM_GSTAT,
	REGNUM_DRV_STATUS,
	REGNUM_MSCNT,
	REGNUM_PWM_SCALE,
	REGNUM_PWM_AUTO
};

uint16_t TmcDriverState::numTimeouts = 0;								// how many times a transfer timed out

// Initialise the state of the driver and its CS pin
void TmcDriverState::Init(uint32_t p_driverNumber) noexcept
pre(!driversPowered)
{
	axisNumber = driverNumber = p_driverNumber;							// axes are mapped straight through to drivers initially
	driverBit = LocalDriversBitmap::MakeFromBits(p_driverNumber);
	enabled = false;
	registersToUpdate.store(0);
	newRegistersToUpdate.store(0);
	specialReadRegisterNumber = 0xFF;
	specialWriteRegisterNumber = 0xFF;
	motorCurrent = 0.0;
	standstillCurrentFraction = (uint16_t)min<uint32_t>((DefaultStandstillCurrentPercent * 256)/100, 256);

#if SUPPORT_PHASE_STEPPING
	currentMode = DriverMode::spreadCycle;
#endif

	// Set default values for all registers and flag them to be updated
	UpdateRegister(WriteGConf, DefaultGConfReg);
#if TMC_TYPE == 5160
	UpdateRegister(Write5160ShortConf, DefaultShortConfReg);
#endif
	UpdateRegister(WriteDrvConf, DefaultDrvConfReg);
	UpdateRegister(WriteGlobalScaler, DefaultGlobalScalerReg);
	UpdateRegister(WriteIholdIrun, DefaultIholdIrunReg);
	UpdateRegister(WriteTpwmthrs, DefaultTpwmthrsReg);
	UpdateRegister(WriteTcoolthrs, DefaultTcoolthrsReg);
	UpdateRegister(WriteThigh, DefaultThighReg);
	UpdateRegister(WriteGstat, DefaultGstatReg);
	configuredChopConfReg = DefaultChopConfReg;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);	// this also updates the chopper control register
	writeRegisters[WriteCoolConf] = DefaultCoolConfReg;
	SetStallDetectThreshold(DefaultStallDetectThreshold);				// this also updates the CoolConf register
	SetStallMinimumStepsPerSecond(DefaultMinimumStepsPerSecond);
	UpdateRegister(WritePwmConf, DefaultPwmConfReg);

	for (size_t i = 0; i < NumReadRegisters; ++i)
	{
		readRegisters[i] = 0;
	}
	accumulatedDriveStatus = 0;
	ResetLoadRegisters();

	regIndexBeingUpdated = regIndexRequested = regIndexJustRequested = previousRegIndexRequested = NoRegIndex;
	numReads = numWrites = 0;
}

// Set a register value and flag it for updating
inline void TmcDriverState::UpdateRegister(size_t regIndex, uint32_t regVal) noexcept
{
	writeRegisters[regIndex] = regVal;
	newRegistersToUpdate.fetch_or(1u << regIndex);							// flag it for sending
}

// Calculate the chopper control register and flag it for sending
void TmcDriverState::UpdateChopConfRegister() noexcept
{
	UpdateRegister(WriteChopConf, (enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK);
}

void TmcDriverState::SetStallDetectThreshold(int sgThreshold) noexcept
{
	const uint32_t sgVal = ((uint32_t)constrain<int>(sgThreshold, -64, 63)) & 127u;
	writeRegisters[WriteCoolConf] = (writeRegisters[WriteCoolConf] & ~COOLCONF_SGT_MASK) | (sgVal << COOLCONF_SGT_SHIFT);
	newRegistersToUpdate.fetch_or(1u << WriteCoolConf);
}

inline void TmcDriverState::SetAxisNumber(size_t p_axisNumber) noexcept
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void TmcDriverState::WriteAll() noexcept
{
	newRegistersToUpdate.store((1u << NumWriteRegisters) - 1);
}

float TmcDriverState::GetStandstillCurrentPercent() const noexcept
{
	return (float)(standstillCurrentFraction * 100)/256;
}

void TmcDriverState::SetStandstillCurrentPercent(float percent) noexcept
{
	standstillCurrentFraction = (uint16_t)constrain<long>(lrintf((percent * 256)/100.0), 0, 256);
	UpdateCurrent();
}

bool TmcDriverState::SetCurrentScaler(int8_t cs) noexcept
{
	if (cs > 31)
	{
		return false;
	}

	if (cs < 0)
	{
		cs = -1;
	}

	currentScaler = cs;
	UpdateCurrent();

	return true;
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift) where shift is in 0..8.
bool TmcDriverState::SetMicrostepping(uint32_t shift, bool interpolate) noexcept
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
unsigned int TmcDriverState::GetMicrostepping(bool& interpolation) const noexcept
{
	interpolation = (configuredChopConfReg & CHOPCONF_INTPOL) != 0;
	return 1u << microstepShiftFactor;
}

// Check that stall detection can occur at the specified speed
const char *_ecv_array _ecv_null TmcDriverState::CheckStallDetectionEnabled(float speed) const noexcept
{
	if (GetDriverMode() > DriverMode::spreadCycle)			// if in stealthChop or direct mode
	{
		return "driver %u is not in spreadCycle mode";
	}
	if (speed * (float)maxStallStepInterval < (float)(1u << microstepShiftFactor))
	{
		return "move is too slow for driver %u to detect stall (increase speed or reduce M915 H parameter)";
	}
#if 0	// the Tpwmthrs setting affects the DIAG pin output but not the stall detection that we read over SPI, so we must not check the following
	if (speed * (float)StepClockRate * (float)writeRegisters[WriteTpwmthrs] > (float)((GetLowestTmcClockSpeed()/256) << microstepShiftFactor))
	{
		return "move is too fast for driver %u to detect stall (reduce speed or M569 V parameter)";
	}
#endif
	return nullptr;
}

bool TmcDriverState::SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept
{
	switch (reg)
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
		UpdateRegister(WriteThigh, regVal & ((1u << 20) - 1));
		return true;

	case SmartDriverRegister::coolStep:
		UpdateRegister(WriteCoolConf, (writeRegisters[WriteCoolConf] & ~COOLCONF_COOL_MASK) | (regVal & COOLCONF_COOL_MASK));
		return true;

	case SmartDriverRegister::hdec:
	default:
		return false;
	}
}

uint32_t TmcDriverState::GetRegister(SmartDriverRegister reg) const noexcept
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
		return writeRegisters[WriteTpwmthrs] & 0x000FFFFF;

	case SmartDriverRegister::tcoolthrs:
		return writeRegisters[WriteTcoolthrs] & 0x000FFFFF;

	case SmartDriverRegister::thigh:
		return writeRegisters[WriteThigh];

	case SmartDriverRegister::coolStep:
		return writeRegisters[WriteCoolConf];

	case SmartDriverRegister::mstepPos:
		return readRegisters[ReadMsCnt];

	case SmartDriverRegister::pwmScale:
		return readRegisters[ReadPwmScale];

	case SmartDriverRegister::pwmAuto:
		return readRegisters[ReadPwmAuto];

	case SmartDriverRegister::hdec:
	default:
		return 0;
	}
}

GCodeResult TmcDriverState::GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept
{
	if (specialReadRegisterNumber == 0xFE)		// this value indicates that the register has been read and the value stored
	{
		reply.printf("Register 0x%02x value 0x%08" PRIx32, regNum, readRegisters[ReadSpecial]);
		specialReadRegisterNumber = 0xFF;
		return GCodeResult::ok;
	}

	if (specialReadRegisterNumber == 0xFF)		// this value indicates that we are read to accept a read register request
	{
		specialReadRegisterNumber = regNum;
	}
	return GCodeResult::notFinished;			// else a read is already in progress
}

GCodeResult TmcDriverState::SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept
{
	for (size_t i = 0; i < NumWriteRegisters; ++i)
	{
		if (regNum == WriteRegNumbers[i])
		{
			UpdateRegister(i, regVal);
			return GCodeResult::ok;
		}
	}
	specialWriteRegisterNumber = regNum;
	UpdateRegister(WriteSpecial, regVal);
	return GCodeResult::ok;
}

// Set the chopper control register to the settings provided by the user. We allow only the lowest 17 bits to be set.
bool TmcDriverState::SetChopConf(uint32_t newVal) noexcept
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

// Set the driver mode, returning true if successful
bool TmcDriverState::SetDriverMode(unsigned int mode) noexcept
{
	switch (mode)
	{
	case (unsigned int)DriverMode::spreadCycle:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~(GCONF_DIRECT_MODE | GCONF_STEALTHCHOP));
		configuredChopConfReg &= ~CHOPCONF_CHM;
		UpdateChopConfRegister();
		break;

	case (unsigned int)DriverMode::stealthChop:
		UpdateRegister(WriteGConf, (writeRegisters[WriteGConf] & ~GCONF_DIRECT_MODE) | GCONF_STEALTHCHOP);
		configuredChopConfReg &= ~CHOPCONF_CHM;
		UpdateChopConfRegister();
		break;

	case (unsigned int)DriverMode::constantOffTime:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~(GCONF_DIRECT_MODE | GCONF_STEALTHCHOP));
		configuredChopConfReg |= CHOPCONF_CHM;
		UpdateChopConfRegister();
		break;

	default:
		return false;
	}

#if SUPPORT_PHASE_STEPPING
	currentMode = (DriverMode)mode;
#endif

	return true;
}

// Get the driver mode
DriverMode TmcDriverState::GetDriverMode() const noexcept
{
	return
#if TMC_TYPE == 5160 && (SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP)
		  ((writeRegisters[WriteGConf] & GCONF_DIRECT_MODE) != 0) ? DriverMode::direct :
#endif
		  ((writeRegisters[WriteGConf] & GCONF_STEALTHCHOP) != 0) ? DriverMode::stealthChop
		: ((configuredChopConfReg & CHOPCONF_CHM) == 0) ? DriverMode::spreadCycle
				: DriverMode::constantOffTime;
}

#if SUPPORT_PHASE_STEPPING

bool TmcDriverState::EnablePhaseStepping(bool enable) noexcept
{
	bool ret = false;
	phaseStepEnabled = enable;
	if (enable)
	{
		UpdateRegister(WriteGConf, (writeRegisters[WriteGConf] & ~GCONF_STEALTHCHOP) | GCONF_DIRECT_MODE);
		ret = true;;
	}
	else
	{
		ret = SetDriverMode((unsigned int)currentMode);
	}
	UpdateCurrent();		// when entering direct mode we need to update the standstill current
	return ret;
}

#endif

// Set the motor current
void TmcDriverState::SetCurrent(float current) noexcept
{
	motorCurrent = constrain<float>(current, MinimumMotorCurrent, MaxMotorCurrent);
	UpdateCurrent();
}

float TmcDriverState::CalculateCurrent() const noexcept
{
	const uint32_t globalScaler = GetGlobalScaler();
	const uint32_t gs = (globalScaler == 0) ? 256 : globalScaler;
	return (float)(gs * (GetIRun() + 1)) / (256 * 32 * RecipFullScaleCurrent);
}

void TmcDriverState::UpdateCurrent() noexcept
{
	// See if we can set IRUN to 31 (or user defined value) and do the current adjustment in the global scaler
	uint8_t iRun = (currentScaler < 0) ? 31 : (uint8_t)currentScaler;

	const float csRecip = (iRun == 31) ? 1.0f : 32.0f / (float)(iRun + 1);
	uint32_t globalScaler = lrintf(motorCurrent * 256 * RecipFullScaleCurrent * csRecip);
	if (globalScaler >= 256)
	{
		const uint32_t prod = globalScaler * (iRun + 1);
		globalScaler = 0;
		iRun = (uint8_t)constrain<float>(rintf(float(prod) / 256u) - 1, 0, 31);		// globalscaler = 0 means 256
	}
	else if (globalScaler < 32)
	{
		// We can't regulate the current just through the global scaler because it has a minimum value of 32
		const uint32_t prod = globalScaler * (iRun + 1);
		globalScaler = 32;
		iRun = (uint8_t)constrain<float>(rintf(float(prod) / globalScaler) - 1, 0, 31);
	}

	// At high motor currents, limit the standstill current fraction to avoid overheating particular pairs of mosfets. Avoid dividing by zero if motorCurrent is zero.
	const uint32_t desiredStandstillCurrentFraction =
#if SUPPORT_PHASE_STEPPING
				(phaseStepEnabled) ? 256 : standstillCurrentFraction;
#else
				standstillCurrentFraction;
#endif

	constexpr float MaxStandstillCurrentTimes256 = 256 * MaxStandstillCurrent;
	const float limitedStandstillCurrentFraction = ((uint32_t)motorCurrent * desiredStandstillCurrentFraction <= MaxStandstillCurrentTimes256)
														? desiredStandstillCurrentFraction
															: MaxStandstillCurrentTimes256/(uint32_t)motorCurrent;
	const float idealIHoldCs = (float)iRun * limitedStandstillCurrentFraction * (1.0/256.0);
	const uint32_t iHoldCsBits = constrain<uint32_t>((unsigned int)(idealIHoldCs + 0.2), 1, 32) - 1;
	UpdateRegister(WriteIholdIrun, (writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRun << IHOLDIRUN_IRUN_SHIFT) | (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
	UpdateRegister(WriteGlobalScaler, globalScaler);
}

// Enable or disable the driver
void TmcDriverState::Enable(bool en) noexcept
{
	if (enabled != en)
	{
		enabled = en;
		UpdateChopConfRegister();
	}
}

// Read the status
StandardDriverStatus TmcDriverState::GetStatus(bool accumulated, bool clearAccumulated) noexcept
{
	uint32_t status;
	if (accumulated)
	{
		AtomicCriticalSectionLocker lock;

		// In the following we must or-in the current drive status, otherwise an error such as S2G may appear to go away between two successive calls
		status = accumulatedDriveStatus | readRegisters[ReadDrvStat];
		if (clearAccumulated)
		{
			// In the following we can't just copy readRegisters[ReadDrvStat] into accumulatedDriveStatus, because we only want to set bits in accumulatedDriveStatus
			// when they occur in 2 successive samples. So clear it instead.
			accumulatedDriveStatus = 0;
		}
	}
	else
	{
		status = readRegisters[ReadDrvStat];
	}

	// The lowest 8 bits of StandardDriverStatus have the same meanings as for the TMC2209 status, but the TMC51xx uses different bit assignments
	StandardDriverStatus rslt;
	rslt.all =  ExtractBit(status, TMC_RR_OTPW_BIT_POS, StandardDriverStatus::OtpwBitPos);
	rslt.all |= ExtractBit(status, TMC_RR_OT_BIT_POS, StandardDriverStatus::OtBitPos);
	rslt.all |= ExtractTwoBits(status, TMC_RR_S2G_BIT_POS, StandardDriverStatus::S2gBitsPos);		// put the s2ga and s2gb bits in the right place
	rslt.all |= ExtractTwoBits(status, TMC_RR_S2VS_BIT_POS, StandardDriverStatus::S2vsBitsPos);		// put s2vsa and s2vsb in the right place
	rslt.all |= ExtractTwoBits(status, TMC_RR_OL_BIT_POS, StandardDriverStatus::OpenLoadBitsPos);	// put ola and olb in the right place
	rslt.all |= ExtractBit(status, TMC_RR_STST_BIT_POS, StandardDriverStatus::StandstillBitPos);	// put the standstill bit in the right place
	rslt.all |= ExtractBit(status, TMC_RR_SG_BIT_POS, StandardDriverStatus::StallBitPos);			// put the stall bit in the right place
	rslt.sgresultMin = minSgLoadRegister;
	return rslt;
}

// Append any additional driver status to a string, and reset the min/max load values
void TmcDriverState::AppendDriverStatus(const StringRef& reply, bool clearGlobalStats) noexcept
{
	if (minSgLoadRegister <= MaxValidSgLoadRegister)
	{
		reply.catf(", SG min %u", minSgLoadRegister);
	}
	else
	{
		reply.cat(", SG min n/a");
	}
	ResetLoadRegisters();

	reply.catf(", mspos %u, reads %u, writes %u timeouts %u", (unsigned int)(readRegisters[ReadMsCnt] & 1023), numReads, numWrites, numTimeouts);
	numReads = numWrites = 0;
	if (clearGlobalStats)
	{
		numTimeouts = 0;
	}
}

void TmcDriverState::SetStallDetectFilter(bool sgFilter) noexcept
{
	if (sgFilter)
	{
		writeRegisters[WriteCoolConf] |= COOLCONF_SGFILT;
	}
	else
	{
		writeRegisters[WriteCoolConf] &= ~COOLCONF_SGFILT;
	}
	newRegistersToUpdate.fetch_or(1u << WriteCoolConf);
}

void TmcDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept
{
	if (stepsPerSecond == 0) { stepsPerSecond = 1; }					// avoid divide-by-zero errors
	maxStallStepInterval = StepClockRate/stepsPerSecond;
	UpdateRegister(WriteTcoolthrs, (GetHighestTmcClockSpeed() + (128 * stepsPerSecond))/(256 * stepsPerSecond));
}

void TmcDriverState::AppendStallConfig(const StringRef& reply) const noexcept
{
	const bool filtered = ((writeRegisters[WriteCoolConf] & COOLCONF_SGFILT) != 0);
	int threshold = (int)((writeRegisters[WriteCoolConf] & COOLCONF_SGT_MASK) >> COOLCONF_SGT_SHIFT);
	if (threshold >= 64)
	{
		threshold -= 128;
	}
	const uint32_t fullstepsPerSecond = StepClockRate/maxStallStepInterval;
	const float stepsPerMm = GetMoveInstance().DriveStepsPerMm(axisNumber);
	const float speed1 = (float)(fullstepsPerSecond << microstepShiftFactor)/stepsPerMm;
	const uint32_t tcoolthrs = writeRegisters[WriteTcoolthrs] & ((1ul << 20) - 1u);
	bool bdummy;
	const float speed2 = ((float)GetHighestTmcClockSpeed() * GetMicrostepping(bdummy))/(256 * tcoolthrs * stepsPerMm);
	reply.catf("stall threshold %d, filter %s, full steps/sec %" PRIu32 " (%.1f mm/sec), coolstep threshold %" PRIu32 " (%.1f mm/sec)",
				threshold, ((filtered) ? "on" : "off"), fullstepsPerSecond, (double)speed1, tcoolthrs, (double)speed2);
}

// Set up the send data block to read a register
void TmcDriverState::GetSpiReadCommand(uint8_t *sendDataBlock) noexcept
{
	if (regIndexRequested >= ReadSpecial)
	{
		regIndexRequested = 0;
	}
	else
	{
		++regIndexRequested;
		if (regIndexRequested == ReadSpecial && specialReadRegisterNumber >= 0x80)
		{
			regIndexRequested = 0;
		}
	}

	sendDataBlock[0] = (regIndexRequested == ReadSpecial) ? specialReadRegisterNumber : ReadRegNumbers[regIndexRequested];
#if SAME70 || SAMC21 || RP2040
	sendDataBlock[1] = 0;
	sendDataBlock[2] = 0;
	sendDataBlock[3] = 0;
	sendDataBlock[4] = 0;
#else
	*reinterpret_cast<uint32_t*>(sendDataBlock + 1) = 0;
#endif
	regIndexJustRequested = regIndexRequested;
}

// In the following, on the SAME70 only byte accesses to sendDataBlock are allowed, because accesses to non-cacheable memory must be aligned
// Inline because it is only called from one place
inline void TmcDriverState::GetSpiCommand(uint8_t *sendDataBlock) noexcept
{
	// Find which register to send. The common case is when no registers need to be updated.
	const uint32_t locRegistersToUpdate = (registersToUpdate |= newRegistersToUpdate.exchange(0));
	if (locRegistersToUpdate == 0)
	{
		// Read a register
		regIndexBeingUpdated = NoRegIndex;
		GetSpiReadCommand(sendDataBlock);
	}
	else
	{
		// Write a register
		regIndexJustRequested = NoRegIndex;
		const size_t regNum = LowestSetBit(locRegistersToUpdate);
		regIndexBeingUpdated = regNum;
		sendDataBlock[0] = ((regNum == WriteSpecial) ? specialWriteRegisterNumber : WriteRegNumbers[regNum]) | 0x80;
#if SAME70 || SAMC21 || RP2040
		StoreBEU32(sendDataBlock + 1, writeRegisters[regNum]);
#else
		*reinterpret_cast<uint32_t*>(sendDataBlock + 1) = __builtin_bswap32(writeRegisters[regNum]);
#endif
	}
}

void TmcDriverState::TransferSucceeded(const uint8_t *rcvDataBlock) noexcept
{
	// If we wrote a register, mark it up to date
	if (regIndexBeingUpdated <= NumWriteRegisters)
	{
		registersToUpdate &= ~(1u << regIndexBeingUpdated);
		++numWrites;
	}

	// Get the full step interval, we will need it later
	const uint32_t interval = GetMoveInstance().GetStepInterval(axisNumber, microstepShiftFactor);		// get the full step interval

	// If we read a register, update our copy
	if (previousRegIndexRequested <= NumReadRegisters)
	{
		++numReads;
#if SAME70 || SAMC21 || RP2040
		uint32_t regVal = LoadBEU32(rcvDataBlock + 1);
#else
		uint32_t regVal = __builtin_bswap32(*reinterpret_cast<const uint32_t*>(rcvDataBlock + 1));
#endif
		if (previousRegIndexRequested == ReadDrvStat)
		{
			// We treat the DRV_STATUS register separately
			if ((regVal & TMC_RR_STST) == 0)							// in standstill, SG_RESULT returns the chopper on-time instead
			{
				const uint16_t sgResult = regVal & TMC_RR_SGRESULT;
				if (sgResult < minSgLoadRegister)
				{
					minSgLoadRegister = sgResult;
				}
			}

			if ((regVal & TMC_RR_OL) != 0)
			{
				if (   (regVal & TMC_RR_STST) != 0
					|| interval == 0
					|| interval > StepClockRate/MinimumOpenLoadFullStepsPerSec
					|| motorCurrent < MinimumOpenLoadMotorCurrent
				   )
				{
					regVal &= ~TMC_RR_OL;								// open load bits are unreliable at standstill, low speeds, and low current
				}
			}

			// Only add bits to the accumulator if they appear in 2 successive samples. This is to avoid seeing transient S2G, S2VS, STST and open load errors.
			const uint32_t oldDrvStat = readRegisters[ReadDrvStat];
			readRegisters[ReadDrvStat] = regVal;
			regVal &= oldDrvStat;
			accumulatedDriveStatus |= regVal;
		}
		else
		{
			readRegisters[previousRegIndexRequested] = regVal;
			if (previousRegIndexRequested == ReadSpecial)
			{
				specialReadRegisterNumber = 0xFE;
			}
		}
	}

	// Deal with the stall status. Note that the TCoolThrs setting prevents us getting a DIAG output at low speeds, but it doesn't seem to affect the stall status
	if (   (rcvDataBlock[0] & (1u << 2)) != 0							// if the status indicates stalled
		&& interval != 0
		&& interval <= maxStallStepInterval								// if the motor speed is high enough to get a reliable stall indication
	   )
	{
		readRegisters[ReadDrvStat] |= TMC_RR_SG;
		accumulatedDriveStatus |= TMC_RR_SG;
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			if (stallEndstopsEnabled.IsBitSet(driverNumber))
			{
				stallEndstopsEnabled.ClearBit(driverNumber);
				SmartDrivers::driverStallsToNotify |= 1u << driverNumber;
				CanInterface::WakeAsyncSender();
			}
		}
		else
#endif
		{
			EndstopOrZProbe::SetDriversStalled(driverBit);
		}
	}
	else
	{
		readRegisters[ReadDrvStat] &= ~TMC_RR_SG;
		EndstopOrZProbe::SetDriversNotStalled(driverBit);
	}

	previousRegIndexRequested = (regIndexBeingUpdated == NoRegIndex) ? regIndexJustRequested : NoRegIndex;
}

void TmcDriverState::TransferFailed() noexcept
{
	regIndexJustRequested = previousRegIndexRequested = NoRegIndex;
}

// State structures for all drivers
static TmcDriverState driverStates[numTmcDrivers];

// TMC management task
static Task<TmcTaskStackWords> tmcTask;

// Declare the DMA buffers with the __nocache attribute for the SAME70. Access to these must be aligned.
// We no longer declare them static, in order that the addresses get included in the linker map file.
const size_t SpiDataSize = 5 * numTmcDrivers;				// number of bytes in the SPI transfer to/from the TMC driver chain
__nocache volatile uint8_t tmcSendData[SpiDataSize];		// used to prepare regular read/write requests via SPI
__nocache volatile uint8_t tmcRcvData[SpiDataSize];

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
__nocache volatile uint8_t tmcPhaseSendData[SpiDataSize];	// used to send specific phase data
__nocache volatile uint8_t tmcAltRcvData[SpiDataSize];

static uint32_t lastWakeupTime = 0;
static StepTimer tmcTimer;
static bool needToSetCoilCurrents = false;
static bool setCoilCurrents = false;
#endif

static volatile DmaCallbackReason dmaFinishedReason;

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP

inline bool TmcDriverState::SetXdirect(uint32_t regVal) noexcept
{
	if (regVal != phaseToSet)
	{
		phaseToSet = regVal;
		needToSetCoilCurrents = true;
		return true;
	}
	return false;
}

#endif

static void InitialiseDMA() noexcept
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
						| XDMAC_CC_PERID(TMC_DmaRxPerid);
		p_cfg.mbr_ubc = SpiDataSize;
		p_cfg.mbr_sa = reinterpret_cast<uint32_t>(&(USART_TMC->US_RHR));
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
						| XDMAC_CC_PERID(TMC_DmaTxPerid);
		p_cfg.mbr_ubc = SpiDataSize;
		p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(USART_TMC->US_THR));
		xdmac_configure_transfer(XDMAC, DmacChanTmcTx, &p_cfg);
	}
#endif
}

// Set up the PDC or DMAC to send a register and receive the status, but don't enable it yet
static void SetupDMA(const volatile uint8_t *txData, const volatile uint8_t *rxData) noexcept
{
#if SAME70
	// Receive
	{
		xdmac_channel_disable(XDMAC, DmacChanTmcRx);
		uint32_t mbr_da = reinterpret_cast<uint32_t>(rxData);
		xdmac_channel_get_interrupt_status(XDMAC, DmacChanTmcRx);
		xdmac_channel_set_destination_addr(XDMAC, DmacChanTmcRx, mbr_da);
	}

	// Transmit
	{
		xdmac_channel_disable(XDMAC, DmacChanTmcTx);
		uint32_t mbr_sa = reinterpret_cast<uint32_t>(txData);
		xdmac_channel_get_interrupt_status(XDMAC, DmacChanTmcRx);
		xdmac_channel_set_source_addr(XDMAC, DmacChanTmcTx, mbr_sa);
	}

#elif SAME5x || SAMC21
	DmacManager::DisableChannel(DmacChanTmcRx);
	DmacManager::DisableChannel(DmacChanTmcTx);
	DmacManager::SetSourceAddress(DmacChanTmcTx, (void*)txData);
	DmacManager::SetDataLength(DmacChanTmcTx, SpiDataSize);
	DmacManager::SetDestinationAddress(DmacChanTmcRx, (void*)rxData);
	DmacManager::SetDataLength(DmacChanTmcRx, SpiDataSize);
#else
	spiPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);		// disable the PDC

	spiPdc->PERIPH_TPR = reinterpret_cast<uint32_t>(txData);
	spiPdc->PERIPH_TCR = SpiDataSize;

	spiPdc->PERIPH_RPR = reinterpret_cast<uint32_t>(rxData);
	spiPdc->PERIPH_RCR = SpiDataSize;
#endif
}

static inline void EnableDma() noexcept
{
#if SAME70
	xdmac_channel_enable(XDMAC, DmacChanTmcRx);
	xdmac_channel_enable(XDMAC, DmacChanTmcTx);
#elif SAME5x || SAMC21
	DmacManager::EnableChannel(DmacChanTmcRx, DmacPrioTmcRx);
	DmacManager::EnableChannel(DmacChanTmcTx, DmacPrioTmcTx);
#else
	spiPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);			// enable the PDC
#endif
}

static inline void DisableDma() noexcept
{
#if SAME70
	xdmac_channel_disable(XDMAC, DmacChanTmcTx);
	xdmac_channel_disable(XDMAC, DmacChanTmcRx);
#elif SAME5x || SAMC21
	DmacManager::DisableChannel(DmacChanTmcTx);
	DmacManager::DisableChannel(DmacChanTmcRx);
#else
	spiPdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);		// disable the PDC
#endif
}

static inline void ResetSpi() noexcept
{
#if TMC_USES_SERCOM
	SERCOM_TMC->SPI.CTRLA.bit.ENABLE = 0;			// warning: this makes SCLK float!
	while (SERCOM_TMC->SPI.SYNCBUSY.bit.ENABLE) { }
#elif TMC_USES_USART
	USART_TMC->US_CR = US_CR_RSTRX | US_CR_RSTTX;	// reset transmitter and receiver
#else
	SPI_TMC->SPI_CR = SPI_CR_SPIDIS;				// disable the SPI
	(void)SPI_TMC->SPI_RDR;							// clear the receive buffer
#endif
}

static inline void EnableSpi() noexcept
{
#if TMC_USES_SERCOM
	SERCOM_TMC->SPI.CTRLB.bit.RXEN = 1;
	while (SERCOM_TMC->SPI.SYNCBUSY.bit.CTRLB) { }
	SERCOM_TMC->SPI.CTRLA.bit.ENABLE = 1;
	while (SERCOM_TMC->SPI.SYNCBUSY.bit.ENABLE) { }
#elif TMC_USES_USART
	USART_TMC->US_CR = US_CR_RXEN | US_CR_TXEN;		// enable transmitter and receiver
#else
	SPI_TMC->SPI_CR = SPI_CR_SPIEN;					// enable SPI
#endif
}

static inline void DisableEndOfTransferInterrupt() noexcept
{
#if SAME70
	xdmac_channel_disable_interrupt(XDMAC, DmacChanTmcRx, XDMAC_CIE_BIE);
#elif TMC_USES_SERCOM
	DmacManager::DisableCompletedInterrupt(DmacChanTmcRx);
#elif TMC_USES_USART
	USART_TMC->US_IDR = US_IDR_ENDRX;				// enable end-of-transfer interrupt
#else
	SPI_TMC->SPI_IDR = SPI_IDR_ENDRX;				// enable end-of-transfer interrupt
#endif
}

static inline void EnableEndOfTransferInterrupt() noexcept
{
#if SAME70
	xdmac_channel_enable_interrupt(XDMAC, DmacChanTmcRx, XDMAC_CIE_BIE);
#elif TMC_USES_SERCOM
	DmacManager::EnableCompletedInterrupt(DmacChanTmcRx);
#elif TMC_USES_USART
	USART_TMC->US_IER = US_IER_ENDRX;				// enable end-of-transfer interrupt
#else
	SPI_TMC->SPI_IER = SPI_IER_ENDRX;				// enable end-of-transfer interrupt
#endif
}

// DMA complete callback
void RxDmaCompleteCallback(CallbackParameter param, DmaCallbackReason reason) noexcept
{
	fastDigitalWriteHigh(GlobalTmcCSPin);			// set CS high
#if SAME70
	DisableEndOfTransferInterrupt();
#endif
	dmaFinishedReason = reason;
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	// When in phase stepping or closed loop mode we send the coil currents if any have changes since last time we sent them.
	// Send a "normal" read or write request after the coil currents have been set.
	// We don't care about the response from setting the motor currents so that is written to tmcAltRcvData so as to not overwrite tmcRcvData
	if (setCoilCurrents)								// if we just wrote the coil currents
	{
		setCoilCurrents = false;
		const uint32_t start = GetCurrentCycles();		// get the time now so we can time the CS high signal
		SetupDMA(tmcSendData, tmcAltRcvData);			// set up the PDC or DMAC
		dmaFinishedReason = DmaCallbackReason::none;
		EnableEndOfTransferInterrupt();
		DelayCycles(start, 2 * SystemCoreClockFreq/DriversSpiClockFrequency);	// keep CS high for 2 SPI clock cycles between transactions
		fastDigitalWriteLow(GlobalTmcCSPin);		// set CS low
		ResetSpi();
		EnableDma();
		EnableSpi();
	}
	else
	{
		// We run the SPI bus at high speeds so that motor currents get updated as quickly as possible.
		// If we wake up as soon as the transfer has completed then we will use too much of the available CPU time.
		// So schedule a wakeup call instead. Try to make the wakeup interval regular.
		lastWakeupTime += DriversDirectSleepClocks;

		{
			// If the DMA interrupt priority is better (lower number) than the step interrupt priority then we must disable interrupts here
			AtomicCriticalSectionLocker lock;
			if (tmcTimer.ScheduleCallbackFromIsr(lastWakeupTime))
			{
				lastWakeupTime = StepTimer::GetTimerTicksWhenInterruptsDisabled();
				tmcTask.GiveFromISR(NotifyIndices::Tmc);
			}
		}
	}
#else
	tmcTask.GiveFromISR(NotifyIndices::Tmc);
#endif
}

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
static void TmcTimerCallback(CallbackParameter) noexcept
{
	tmcTask.GiveFromISR(NotifyIndices::Tmc);
}
#endif

extern "C" [[noreturn]] void TmcLoop(void *) noexcept
{
	InitialiseDMA();
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	tmcTimer.SetCallback(TmcTimerCallback, (CallbackParameter)0);
#endif
	bool timedOut = true;
	for (;;)
	{
		if (driversState == DriversState::noPower)
		{
			TaskBase::TakeIndexed(NotifyIndices::Tmc);
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
			lastWakeupTime = StepTimer::GetTimerTicks();
#endif
		}
		else if (driversState == DriversState::notInitialised)
		{
			for (size_t drive = 0; drive < numTmcDrivers; ++drive)
			{
				driverStates[drive].WriteAll();
			}
			driversState = DriversState::initialising;
		}
		else if (!timedOut)
		{
			// Handle the read response - data comes out of the drivers in reverse driver order
#if SINGLE_DRIVER
			driverStates[0].TransferSucceeded(const_cast<const uint8_t*>(tmcRcvData));
			if (driversState == DriversState::initialising && !driverStates[0].UpdatePending())
			{
				fastDigitalWriteLow(GlobalTmcEnablePin);
				driversState = DriversState::ready;
			}
#else
			const volatile uint8_t *readPtr = tmcRcvData + 5 * numTmcDrivers;
			for (size_t drive = 0; drive < numTmcDrivers; ++drive)
			{
				readPtr -= 5;
				driverStates[drive].TransferSucceeded(const_cast<const uint8_t*>(readPtr));
			}

			if (driversState == DriversState::initialising)
			{
				// If all drivers that share the global enable have been initialised, set the global enable
				bool allInitialised = true;
				for (size_t i = 0; i < numTmcDrivers; ++i)
				{
					if (driverStates[i].UpdatePending())
					{
						allInitialised = false;
						break;
					}
				}

				if (allInitialised)
				{
					fastDigitalWriteLow(GlobalTmcEnablePin);
					driversState = DriversState::ready;
				}
			}
#endif
		}

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		// Set the motor phase currents before we write them
		GetMoveInstance().PhaseStepControlLoop();
#endif

		// Set up data to write. Driver 0 is the first in the SPI chain so we must write them in reverse order.
#if SINGLE_DRIVER
		driverStates[0].GetSpiCommand(const_cast<uint8_t*>(tmcSendData));

# if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (needToSetCoilCurrents)
		{
			needToSetCoilCurrents = false;
			tmcPhaseSendData[0] = REGNUM_X_DIRECT | 0x80;
			StoreBEU32(const_cast<uint8_t*>(tmcPhaseSendData + 1), driverStates[0].GetPhaseToSet());
			setCoilCurrents = true;
		}
# endif
#else
		volatile uint8_t *writeBufPtr = tmcSendData + 5 * numTmcDrivers;
		for (size_t i = 0; i < numTmcDrivers; ++i)
		{
			writeBufPtr -= 5;
			driverStates[i].GetSpiCommand(const_cast<uint8_t*>(writeBufPtr));
		}

# if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (needToSetCoilCurrents)
		{
			needToSetCoilCurrents = false;
			writeBufPtr = tmcPhaseSendData + 5 * numTmcDrivers;
			for (size_t i = 0; i < numTmcDrivers; ++i)
			{
				writeBufPtr -= 5;
				writeBufPtr[0] = REGNUM_X_DIRECT | 0x80;
				StoreBEU32(const_cast<uint8_t*>(writeBufPtr + 1), driverStates[i].GetPhaseToSet());
			}
			setCoilCurrents = true;
		}
# endif
#endif

		// Kick off a transfer.
		// On the SAME5x the only way I have found to get reliable transfers and no timeouts is to disable SPI, enable DMA, and then enable SPI.
		// Enabling SPI before DMA sometimes results in timeouts.
		// Unfortunately, when we disable SPI the SCLK line floats. Therefore we disable SPI for as little time as possible.
		{
			TaskCriticalSectionLocker lock;

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
			SetupDMA((setCoilCurrents) ? tmcPhaseSendData : tmcSendData, tmcRcvData);	// set up the PDC or DMAC
#else
			SetupDMA(tmcSendData, tmcRcvData);					// set up the PDC or DMAC
#endif
			dmaFinishedReason = DmaCallbackReason::none;

			AtomicCriticalSectionLocker lock2;

			fastDigitalWriteLow(GlobalTmcCSPin);				// set CS low
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
			tmcTimer.CancelCallbackFromIsr();					// in case the timer is still running from a previous timed-out transfer
#endif
			TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::Tmc);
			EnableEndOfTransferInterrupt();
			ResetSpi();
			EnableDma();
			EnableSpi();
		}

		// Wait for the end-of-transfer interrupt
		(void)TaskBase::TakeIndexed(NotifyIndices::Tmc, TransferTimeout);
		DisableEndOfTransferInterrupt();
		DisableDma();

		// We don't care if the TakeIndexed call returned timeout, if the DMA completed then the transfer is OK
		timedOut = (dmaFinishedReason != DmaCallbackReason::complete);
		if (timedOut)
		{
			TmcDriverState::TransferTimedOut();
			// If the transfer was interrupted then we will have written dud data to the drivers. So we should re-initialise them all.
			// Unfortunately registers that we don't normally write to may have changed too.
			fastDigitalWriteHigh(GlobalTmcEnablePin);
			fastDigitalWriteHigh(GlobalTmcCSPin);				// set CS high
			driversState = DriversState::notInitialised;
			for (size_t drive = 0; drive < numTmcDrivers; ++drive)
			{
				driverStates[drive].TransferFailed();
			}
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
			lastWakeupTime = StepTimer::GetTimerTicks();
#endif
		}
	}
}

// Members of namespace SmartDrivers

// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
void SmartDrivers::Init() noexcept
{
	// Make sure the ENN and CS pins are high
	SetPinMode(GlobalTmcEnablePin, OUTPUT_HIGH);
	SetPinMode(GlobalTmcCSPin, OUTPUT_HIGH);

	SetPinFunction(TMCMosiPin, TMCMosiPinPeriphMode);
	SetPinFunction(TMCMisoPin, TMCMisoPinPeriphMode);
	SetPinFunction(TMCSclkPin, TMCSclkPinPeriphMode);

	// Enable the clock to the USART or SPI
#if SAME5x || SAMC21
	Serial::EnableSercomClock(TmcSercomNumber);
#else
	pmc_enable_periph_clk(ID_TMC_SPI);
#endif

#if TMC_USES_SERCOM
	// Set up the SERCOM
	const uint32_t regCtrlA = SERCOM_SPI_CTRLA_MODE(3) | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_FORM(0)
							| SERCOM_SPI_CTRLA_CPOL | SERCOM_SPI_CTRLA_CPHA;
	const uint32_t regCtrlB = 0;											// 8 bits, slave select disabled, receiver disabled for now
# if !SAMC21
	const uint32_t regCtrlC = 0;											// not 32-bit mode
# endif

	if (!hri_sercomspi_is_syncing(SERCOM_TMC, SERCOM_SPI_SYNCBUSY_SWRST))
	{
		uint32_t mode = regCtrlA & SERCOM_SPI_CTRLA_MODE_Msk;
		if (hri_sercomspi_get_CTRLA_reg(SERCOM_TMC, SERCOM_SPI_CTRLA_ENABLE))
		{
			hri_sercomspi_clear_CTRLA_ENABLE_bit(SERCOM_TMC);
			hri_sercomspi_wait_for_sync(SERCOM_TMC, SERCOM_SPI_SYNCBUSY_ENABLE);
		}
		hri_sercomspi_write_CTRLA_reg(SERCOM_TMC, SERCOM_SPI_CTRLA_SWRST | mode);
	}
	hri_sercomspi_wait_for_sync(SERCOM_TMC, SERCOM_SPI_SYNCBUSY_SWRST);

	hri_sercomspi_write_CTRLA_reg(SERCOM_TMC, regCtrlA);
	hri_sercomspi_write_CTRLB_reg(SERCOM_TMC, regCtrlB);
# if !SAMC21
	hri_sercomspi_write_CTRLC_reg(SERCOM_TMC, regCtrlC);
# endif
#if SAME70
	hri_sercomspi_write_BAUD_reg(SERCOM_TMC, SERCOM_SPI_BAUD_BAUD(SystemPeripheralClock/(2 * DriversSpiClockFrequency) - 1));
#else
	hri_sercomspi_write_BAUD_reg(SERCOM_TMC, SERCOM_SPI_BAUD_BAUD(Serial::SercomFastGclkFreq/(2 * DriversSpiClockFrequency) - 1));
#endif
	hri_sercomspi_write_DBGCTRL_reg(SERCOM_TMC, SERCOM_I2CM_DBGCTRL_DBGSTOP);			// baud rate generator is stopped when CPU halted by debugger

	// Set up the DMA descriptors
	// We use separate write-back descriptors, so we only need to set this up once
	DmacManager::SetBtctrl(DmacChanTmcRx, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_STEPSIZE_X1);
	DmacManager::SetSourceAddress(DmacChanTmcRx, &(SERCOM_TMC->SPI.DATA.reg));
	DmacManager::SetTriggerSourceSercomRx(DmacChanTmcRx, TmcSercomNumber);

	DmacManager::SetBtctrl(DmacChanTmcTx, DMAC_BTCTRL_VALID | DMAC_BTCTRL_EVOSEL_DISABLE | DMAC_BTCTRL_BLOCKACT_INT | DMAC_BTCTRL_BEATSIZE_BYTE
								| DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_STEPSIZE_X1);
	DmacManager::SetDestinationAddress(DmacChanTmcTx, &(SERCOM_TMC->SPI.DATA.reg));
	DmacManager::SetTriggerSourceSercomTx(DmacChanTmcTx, TmcSercomNumber);

	DmacManager::SetInterruptCallback(DmacChanTmcRx, RxDmaCompleteCallback, CallbackParameter(0U));

	SERCOM_TMC->SPI.CTRLA.bit.ENABLE = 1;		// keep the SPI enabled all the time so that the SPCLK line is driven

#elif TMC_USES_USART
	// Set USART_EXT_DRV in SPI mode, with data changing on the falling edge of the clock and captured on the rising edge
	USART_TMC->US_IDR = ~0u;
	USART_TMC->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;
	USART_TMC->US_MR = US_MR_USART_MODE_SPI_MASTER
					| US_MR_USCLKS_MCK
					| US_MR_CHRL_8_BIT
					| US_MR_CHMODE_NORMAL
					| US_MR_CPOL
					| US_MR_CLKO;
	USART_TMC->US_BRGR = SystemPeripheralClock()/DriversSpiClockFrequency;		// set SPI clock frequency
	USART_TMC->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;

	// We need a few microseconds of delay here for the USART to sort itself out before we send any data,
	// otherwise the processor generates two short reset pulses on its own NRST pin, and resets itself.
	// 2016-07-07: removed this delay, because we no longer send commands to the TMC2660 drivers immediately.
	//delay(10);
#else
	// Set up the SPI interface with data changing on the falling edge of the clock and captured on the rising edge
	spi_reset(SPI_TMC);										// this clears the transmit and receive registers and puts the SPI into slave mode
	SPI_TMC->SPI_MR = SPI_MR_MSTR							// master mode
					| SPI_MR_MODFDIS							// disable fault detection
					| SPI_MR_PCS(0);							// fixed peripheral select

	// Set SPI mode, clock frequency, CS active after transfer, delay between transfers
	const uint16_t baud_div = (uint16_t)spi_calc_baudrate_div(DriversSpiClockFrequency, SystemCoreClock);
	const uint32_t csr = SPI_CSR_SCBR(baud_div)					// Baud rate
					| SPI_CSR_BITS_8_BIT						// Transfer bit width
					| SPI_CSR_DLYBCT(0)      					// Transfer delay
					| SPI_CSR_CSAAT								// Keep CS low after transfer in case we are slow in writing the next byte
					| SPI_CSR_CPOL;								// clock high between transfers
	SPI_TMC->SPI_CSR[0] = csr;
#endif

	driversState = DriversState::noPower;
	for (size_t driver = 0; driver < numTmcDrivers; ++driver)
	{
		driverStates[driver].Init(driver);
	}

#if SAME70
	xdmac_channel_disable_interrupt(XDMAC, DmacChanTmcRx, 0xFFFFFFFF);
	DmacManager::SetInterruptCallback(DmacChanTmcRx, RxDmaCompleteCallback, CallbackParameter());				// set up DMA receive complete callback
	xdmac_enable_interrupt(XDMAC, DmacChanTmcRx);
#endif

	driversState = DriversState::noPower;
	tmcTask.Create(TmcLoop, "TMC", nullptr, TaskPriority::TmcPriority);
}

// Shut down the drivers and stop any related interrupts
void SmartDrivers::Exit() noexcept
{
	digitalWrite(GlobalTmcEnablePin, true);					// disable the drivers
#if !TMC_USES_SERCOM
	NVIC_DisableIRQ(TMC_SPI_IRQn);
#endif
	tmcTask.TerminateAndUnlink();
	driversState = DriversState::shutDown;						// prevent Spin() calls from doing anything
}

void SmartDrivers::SetAxisNumber(size_t driver, uint32_t axisNumber) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].SetAxisNumber(axisNumber);
	}
}

uint32_t SmartDrivers::GetAxisNumber(size_t drive) noexcept
{
	return (drive < numTmcDrivers) ? driverStates[drive].GetAxisNumber() : 0;
}

void SmartDrivers::SetCurrent(size_t driver, float current) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].SetCurrent(current);
	}
}

void SmartDrivers::EnableDrive(size_t driver, bool en) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].Enable(en);
	}
}

// Set microstepping and microstep interpolation
bool SmartDrivers::SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept
{
	if (driver < numTmcDrivers && microsteps > 0)
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
unsigned int SmartDrivers::GetMicrostepping(size_t driver, bool& interpolation) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].GetMicrostepping(interpolation);
	}
	interpolation = false;
	return 1;
}

#if SUPPORT_PHASE_STEPPING

bool SmartDrivers::EnablePhaseStepping(size_t driver, bool enable) noexcept
{
	if (driver >= numTmcDrivers)
	{
		return false;
	}

	if (!driverStates[driver].EnablePhaseStepping(enable))
	{
		return false;
	}

	bool anyDriversUsingPhaseStepping = false;
	if (enable)
	{
		anyDriversUsingPhaseStepping = true;
	}
	else
	{
		for (size_t i = 0; i < numTmcDrivers; i++)
		{
			if (driverStates[i].IsPhaseSteppingEnabled())
			{
				anyDriversUsingPhaseStepping = true;
				break;
			}
		}
	}

	DriversDirectSleepClocks = anyDriversUsingPhaseStepping ? PhaseStepSpiSleepClocks : DefaultSpiSleepClocks;
	tmcTask.SetPriority(anyDriversUsingPhaseStepping ? TaskPriority::TmcPhaseStepPriority : TaskPriority::TmcPriority);
	return true;
}

bool SmartDrivers::IsPhaseSteppingEnabled(size_t driver) noexcept
{
	return driver < numTmcDrivers && driverStates[driver].IsPhaseSteppingEnabled();
}

#endif

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP

// Get the configured motor current in mA
float SmartDrivers::GetCurrent(size_t driver) noexcept
{
	return (driver < numTmcDrivers) ? driverStates[driver].GetCurrent() : 0.0;
}

// Get the amount we have to shift 1 left by to get the microstepping
unsigned int SmartDrivers::GetMicrostepShift(size_t driver) noexcept
{
	return (driver < numTmcDrivers) ? driverStates[driver].GetMicrostepShift() : 0;
}

// Get the coil A microstep position as a number in the range 0..1023
uint16_t SmartDrivers::GetMicrostepPosition(size_t driver) noexcept
{
	return (driver < numTmcDrivers) ? driverStates[driver].GetMicrostepPosition() : 0;
}

// Schedules a request to update the motor phases using XDIRECT register.
// Returns true if request is scheduled. Will not schedule a request if it is equal to the current value.
bool SmartDrivers::SetMotorPhases(size_t driver, uint32_t regVal) noexcept
{
	return driverStates[driver].SetXdirect(regVal);
}

#endif

bool SmartDrivers::SetDriverMode(size_t driver, unsigned int mode) noexcept
{
	if (driver >= numTmcDrivers)
	{
		return false;
	}
#if SUPPORT_PHASE_STEPPING
	if (driverStates[driver].IsPhaseSteppingEnabled())
	{
		return false;
	}
#endif
	return driverStates[driver].SetDriverMode(mode);
}

DriverMode SmartDrivers::GetDriverMode(size_t driver) noexcept
{
	return (driver < numTmcDrivers) ? driverStates[driver].GetDriverMode() : DriverMode::unknown;
}

// Flag that the the drivers have been powered up or down
// Before the first call to this function with 'powered' true, you must call Init()
void SmartDrivers::Spin(bool powered) noexcept
{
	TaskCriticalSectionLocker lock;

	if (powered)
	{
		if (driversState == DriversState::noPower)
		{
			driversState = DriversState::notInitialised;
			tmcTask.Give(NotifyIndices::Tmc);				// wake up the TMC task because the drivers need to be initialised
		}
	}
	else if (driversState != DriversState::shutDown)
	{
		driversState = DriversState::noPower;				// flag that there is no power to the drivers
		fastDigitalWriteHigh(GlobalTmcEnablePin);		// disable the drivers
	}
}

// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
void SmartDrivers::TurnDriversOff() noexcept
{
	digitalWrite(GlobalTmcEnablePin, true);				// disable the drivers
	driversState = DriversState::noPower;
}

void SmartDrivers::SetStallThreshold(size_t driver, int sgThreshold) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].SetStallDetectThreshold(sgThreshold);
	}
}

void SmartDrivers::SetStallFilter(size_t driver, bool sgFilter) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].SetStallDetectFilter(sgFilter);
	}
}

void SmartDrivers::SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].SetStallMinimumStepsPerSecond(stepsPerSecond);
	}
}

void SmartDrivers::AppendStallConfig(size_t driver, const StringRef& reply) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].AppendStallConfig(reply);
	}
}

void SmartDrivers::AppendDriverStatus(size_t driver, const StringRef& reply) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].AppendDriverStatus(reply, driver + 1 == numTmcDrivers);
	}
}

float SmartDrivers::GetStandstillCurrentPercent(size_t driver) noexcept
{
	return (driver < numTmcDrivers) ? driverStates[driver].GetStandstillCurrentPercent() : 0.0;
}

void SmartDrivers::SetStandstillCurrentPercent(size_t driver, float percent) noexcept
{
	if (driver < numTmcDrivers)
	{
		driverStates[driver].SetStandstillCurrentPercent(percent);
	}
}

bool SmartDrivers::SetCurrentScaler(size_t driver, int8_t cs) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].SetCurrentScaler(cs);
	}
	return false;
}


uint8_t SmartDrivers::GetIRun(size_t driver) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].GetIRun();
	}
	return 0;
}

uint8_t SmartDrivers::GetIHold(size_t driver) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].GetIHold();
	}
	return 0;
}

uint32_t SmartDrivers::GetGlobalScaler(size_t driver) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].GetGlobalScaler();
	}
	return 0;
}

float SmartDrivers::GetCalculatedCurrent(size_t driver) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].CalculateCurrent();
	}
	return 0.0f;
}

bool SmartDrivers::SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept
{
	return (driver < numTmcDrivers) && driverStates[driver].SetRegister(reg, regVal);
}

uint32_t SmartDrivers::GetRegister(size_t driver, SmartDriverRegister reg) noexcept
{
	return (driver < numTmcDrivers) ? driverStates[driver].GetRegister(reg) : 0;
}

// Read any register from a driver
// This will return GCodeResult:notFinished for at least the first call if the driver number is valid, so it must be called repeatedly until it returns a different value.
GCodeResult SmartDrivers::GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].GetAnyRegister(reply, regNum);
	}
	reply.copy("Invalid smart driver number");
	return GCodeResult::error;
}

GCodeResult SmartDrivers::SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].SetAnyRegister(reply, regNum, regVal);
	}
	reply.copy("Invalid smart driver number");
	return GCodeResult::error;
}

StandardDriverStatus SmartDrivers::GetStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept
{
	if (driver < numTmcDrivers)
	{
		return driverStates[driver].GetStatus(accumulated, clearAccumulated);
	}
	return StandardDriverStatus();
}

// Check whether stall detection is viable. If yes, return nullptr. If no, return a message string in flash memory containing a single %u placeholder for the driver number.
const char *_ecv_array _ecv_null SmartDrivers::CheckStallDetectionEnabled(size_t driver, float speed) noexcept
{
	return (driver < numTmcDrivers)
			? driverStates[driver].CheckStallDetectionEnabled(speed)
				: "driver %u does not support stall detection";
}

#if SUPPORT_REMOTE_COMMANDS

GCodeResult SmartDrivers::SetStallEndstopReporting(uint16_t driverNumber, float speed, const StringRef& reply) noexcept
{
	if (driverNumber < numTmcDrivers)
	{
		const char *_ecv_array _ecv_null const msg = driverStates[driverNumber].CheckStallDetectionEnabled(speed);
		if (msg == nullptr)
		{
			stallEndstopsEnabled.SetBit(driverNumber);
			return GCodeResult::ok;
		}
		reply.printf(msg, driverNumber);
		return GCodeResult::error;
	}
	else
	{
		stallEndstopsEnabled.Clear();
		driverStallsToNotify = 0;
		return GCodeResult::ok;
	}
}

#endif

#endif

// End
