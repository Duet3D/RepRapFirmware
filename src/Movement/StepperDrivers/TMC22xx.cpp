/*
 * TMC22xx.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#include "TMC22xx.h"

#if SUPPORT_TMC22xx

#ifndef TMC22xx_HAS_ENABLE_PINS
# error TMC22xx_HAS_ENABLE_PINS not defined
#endif

#ifndef TMC22xx_SINGLE_DRIVER
# error TMC22xx_SINGLE_DRIVER not defined
#endif

#ifndef TMC22xx_HAS_MUX
# error TMC22xx_HAS_MUX not defined
#endif

#ifndef TMC22xx_VARIABLE_NUM_DRIVERS
# error TMC22xx_VARIABLE_NUM_DRIVERS not defined
#endif

#ifndef TMC22xx_USE_SLAVEADDR
# error TMC22xx_USE_SLAVEADDR not defined
#endif

#ifndef TMC22xx_DEFAULT_STEALTHCHOP
# error TMC22xx_DEFAULT_STEALTHCHOP not defined
#endif

#define TMC22xx_SINGLE_UART		(TMC22xx_SINGLE_DRIVER || TMC22xx_HAS_MUX || TMC22xx_USE_SLAVEADDR)

#define RESET_MICROSTEP_COUNTERS_AT_INIT	0		// Duets use pulldown resistors on the step pins, so we don't get phantom microsteps at power up
#define USE_FAST_CRC	1

#include <Platform/RepRap.h>
#include <Platform/TaskPriorities.h>
#include <Movement/Move.h>
#include <Movement/StepTimer.h>
#include <Cache.h>
#include <General/Portability.h>
#include <Hardware/IoPorts.h>

#if SAME5x || SAMC21
# include <DmacManager.h>
# include <Serial.h>
# include <component/sercom.h>
#else
# include <pdc/pdc.h>
# include <pmc/pmc.h>
# include <uart/uart.h>
#endif

#if HAS_STALL_DETECT
static void InitStallDetectionLogic() noexcept;				// forward declaration
#endif

// Important note:
// The TMC22xx does handle a write request immediately followed by a read request to the same driver.
// The TMC2209 does _not_ handle back-to-back read requests to different drivers on the same multiplexer channel, it needs a short delay between them to allow the first driver to release the bus.

constexpr float MinimumOpenLoadMotorCurrent = 500;			// minimum current in mA for the open load status to be taken seriously
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
constexpr size_t TmcTaskStackWords = 150;					// 100 is sufficient unless we use debugPrintf in the code executed by the TMC task

constexpr uint16_t DriverNotPresentTimeouts = 20;

#if HAS_STALL_DETECT
const int DefaultStallDetectThreshold = 1;
const unsigned int DefaultMinimumStepsPerSecond = 200;		// for stall detection: 1 rev per second assuming 1.8deg/step, as per the TMC5160 datasheet
#endif

#if TMC22xx_VARIABLE_NUM_DRIVERS

static size_t numTmc22xxDrivers;
static inline size_t GetNumTmcDrivers() { return numTmc22xxDrivers; }

#else

static inline constexpr size_t GetNumTmcDrivers() { return MaxSmartDrivers; }

#endif

#if RESET_MICROSTEP_COUNTERS_AT_INIT
static Bitmap<uint16_t> driversStepped;
static_assert(driversStepped.MaxBits() >= MaxSmartDrivers);
#endif

enum class DriversState : uint8_t
{
	shutDown = 0,
	noPower,
	notInitialised,
	initialising,
#if RESET_MICROSTEP_COUNTERS_AT_INIT
	stepping,
	reinitialising,
#endif
	ready
};

static DriversState driversState = DriversState::shutDown;

#if TMC22xx_USE_SLAVEADDR && TMC22xx_HAS_MUX
static bool currentMuxState;
#endif

// GCONF register (0x00, RW)
constexpr uint8_t REGNUM_GCONF = 0x00;
constexpr uint32_t GCONF_USE_VREF = 1 << 0;					// use external VRef
constexpr uint32_t GCONF_INT_RSENSE = 1 << 1;				// use internal sense resistors
constexpr uint32_t GCONF_SPREAD_CYCLE = 1 << 2;				// use spread cycle mode (else stealthchop mode)
constexpr uint32_t GCONF_REV_DIR = 1 << 3;					// reverse motor direction
constexpr uint32_t GCONF_INDEX_OTPW = 1 << 4;				// INDEX output shows over temperature warning (else it shows first microstep position)
constexpr uint32_t GCONF_INDEX_PULSE = 1 << 5;				// INDEX output shows pulses from internal pulse generator, else as set by GCONF_INDEX_OTPW
constexpr uint32_t GCONF_UART = 1 << 6;						// PDN_UART used for UART interface (else used for power down)
constexpr uint32_t GCONF_MSTEP_REG = 1 << 7;				// microstep resolution set by MSTEP register (else by MS1 and MS2 pins)
constexpr uint32_t GCONF_MULTISTEP_FILT = 1 << 8;			// pulse generation optimised for >750Hz full stepping frequency
constexpr uint32_t GCONF_TEST_MODE = 1 << 9;				// test mode, do not set this bit for normal operation

constexpr uint32_t DefaultGConfReg =
#if TMC22xx_DEFAULT_STEALTHCHOP
									GCONF_UART | GCONF_MSTEP_REG | GCONF_MULTISTEP_FILT;
#else
									GCONF_UART | GCONF_MSTEP_REG | GCONF_MULTISTEP_FILT | GCONF_SPREAD_CYCLE;
#endif

// General configuration and status registers

// GSTAT register (0x01, RW). Write 1 bits to clear the flags.
constexpr uint8_t REGNUM_GSTAT = 0x01;
constexpr uint32_t GSTAT_RESET = 1 << 0;					// driver has been reset since last read
constexpr uint32_t GSTAT_DRV_ERR = 1 << 1;					// driver has been shut down due to over temp or short circuit
constexpr uint32_t GSTAT_UV_CP = 1 << 2;					// undervoltage on charge pump, driver disabled. Not latched so does not need to be cleared.

// IFCOUNT register (0x02, RO)
constexpr uint8_t REGNUM_IFCOUNT = 0x02;
constexpr uint32_t IFCOUNT_MASK = 0x000F;					// interface transmission counter

// SLAVECONF register (0x03, WO)
constexpr uint8_t REGNUM_SLAVECONF = 0x03;
constexpr uint32_t SLAVECONF_SENDDLY_8_BITS = 0 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_24_BITS = 2 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_40_BITS = 4 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_56_BITS = 6 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_72_BITS = 8 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_88_BITS = 10 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_104_BITS = 12 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_120_BITS = 14 << 8;

constexpr uint32_t DefaultSlaveConfReg = SLAVECONF_SENDDLY_8_BITS;	// we don't need any delay between transmission and reception

// OTP_PROG register (0x04, WO)
constexpr uint8_t REGNUM_OTP_PROG = 0x04;
constexpr uint32_t OTP_PROG_BIT_SHIFT = 0;
constexpr uint32_t OTP_PROG_BIT_MASK = 7 << OTP_PROG_BIT_SHIFT;
constexpr uint32_t OTP_PROG_BYTE_SHIFT = 4;
constexpr uint32_t OTP_PROG_BYTE_MASK = 3 << OTP_PROG_BYTE_SHIFT;
constexpr uint32_t OTP_PROG_MAGIC = 0xBD << 8;

// OTP_READ register (0x05, RO)
constexpr uint8_t REGNUM_OTP_READ = 0x05;
constexpr uint32_t OTP_READ_BYTE0_SHIFT = 0;
constexpr uint32_t OTP_READ_BYTE0_MASK = 0xFF << OTP_READ_BYTE0_SHIFT;
constexpr uint32_t OTP_READ_BYTE1_SHIFT = 8;
constexpr uint32_t OTP_READ_BYTE1_MASK = 0xFF << OTP_READ_BYTE1_SHIFT;
constexpr uint32_t OTP_READ_BYTE2_SHIFT = 16;
constexpr uint32_t OTP_READ_BYTE2_MASK = 0xFF << OTP_READ_BYTE2_SHIFT;

// IOIN register (0x06, RO)
constexpr uint8_t REGNUM_IOIN = 0x06;
constexpr uint32_t IOIN_220x_ENN = 1 << 0;
constexpr uint32_t IOIN_222x_PDN_UART = 1 << 1;
constexpr uint32_t IOIN_220x_MS1 = 1 << 2;
constexpr uint32_t IOIN_222x_SPREAD = 2 << 1;
constexpr uint32_t IOIN_220x_MS2 = 1 << 3;
constexpr uint32_t IOIN_222x_DIR = 1 << 3;
constexpr uint32_t IOIN_220x_DIAG = 1 << 4;
constexpr uint32_t IOIN_222x_ENN = 1 << 4;
constexpr uint32_t IOIN_222x_STEP = 1 << 5;
constexpr uint32_t IOIN_220x_PDN_UART = 1 << 6;
constexpr uint32_t IOIN_222x_MS1 = 1 << 6;
constexpr uint32_t IOIN_220x_STEP = 1 << 7;
constexpr uint32_t IOIN_222x_MS2 = 1 << 7;
constexpr uint32_t IOIN_IS_220x = 1 << 8;					// 1 if TMC220x, 0 if TMC222x
constexpr uint32_t IOIN_2209_SPREAD_EN = 1 << 8;
constexpr uint32_t IOIN_220x_DIR = 1 << 9;
constexpr uint32_t IOIN_VERSION_SHIFT = 24;
constexpr uint32_t IOIN_VERSION_MASK = 0xFF << IOIN_VERSION_SHIFT;

constexpr uint32_t IOIN_VERSION_2208_2224 = 0x20;			// version for TMC2208/2224
constexpr uint32_t IOIN_VERSION_2209 = 0x21;				// version for TMC2209

// FACTORY_CONF register (0x07, RW)
constexpr uint8_t REGNUM_FACTORY_CONF = 0x07;
constexpr uint32_t FACTORY_CONF_FCLKTRIM_SHIFT = 0;
constexpr uint32_t FACTORY_CONF_FCLKTRIM_MASK = 0x0F << FACTORY_CONF_FCLKTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_SHIFT = 8;
constexpr uint32_t FACTORY_CONF_OTTRIM_MASK = 0x03 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_143_120 = 0x00 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_150_120 = 0x01 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_150_143 = 0x02 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_157_143 = 0x03 << FACTORY_CONF_OTTRIM_SHIFT;

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
															// approx. 0.5 sec motor current reduction to low power

constexpr uint8_t REGNUM_TPOWER_DOWN = 0x11;	// wo, 8 bits, sets delay from standstill detection to motor current reduction
constexpr uint8_t REGNUM_TSTEP = 0x12;			// ro, 20 bits, measured time between two 1/256 microsteps, in clocks
constexpr uint8_t REGNUM_TPWMTHRS = 0x13;		// wo, 20 bits, upper velocity for StealthChop mode
constexpr uint8_t REGNUM_VACTUAL = 0x22;		// wo, 24 bits signed, sets motor velocity for continuous rotation

// Stallguard registers (TMC2209 only)
constexpr uint8_t REGNUM_TCOOLTHRS = 0x14;		// wo, 20-bit lower threshold velocity. CoolStep and the StallGuard DIAG output are enabled above this speed.
constexpr uint8_t REGNUM_SGTHRS = 0x40;			// w0, 8-bit stall detection threshold. Stall is signalled when SG_RESULT <= SGTHRS * 2.
constexpr uint8_t REGNUM_SG_RESULT = 0x41;		// 10-bit StallGard result, read-only. Bits 0 and 9 are always 0.
constexpr uint8_t REGNUM_COOLCONF = 0x42;		// 16-bit CoolStep control

constexpr uint32_t SG_RESULT_MASK = 1023;

// Minimum StallGuard value. Current is increased if SGRESULT < SEMIN * 32.
constexpr unsigned int COOLCONF_SEMIN_SHIFT = 0;
constexpr uint32_t COOLCONF_SEMIN_MASK = 0x000F << COOLCONF_SEMIN_SHIFT;
// Current increment steps per measured SG_RESULT value: 1,2,4,8
constexpr unsigned int COOLCONF_SEUP_SHIFT = 5;
constexpr uint32_t COOLCONF_SEMUP_MASK = 0x0003 << COOLCONF_SEUP_SHIFT;
// Hysteresis value for smart current control. Motor current is reduced if SG_RESULT >= (SEMIN+SEMAX+1)*32.
constexpr unsigned int COOLCONF_SEMAX_SHIFT = 8;
constexpr uint32_t COOLCONF_SEMAX_MASK = 0x000F << COOLCONF_SEMAX_SHIFT;
// Current down step speed. For each {32,8,2,1} SG_RESULT value, decrease by one
constexpr unsigned int COOLCONF_SEDN_SHIFT = 13;
constexpr uint32_t COOLCONF_SEDN_MASK = 0x0003 << COOLCONF_SEDN_SHIFT;
// Minimum current for smart current control, 0 = half of IRUN, 1 = 1/4 of IRUN
constexpr unsigned int COOLCONF_SEIMIN_SHIFT = 15;
constexpr uint32_t COOLCONF_SEIMIN_MASK = 0x0001 << COOLCONF_SEIMIN_SHIFT;

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

constexpr uint32_t DefaultChopConfReg = 0x00000053 | CHOPCONF_VSENSE_HIGH;	// this is the reset default + CHOPCONF_VSENSE_HIGH - CHOPCONF_INTPOL. Try it until we find something better.

#if RESET_MICROSTEP_COUNTERS_AT_INIT
constexpr uint32_t ChopConf256mstep = DefaultChopConfReg;	// the default uses x256 microstepping already
#endif

// DRV_STATUS register
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;
constexpr uint32_t TMC_RR_OT = 1u << 1;			// over temperature shutdown
constexpr uint32_t TMC_RR_OTPW = 1u << 0;		// over temperature warning
constexpr uint32_t TMC_RR_S2G = 15u << 2;		// short to ground counter (4 bits)
constexpr uint32_t TMC_RR_OLA = 1u << 6;		// open load A
constexpr uint32_t TMC_RR_OLB = 1u << 7;		// open load B
constexpr uint32_t TMC_RR_STST = 1u << 31;		// standstill detected
constexpr uint32_t TMC_RR_OPW_120 = 1u << 8;	// temperature threshold exceeded
constexpr uint32_t TMC_RR_OPW_143 = 1u << 9;	// temperature threshold exceeded
constexpr uint32_t TMC_RR_OPW_150 = 1u << 10;	// temperature threshold exceeded
constexpr uint32_t TMC_RR_OPW_157 = 1u << 11;	// temperature threshold exceeded
constexpr uint32_t TMC_RR_TEMPBITS = 15u << 8;	// all temperature threshold bits

constexpr uint32_t TMC_RR_RESERVED = (15u << 12) | (0x01FF << 21);	// reserved bits
constexpr uint32_t TMC_RR_SG = 1u << 12;		// this is a reserved bit, which we use to signal a stall

constexpr unsigned int TMC_RR_STST_BIT_POS = 31;
constexpr unsigned int TMC_RR_SG_BIT_POS = 12;

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC10D0024;			// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

// Send/receive data and CRC stuff

// Data format to write a driver register:
// Byte 0 sync byte, 0x05 (4 LSBs are don't cares but included in CRC)
// Byte 1 slave address, 0x00
// Byte 2 register register address to write | 0x80
// Bytes 3-6 32-bit data, MSB first
// Byte 7 8-bit CRC of bytes 0-6

// Data format to read a driver register:
// Byte 0 sync byte, 0x05 (4 LSBs are don't cares but included in CRC)
// Byte 1 slave address, 0x00
// Byte 2 register address to read (top bit clear)
// Byte 3 8-bit CRC of bytes 0-2

// Reply to a read request:
// Byte 0 sync byte, 0x05
// Byte 1 master address, 0xFF
// Byte 2 register address (top bit clear)
// Bytes 3-6 32-bit data, MSB first
// Byte 7 8-bit CRC

// Fast table-driven CRC-8. The result after we have taken the CRC of all bytes needs to be reflected.
// The CRC polynomial used by the TMC drivers is: X^8 + X^2 + X + 1 which is CRC-8-CCITT
static constexpr uint8_t crc_table[256] =
{
	0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
	0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
	0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
	0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
	0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
	0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
	0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
	0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
	0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
	0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
	0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
	0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
	0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
	0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
	0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
	0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};

// Add a byte to a CRC
static inline constexpr uint8_t CRCAddByte(uint8_t crc, uint8_t currentByte) noexcept
{
	return crc_table[crc ^ currentByte];
}

// Version of Reflect that can be declared constexpr so that we can use it in a static_assert
static inline constexpr uint8_t SlowReflect(uint8_t b) noexcept
{
	b = (b & 0b11110000) >> 4 | (b & 0b00001111) << 4;
	b = (b & 0b11001100) >> 2 | (b & 0b00110011) << 2;
	b = (b & 0b10101010) >> 1 | (b & 0b01010101) << 1;
	return b;
}

// Reverse the order of the bits
static inline uint8_t Reflect(uint8_t b) noexcept
{
#if SAMC21
	return SlowReflect(b);
#else
	uint32_t temp = b;
	asm("rbit %0,%1" : "=r" (temp) : "r" (temp));
	return temp >> 24;
#endif
}

static inline constexpr uint8_t CRCAddFinalByte(uint8_t crc, uint8_t finalByte) noexcept
{
	return SlowReflect(CRCAddByte(crc, finalByte));
}

static_assert(CRCAddFinalByte(CRCAddByte(CRCAddByte(0, 1), 2), 3) == 0x1E);

// CRC of the first byte we send in any request
static constexpr uint8_t InitialByteCRC = CRCAddByte(0, 0x05);

// CRC of the first two bytes we receive in any reply
static constexpr uint8_t InitialReceiveCrc = CRCAddByte(CRCAddByte(0, 0x05), 0xFF);

#if !TMC22xx_USE_SLAVEADDR

// CRC of the first 2 bytes we send in any request
static constexpr uint8_t InitialSendCRC = CRCAddByte(InitialByteCRC, 0x00);

// CRC of a complete request to read the IFCOUNT register
static constexpr uint8_t ReadIfcountCRC =
# if USE_FAST_CRC
	SlowReflect(CRCAddByte(InitialSendCRC, REGNUM_IFCOUNT));
# else
	CRCAddByte(InitialSendCRC, REGNUM_IFCOUNT);
# endif

#endif

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class TmcDriverState
{
public:
	void Init(uint32_t p_driverNumber
#if TMC22xx_HAS_ENABLE_PINS
							, Pin p_enablePin
#endif
#if HAS_STALL_DETECT
							, Pin p_diagPin
#endif
			 ) noexcept;
	void SetAxisNumber(size_t p_axisNumber) noexcept;
	uint32_t GetAxisNumber() const noexcept { return axisNumber; }
	void WriteAll() noexcept;
	bool SetMicrostepping(uint32_t shift, bool interpolate) noexcept;
	unsigned int GetMicrostepping(bool& interpolation) const noexcept;
	bool SetDriverMode(unsigned int mode) noexcept;
	DriverMode GetDriverMode() const noexcept;
	void SetCurrent(float current) noexcept;
	void Enable(bool en) noexcept;
#if HAS_STALL_DETECT
	void SetStallDetectThreshold(int sgThreshold) noexcept;
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(const StringRef& reply) const noexcept;
#endif
	void AppendDriverStatus(const StringRef& reply) noexcept;
	StandardDriverStatus GetStatus(bool accumulated, bool clearAccumulated) noexcept;
	uint8_t GetDriverNumber() const noexcept { return driverNumber; }
	bool UpdatePending() const noexcept;
#if TMC22xx_HAS_ENABLE_PINS
	bool UsesGlobalEnable() const noexcept { return enablePin == NoPin; }
#endif

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(SmartDriverRegister reg) const noexcept;
	GCodeResult GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;

	float GetStandstillCurrentPercent() const noexcept;
	void SetStandstillCurrentPercent(float percent) noexcept;

	bool DriverAssumedPresent() const noexcept { return numWrites != 0 || numTimeouts < DriverNotPresentTimeouts; }

	void TransferDone() noexcept SPEED_CRITICAL;	// called by the ISR when the SPI transfer has completed
	void StartTransfer() noexcept SPEED_CRITICAL;	// called to start a transfer
	void TransferTimedOut() noexcept
	{
		if (DriverAssumedPresent())
		{
			++numTimeouts;
		}
		AbortTransfer();
	}

	void DmaError() noexcept { ++numDmaErrors; AbortTransfer(); }
	void AbortTransfer() noexcept;

	void UpdateChopConfRegister() noexcept;					// calculate the chopper control register and flag it for sending

#if RESET_MICROSTEP_COUNTERS_AT_INIT
	void SetMicrostepping256() noexcept;					// temporarily set microstepping to x256 without overwriting the user setting

	bool GetMicrostepPosition(uint32_t& mscnt) noexcept
	{
		mscnt = readRegisters[ReadMsCnt];
		return mscnt < 1024;
	}

	void ClearMicrostepPosition() noexcept
	{
		readRegisters[ReadMsCnt] = 0xFFFFFFFF;				// special value to indicate that we don't know the microstep position
	}

	void RecordStepFailure() noexcept { hadStepFailure = true; }
#endif

	// Variables used by the ISR
	static uint32_t transferStartedTime;

	void UartTmcHandler() noexcept;							// core of the ISR for this driver
private:
	bool SetChopConf(uint32_t newVal) noexcept;
	void UpdateRegister(size_t regIndex, uint32_t regVal) noexcept;
	void UpdateCurrent() noexcept;
	void UpdateMaxOpenLoadStepInterval() noexcept;
#if HAS_STALL_DETECT
	void ResetLoadRegisters() noexcept
	{
		minSgLoadRegister = 9999;							// values read from the driver are in the range 0 to 1023, so 9999 indicates that it hasn't been read
	}
#endif

#if TMC22xx_HAS_MUX
	void SetUartMux() noexcept;
#endif
#if TMC22xx_USE_SLAVEADDR
	void SetupDMASend(uint8_t regnum, uint32_t outVal) noexcept SPEED_CRITICAL;			// set up the DMAC to send a register
	void SetupDMARead(uint8_t regnum) noexcept SPEED_CRITICAL;							// set up the DMAC to receive a register
#else
	static void SetupDMASend(uint8_t regnum, uint32_t outVal) noexcept SPEED_CRITICAL;	// set up the DMAC to send a register
	static void SetupDMARead(uint8_t regnum) noexcept SPEED_CRITICAL;					// set up the DMAC to receive a register
#endif

#if HAS_STALL_DETECT
	static constexpr unsigned int NumWriteRegisters = 9;		// the number of registers that we write to on a TMC2209
#else
	static constexpr unsigned int NumWriteRegisters = 6;		// the number of registers that we write to on a TMC2208/2224
#endif
	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteGConf = 0;				// microstepping
	static constexpr unsigned int WriteSlaveConf = 1;			// read response timing
	static constexpr unsigned int WriteChopConf = 2;			// enable/disable and microstep setting
	static constexpr unsigned int WriteIholdIrun = 3;			// current setting
	static constexpr unsigned int WritePwmConf = 4;				// read register select, sense voltage high/low sensitivity
	static constexpr unsigned int WriteTpwmthrs = 5;			// upper step rate limit for stealthchop
#if HAS_STALL_DETECT
	static constexpr unsigned int WriteTcoolthrs = 6;			// coolstep and stall DIAG output lower speed threshold
	static constexpr unsigned int WriteSgthrs = 7;				// stallguard threshold
	static constexpr unsigned int WriteCoolconf = 8;			// coolstep configuration
#endif
	static constexpr unsigned int WriteSpecial = NumWriteRegisters;

#if HAS_STALL_DETECT
	static constexpr unsigned int NumReadRegisters = 8;			// the number of registers that we read from on a TMC2209
#else
	static constexpr unsigned int NumReadRegisters = 7;			// the number of registers that we read from on a TMC2208/2224
#endif
	static const uint8_t ReadRegNumbers[NumReadRegisters];		// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadIoIn = 0;				// includes the version which we use to distinguish TMC2209 from 2208/2224
	static constexpr unsigned int ReadGStat = 1;			// global status
	static constexpr unsigned int ReadDrvStat = 2;			// drive status
	static constexpr unsigned int ReadMsCnt = 3;			// microstep counter
	static constexpr unsigned int ReadChopConf = 4;			// chopper control register - we read it to detect the VSENSE bit getting cleared
	static constexpr unsigned int ReadPwmScale = 5;			// PWM scaling
	static constexpr unsigned int ReadPwmAuto = 6;			// PWM scaling
#if HAS_STALL_DETECT
	static constexpr unsigned int ReadSgResult = 7;			// stallguard result, TMC2209 only
#endif
	static constexpr unsigned int ReadSpecial = NumReadRegisters;

	volatile uint32_t writeRegisters[NumWriteRegisters + 1];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters + 1];		// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedReadRegisters[NumReadRegisters];

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	volatile uint32_t registersToUpdate;					// bitmap of register indices whose values need to be sent to the driver chip

	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	float motorCurrent;										// the configured motor current in mA
	uint32_t maxOpenLoadStepInterval;						// the maximum step pulse interval for which we consider open load detection to be reliable

#if HAS_STALL_DETECT
	uint16_t minSgLoadRegister;								// the minimum value of the StallGuard bits we read
#endif

#if TMC22xx_SINGLE_UART
# if TMC22xx_USES_SERCOM
	static Sercom * const sercom;
	static uint8_t const sercomNumber;
# else
	static Uart * const uart;								// the UART that controls all drivers
# endif
#else
# if TMC22xx_USES_SERCOM
	Sercom * sercom;										// the SERCOM that controls this driver
	uint8_t sercomNumber;
# else
	Uart *uart;												// the UART that controls this driver
# endif
#endif

	// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
	// This is the message we send - volatile because we care about when it is written
	static volatile uint8_t sendData[12];

	// Buffer for the message we receive when reading data. The first 4 or 12 bytes bytes are our own transmitted data.
	static volatile uint8_t receiveData[20];

	uint16_t readErrors;									// how many read errors we had
	uint16_t writeErrors;									// how many write errors we had
	uint16_t numReads;										// how many successful reads we had
	uint16_t numWrites;										// how many successful writes we had
	uint16_t numTimeouts;									// how many times a transfer timed out
	uint16_t numDmaErrors;
	uint16_t badChopConfErrors;

#if TMC22xx_HAS_ENABLE_PINS
	Pin enablePin;											// the enable pin of this driver, if it has its own
#endif
#if HAS_STALL_DETECT
	Pin diagPin;
#endif
	uint8_t driverNumber;									// the number of this driver as addressed by the UART multiplexer
	uint8_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction
	uint8_t registerToRead;									// the next register we need to read
	uint8_t regnumBeingUpdated;								// which register we are sending
	uint8_t lastIfCount;									// the value of the IFCNT register last time we read it
	uint8_t failedOp;
#if TMC22xx_USE_SLAVEADDR
	uint8_t initialSendCRC;
	uint8_t readIfCountCRC;									// CRC of the message needed to read the IFCNT register
#endif
	volatile uint8_t specialReadRegisterNumber;				// the special register number we are reading
	volatile uint8_t specialWriteRegisterNumber;			// the special register number we are writing
	bool enabled;											// true if driver is enabled
#if RESET_MICROSTEP_COUNTERS_AT_INIT
	bool hadStepFailure;
#endif
};

// Static data members of class TmcDriverState

#if TMC22xx_SINGLE_UART
# if TMC22xx_USES_SERCOM
Sercom * const TmcDriverState::sercom = SERCOM_TMC22xx;
uint8_t const TmcDriverState::sercomNumber = TMC22xxSercomNumber;
# else
Uart * const TmcDriverState::uart = UART_TMC22xx;
# endif
#endif

// TMC22xx management task
static Task<TmcTaskStackWords> tmcTask;

#if TMC22xx_USES_SERCOM
static DmaCallbackReason dmaFinishedReason;
#else
static bool dmaFinished;
#endif

// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
// This is the message we send - volatile because we care about when it is written, and dword-aligned so that we can use 32-bit mode on the SAME5x
alignas(4) volatile uint8_t TmcDriverState::sendData[12] =
{
	0x05, 0x00,							// sync byte and slave address
	0x00,								// register address and write flag (filled in)
	0x00, 0x00, 0x00, 0x00,				// value to write (if writing), or 1 byte of CRC if read request (filled in)
	0x00,								// CRC of write request (filled in)
	0x05, 0x00,							// sync byte and slave address
	REGNUM_IFCOUNT,						// register we want to read
#if TMC22xx_USE_SLAVEADDR
	0x00								// CRC (changes according to the slave address)
#else
	ReadIfcountCRC						// CRC
#endif
};

constexpr size_t SendDataSlaveAddressIndex0 = 1;
constexpr size_t SendDataSlaveAddressIndex1 = 9;
constexpr size_t SendDataCRCIndex0 = 7;
constexpr size_t SendDataCRCIndex1 = 11;

// Buffer for the message we receive when reading data, dword-aligned so that we can use 32-bit mode on the SAME5x. The first 4 or 12 bytes bytes are our own transmitted data.
// Align on a 16-bit boundary to make sure it covers only 2 cache lines not 3
alignas(16) volatile uint8_t TmcDriverState::receiveData[20];

constexpr uint8_t TmcDriverState::WriteRegNumbers[NumWriteRegisters] =
{
	REGNUM_GCONF,
	REGNUM_SLAVECONF,
	REGNUM_CHOPCONF,
	REGNUM_IHOLDIRUN,
	REGNUM_PWMCONF,
	REGNUM_TPWMTHRS,
#if HAS_STALL_DETECT
	// The rest are on TMC2209 only
	REGNUM_TCOOLTHRS,
	REGNUM_SGTHRS,
	REGNUM_COOLCONF
#endif
};

constexpr uint8_t TmcDriverState::ReadRegNumbers[NumReadRegisters] =
{
	REGNUM_IOIN,						// tells us whether we have a TMC2208/24 or a TMC2209
	REGNUM_GSTAT,
	REGNUM_DRV_STATUS,
	REGNUM_MSCNT,
	REGNUM_CHOPCONF,
	REGNUM_PWM_SCALE,
	REGNUM_PWM_AUTO,
#if HAS_STALL_DETECT
	REGNUM_SG_RESULT					// TMC2209 only
#endif
};

// State structures for all drivers
static TmcDriverState driverStates[MaxSmartDrivers];

inline bool TmcDriverState::UpdatePending() const noexcept
{
	return registersToUpdate != 0;
}

// Set up the PDC or DMAC to send a register
inline void TmcDriverState::SetupDMASend(uint8_t regNum, uint32_t regVal) noexcept
{
#if TMC22xx_USES_SERCOM
	DmacManager::DisableChannel(DmacChanTmcTx);
	DmacManager::DisableChannel(DmacChanTmcRx);
#elif defined(SAM4E) || defined(SAM4S)
	Pdc * const pdc = uart_get_pdc_base(uart);
	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);	// disable the PDC
#else
# error Unsupported processor
#endif

#if TMC22xx_USE_SLAVEADDR
	const uint8_t slaveAddress = driverNumber & 3u;
	sendData[SendDataSlaveAddressIndex0] = slaveAddress;
	sendData[SendDataSlaveAddressIndex1] = slaveAddress;
	uint8_t crc = initialSendCRC;
#else
	uint8_t crc = InitialSendCRC;
#endif
	{
		const uint8_t byte =  regNum | 0x80;
		sendData[2] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)(regVal >> 24);
		sendData[3] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)(regVal >> 16);
		sendData[4] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)(regVal >> 8);
		sendData[5] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)regVal;
		sendData[6] = byte;
		crc = CRCAddByte(crc, byte);
	}

	sendData[SendDataCRCIndex0] =
#if USE_FAST_CRC
		Reflect(crc);
#else
		crc;
#endif

#if TMC22xx_USE_SLAVEADDR
	sendData[SendDataCRCIndex1] = readIfCountCRC;
#endif

	Cache::FlushBeforeDMASend(sendData, sizeof(sendData));
	Cache::FlushBeforeDMAReceive(receiveData, sizeof(receiveData));

#if TMC22xx_USES_SERCOM
	DmacManager::SetSourceAddress(DmacChanTmcTx, sendData);
	DmacManager::SetDestinationAddress(DmacChanTmcTx, &(sercom->USART.DATA));
	DmacManager::SetBtctrl(DmacChanTmcTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanTmcTx, 12/4);
	DmacManager::SetTriggerSourceSercomTx(DmacChanTmcTx, sercomNumber);

	DmacManager::SetSourceAddress(DmacChanTmcRx, &(sercom->USART.DATA));
	DmacManager::SetDestinationAddress(DmacChanTmcRx, receiveData);
	DmacManager::SetBtctrl(DmacChanTmcRx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanTmcRx, 20/4);
	DmacManager::SetTriggerSourceSercomRx(DmacChanTmcRx, sercomNumber);

	DmacManager::EnableChannel(DmacChanTmcTx, DmacPrioTmcTx);
	DmacManager::EnableChannel(DmacChanTmcRx, DmacPrioTmcRx);
#elif defined(SAM4E) || defined(SAM4S)
	pdc->PERIPH_TPR = reinterpret_cast<uint32_t>(sendData);
	pdc->PERIPH_TCR = 12;											// number of bytes to send: 8 bytes send request + 4 bytes read IFCOUNT request

	pdc->PERIPH_RPR = reinterpret_cast<uint32_t>(receiveData);
	pdc->PERIPH_RCR = 20;											// number of bytes to receive: the sent data + 8 bytes of received data

	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);		// enable the PDC to transmit and receive
#else
# error Unsupported processor
#endif
}

// Set up the PDC or DMAC to send a register read request and receive the status
inline void TmcDriverState::SetupDMARead(uint8_t regNum) noexcept
{
#if TMC22xx_USES_SERCOM
	DmacManager::DisableChannel(DmacChanTmcTx);
	DmacManager::DisableChannel(DmacChanTmcRx);
#elif defined(SAM4E) || defined(SAM4S)
	Pdc * const pdc = uart_get_pdc_base(uart);
	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);	// disable the PDC
#else
# error Unsupported processor
#endif

#if TMC22xx_USE_SLAVEADDR
	sendData[SendDataSlaveAddressIndex0] = driverNumber & 3u;
	uint8_t crc = initialSendCRC;
#else
	sendData[SendDataSlaveAddressIndex0] = 0;
	uint8_t crc = InitialSendCRC;
#endif
	sendData[2] = regNum;
	sendData[3] =
#if USE_FAST_CRC
		Reflect(CRCAddByte(crc, regNum));
#else
		CRCAddByte(crc, regNum);
#endif

	Cache::FlushBeforeDMASend(sendData, sizeof(sendData));
	Cache::FlushBeforeDMAReceive(receiveData, sizeof(receiveData));

#if TMC22xx_USES_SERCOM
	DmacManager::SetSourceAddress(DmacChanTmcTx, sendData);
	DmacManager::SetDestinationAddress(DmacChanTmcTx, &(sercom->USART.DATA));
	DmacManager::SetBtctrl(DmacChanTmcTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanTmcTx, 4/4);
	DmacManager::SetTriggerSourceSercomTx(DmacChanTmcTx, sercomNumber);

	DmacManager::SetSourceAddress(DmacChanTmcRx, &(sercom->USART.DATA));
	DmacManager::SetDestinationAddress(DmacChanTmcRx, receiveData);
	DmacManager::SetBtctrl(DmacChanTmcRx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanTmcRx, 12/4);
	DmacManager::SetTriggerSourceSercomRx(DmacChanTmcRx, sercomNumber);

	DmacManager::EnableChannel(DmacChanTmcTx, DmacPrioTmcTx);
	DmacManager::EnableChannel(DmacChanTmcRx, DmacPrioTmcRx);
#elif defined(SAM4E) || defined(SAM4S)
	pdc->PERIPH_TPR = reinterpret_cast<uint32_t>(sendData);
	pdc->PERIPH_TCR = 4;											// send a 4 byte read data request

	pdc->PERIPH_RPR = reinterpret_cast<uint32_t>(receiveData);
	pdc->PERIPH_RCR = 12;											// receive the 4 bytes we sent + 8 bytes of received data

	pdc->PERIPH_PTCR = (PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);		// enable the PDC to transmit and receive
#else
# error Unsupported processor
#endif
}

// Update the maximum step pulse interval at which we consider open load detection to be reliable
void TmcDriverState::UpdateMaxOpenLoadStepInterval() noexcept
{
	const uint32_t defaultMaxInterval = StepClockRate/MinimumOpenLoadFullStepsPerSec;
	if ((writeRegisters[WriteGConf] & GCONF_SPREAD_CYCLE) != 0)
	{
		maxOpenLoadStepInterval = defaultMaxInterval;
	}
	else
	{
		// In stealthchop mode open load detection in unreliable, so disable it below the speed at which we switch to spreadCycle
		const uint32_t tpwmthrs = writeRegisters[WriteTpwmthrs] & 0x000FFFFF;
		// tpwmthrs is the 20-bit interval between 1/256 microsteps threshold, in clock cycles @ 12MHz.
		// We need to convert it to the interval between full steps, measured in our step clocks, less about 20% to allow some margin.
		// So multiply by the step clock rate divided by 12MHz, also multiply by 256 less 20%.
		constexpr uint32_t conversionFactor = ((256 - 51) * (StepClockRate/1000000))/12;
		const uint32_t fullStepClocks = tpwmthrs * conversionFactor;
		maxOpenLoadStepInterval = min<uint32_t>(fullStepClocks, defaultMaxInterval);
	}
}

// Set a register value and flag it for updating
void TmcDriverState::UpdateRegister(size_t regIndex, uint32_t regVal) noexcept
{
	{
		AtomicCriticalSectionLocker lock;
		writeRegisters[regIndex] = regVal;
		registersToUpdate |= (1u << regIndex);								// flag it for sending
	}

	if (regIndex == WriteGConf || regIndex == WriteTpwmthrs)
	{
		UpdateMaxOpenLoadStepInterval();
	}
}

// Calculate the chopper control register and flag it for sending
void TmcDriverState::UpdateChopConfRegister() noexcept
{
	// It's critical that CHOPCONF_VSENSE_HIGH is always set, so we or-it in here just in case
	UpdateRegister(WriteChopConf, ((enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK) | CHOPCONF_VSENSE_HIGH);
}

#if RESET_MICROSTEP_COUNTERS_AT_INIT
// Temporarily set microstepping to x256 and driver disabled without overwriting the user setting
void TmcDriverState::SetMicrostepping256() noexcept
{
	UpdateRegister(WriteChopConf, ChopConf256mstep);
}
#endif

// Initialise the state of the driver and its CS pin
void TmcDriverState::Init(uint32_t p_driverNumber
#if TMC22xx_HAS_ENABLE_PINS
							, Pin p_enablePin
#endif
#if HAS_STALL_DETECT
							, Pin p_diagPin
#endif
) noexcept
pre(!driversPowered)
{
	driverNumber = p_driverNumber;
	axisNumber = p_driverNumber;										// assume straight-through axis mapping initially
#if TMC22xx_HAS_ENABLE_PINS
	enablePin = p_enablePin;											// this is NoPin for the built-in drivers
	IoPort::SetPinMode(p_enablePin, OUTPUT_HIGH);
#endif

#if HAS_STALL_DETECT
	diagPin = p_diagPin;
	IoPort::SetPinMode(p_diagPin, INPUT_PULLDOWN);						// pull down not up so that missing drivers don't signal stalls
#endif

#if !TMC22xx_SINGLE_UART
# if TMC22xx_USES_SERCOM
	sercom = TMC22xxSercoms[p_driverNumber];
	sercomNumber = TMC22xxSercomNumbers[p_driverNumber];
# else
	uart = TMC22xxUarts[p_driverNumber];
# endif
#endif

#if TMC22xx_USE_SLAVEADDR
	initialSendCRC = CRCAddByte(InitialByteCRC, driverNumber & 3u);		// CRC of the first 2 bytes of any transmission
	readIfCountCRC =
# if USE_FAST_CRC
		Reflect(CRCAddByte(initialSendCRC, REGNUM_IFCOUNT));
# else
		CRCAddByte(initialSendCRC, REGNUM_IFCOUNT);
# endif
#endif
	enabled = false;
#if RESET_MICROSTEP_COUNTERS_AT_INIT
	hadStepFailure = false;
#endif
	registersToUpdate = 0;
	specialReadRegisterNumber = specialWriteRegisterNumber = 0xFF;
	motorCurrent = 0.0;
	standstillCurrentFraction = (uint8_t)min<uint32_t>((DefaultStandstillCurrentPercent * 256)/100, 255);
	UpdateRegister(WriteGConf, DefaultGConfReg);
	UpdateRegister(WriteSlaveConf, DefaultSlaveConfReg);
	configuredChopConfReg = DefaultChopConfReg;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);	// this also updates the chopper control register
	UpdateRegister(WriteIholdIrun, DefaultIholdIrunReg);
	UpdateRegister(WritePwmConf, DefaultPwmConfReg);
	UpdateRegister(WriteTpwmthrs, DefaultTpwmthrsReg);
#if HAS_STALL_DETECT
	SetStallDetectThreshold(DefaultStallDetectThreshold);
	SetStallMinimumStepsPerSecond(DefaultMinimumStepsPerSecond);
	UpdateRegister(WriteCoolconf, 0);									// coolStep disabled
#endif

	for (size_t i = 0; i < NumReadRegisters; ++i)
	{
		accumulatedReadRegisters[i] = readRegisters[i] = 0;				// clear all read registers so that we don't use dud values, in particular we don't know the driver type yet
	}

#if RESET_MICROSTEP_COUNTERS_AT_INIT
	ClearMicrostepPosition();
#endif

	regnumBeingUpdated = 0xFF;
	failedOp = 0xFF;
	registerToRead = 0;
	lastIfCount = 0;
	readErrors = writeErrors = numReads = numWrites = numTimeouts = numDmaErrors = badChopConfErrors = 0;
#if HAS_STALL_DETECT
	ResetLoadRegisters();
#endif
}

#if HAS_STALL_DETECT

void TmcDriverState::SetStallDetectThreshold(int sgThreshold) noexcept
{
	// M915 was originally defined for Stallguard 2, which uses values of -64 (most sensitive) to +63 (least sensitive)
	// Stallguard 4 on the TMC2209 uses 0 (least sensitive) to 255 (most sensitive). So we map values -128..+127 to 255..0
	const uint32_t sgthrs = (uint32_t)(127 - constrain<int>(sgThreshold, -128, 127));
	UpdateRegister(WriteSgthrs, sgthrs);
}

void TmcDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept
{
	UpdateRegister(WriteTcoolthrs, (12000000 + (128 * stepsPerSecond))/(256 * stepsPerSecond));
}

void TmcDriverState::AppendStallConfig(const StringRef& reply) const noexcept
{
	// Map stall sensitivity value 0..255 to 128..-128
	const int threshold = 127 - (int)writeRegisters[WriteSgthrs];
	reply.catf("stall threshold %d, steps/sec %" PRIu32 ", coolstep %" PRIx32,
				threshold, 12000000 / (256 * writeRegisters[WriteTcoolthrs]), writeRegisters[WriteCoolconf] & 0xFFFF);
}

#endif

inline void TmcDriverState::SetAxisNumber(size_t p_axisNumber) noexcept
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void TmcDriverState::WriteAll() noexcept
{
	registersToUpdate = (1u << NumWriteRegisters) - 1;
}

float TmcDriverState::GetStandstillCurrentPercent() const noexcept
{
	return (float)(standstillCurrentFraction * 100)/256;
}

void TmcDriverState::SetStandstillCurrentPercent(float percent) noexcept
{
	standstillCurrentFraction = (uint8_t)constrain<long>(lrintf((percent * 256)/100), 0, 255);
	UpdateCurrent();
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift).
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
	interpolation = (writeRegisters[WriteChopConf] & CHOPCONF_INTPOL) != 0;
	return 1u << microstepShiftFactor;
}

bool TmcDriverState::SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept
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

#if HAS_STALL_DETECT
	case SmartDriverRegister::coolStep:
		UpdateRegister(WriteCoolconf, regVal & ((1u << 16) - 1));
		return true;
#endif

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

	case SmartDriverRegister::mstepPos:
		return readRegisters[ReadMsCnt];

	case SmartDriverRegister::pwmScale:
		return readRegisters[ReadPwmScale];

	case SmartDriverRegister::pwmAuto:
		return readRegisters[ReadPwmAuto];

	case SmartDriverRegister::hdec:
	case SmartDriverRegister::coolStep:
	default:
		return 0;
	}
}

GCodeResult TmcDriverState::GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept
{
	if (specialReadRegisterNumber == 0xFE)
	{
		reply.printf("Register 0x%02x value 0x%08" PRIx32, regNum, readRegisters[ReadSpecial]);
		specialReadRegisterNumber = 0xFF;
		return GCodeResult::ok;
	}

	if (specialReadRegisterNumber == 0xFF)
	{
		specialReadRegisterNumber = regNum;
	}
	return GCodeResult::notFinished;
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

// Set the driver mode
bool TmcDriverState::SetDriverMode(unsigned int mode) noexcept
{
	switch (mode)
	{
	case (unsigned int)DriverMode::spreadCycle:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] | GCONF_SPREAD_CYCLE);
		return true;

	case (unsigned int)DriverMode::stealthChop:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_SPREAD_CYCLE);
		return true;

	default:
		return false;
	}
}

// Get the driver mode
DriverMode TmcDriverState::GetDriverMode() const noexcept
{
	return ((writeRegisters[WriteGConf] & GCONF_SPREAD_CYCLE) != 0) ? DriverMode::spreadCycle : DriverMode::stealthChop;
}

// Set the motor current
void TmcDriverState::SetCurrent(float current) noexcept
{
	motorCurrent = constrain<float>(current, 50.0, MaximumMotorCurrent);
	UpdateCurrent();
}

void TmcDriverState::UpdateCurrent() noexcept
{
	const float idealIRunCs = DriverCsMultiplier * motorCurrent;
	const uint32_t iRunCsBits = constrain<uint32_t>((unsigned int)(idealIRunCs + 0.2), 1, 32) - 1;
	const float idealIHoldCs = idealIRunCs * standstillCurrentFraction * (1.0/256.0);
	const uint32_t iHoldCsBits = constrain<uint32_t>((unsigned int)(idealIHoldCs + 0.2), 1, 32) - 1;
	UpdateRegister(WriteIholdIrun,
					  (writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK))
					| (iRunCsBits << IHOLDIRUN_IRUN_SHIFT)
					| (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
}

// Enable or disable the driver
void TmcDriverState::Enable(bool en) noexcept
{
	if (enabled != en)
	{
		enabled = en;
#if TMC22xx_HAS_ENABLE_PINS
		if (enablePin != NoPin)
		{
			digitalWrite(enablePin, !en);			// we assume that smart drivers always have active low enables
		}
#endif
		UpdateChopConfRegister();
	}
}

StandardDriverStatus TmcDriverState::GetStatus(bool accumulated, bool clearAccumulated) noexcept
{
	StandardDriverStatus rslt;
	if (DriverAssumedPresent())
	{
		uint32_t status;
		if (accumulated)
		{
			AtomicCriticalSectionLocker lock;

			status = accumulatedReadRegisters[ReadDrvStat];
			if (clearAccumulated)
			{
				accumulatedReadRegisters[ReadDrvStat] = readRegisters[ReadDrvStat];
			}
		}
		else
		{
			status = readRegisters[ReadDrvStat];
			if (!enabled)
			{
				status &= ~(TMC_RR_OLA | TMC_RR_OLB);
			}
		}
#if HAS_STALL_DETECT
		if (IoPort::ReadPin(diagPin))
		{
			status |= TMC_RR_SG;
		}
#endif

		// The lowest 8 bits of StandardDriverStatus have the same meanings as for the TMC2209 status
		rslt.all = status & 0x000000FF;
		rslt.all |= ExtractBit(status, TMC_RR_STST_BIT_POS, StandardDriverStatus::StandstillBitPos);	// put the standstill bit in the right place
		rslt.all |= ExtractBit(status, TMC_RR_SG_BIT_POS, StandardDriverStatus::StallBitPos);			// put the stall bit in the right place
#if HAS_STALL_DETECT
		rslt.sgresultMin = minSgLoadRegister;
#endif
	}
	else
	{
		rslt.all = 0;
		rslt.notPresent = true;
	}
	return rslt;
}

// Append any additional driver status to a string, and reset the min/max load values
void TmcDriverState::AppendDriverStatus(const StringRef& reply) noexcept
{
#if RESET_MICROSTEP_COUNTERS_AT_INIT
	if (hadStepFailure)
	{
		reply.cat(", stepFail");
	}
#endif

#if HAS_STALL_DETECT
	if (minSgLoadRegister <= 1023)
	{
		reply.catf(", SG min %u", minSgLoadRegister);
	}
	else
	{
		reply.cat(", SG min n/a");
	}
	ResetLoadRegisters();
#endif

	reply.catf(", read errors %u, write errors %u, ifcnt %u, reads %u, writes %u, timeouts %u, DMA errors %u, CC errors %u",
					readErrors, writeErrors, lastIfCount, numReads, numWrites, numTimeouts, numDmaErrors, badChopConfErrors);
	if (failedOp != 0xFF)
	{
		reply.catf(", failedOp 0x%02x", failedOp);
		failedOp = 0xFF;
	}
	readErrors = writeErrors = numReads = numWrites = numTimeouts = numDmaErrors = badChopConfErrors = 0;
}

// This is called by the ISR when the SPI transfer has completed
inline void TmcDriverState::TransferDone() noexcept
{
	Cache::InvalidateAfterDMAReceive(receiveData, sizeof(receiveData));
	if (sendData[2] & 0x80)								// if we were writing a register
	{
		const uint8_t currentIfCount = receiveData[18];
		// Note, the TMC2209 IFCNT register seems to start at a random value, so we expect to get a write error on the first write.
		// We could read IFCNT once to get the initial value, but doing the first write twice does no harm.
		if (   regnumBeingUpdated <= NumWriteRegisters
			&& currentIfCount == (uint8_t)(lastIfCount + 1)
			&& (sendData[2] & 0x7F) == ((regnumBeingUpdated == WriteSpecial) ? specialWriteRegisterNumber : WriteRegNumbers[regnumBeingUpdated])
		   )
		{
			++numWrites;
			registersToUpdate &= ~(1u << regnumBeingUpdated);
			// The value to be written may have changed since we sent it, so check that we wrote the latest data
			if (LoadBE32(const_cast<const uint8_t *>(sendData + 3)) != writeRegisters[regnumBeingUpdated])
			{
				registersToUpdate |= 1u << regnumBeingUpdated;
			}
		}
		else
		{
			++writeErrors;
		}
		lastIfCount = currentIfCount;
		regnumBeingUpdated = 0xFF;
	}
	else if (driversState != DriversState::noPower)		// we don't check the CRC, so only accept the result if power is still good
	{
		const uint8_t readRegNumber = (registerToRead < NumReadRegisters) ? ReadRegNumbers[registerToRead] : specialReadRegisterNumber;
		if (sendData[2] == readRegNumber
			&& readRegNumber == receiveData[6]
			&& receiveData[4] == 0x05
			&& receiveData[5] == 0xFF
			&& Reflect(CRCAddByte(CRCAddByte(CRCAddByte(CRCAddByte(CRCAddByte(InitialReceiveCrc, receiveData[6]), receiveData[7]), receiveData[8]), receiveData[9]), receiveData[10])) == receiveData[11]
		   )
		{
			// We asked to read the scheduled read register, and the sync byte, slave address and register number in the received message match, also the CRC is correct
			uint32_t regVal = ((uint32_t)receiveData[7] << 24) | ((uint32_t)receiveData[8] << 16) | ((uint32_t)receiveData[9] << 8) | receiveData[10];
			if (registerToRead == ReadDrvStat)
			{
				uint32_t interval;
				if (   (regVal & TMC_RR_STST) != 0
					|| (interval = reprap.GetMove().GetStepInterval(axisNumber, microstepShiftFactor)) == 0		// get the full step interval
					|| interval > maxOpenLoadStepInterval
					|| motorCurrent < MinimumOpenLoadMotorCurrent
				   )
				{
					regVal &= ~(TMC_RR_OLA | TMC_RR_OLB);				// open load bits are unreliable at standstill and low speeds
				}
			}
			else if (registerToRead == ReadChopConf)
			{
				// Sometimes the CHOPCONF register VSENSE bit gets cleared unexpectedly. Now we monitor it to check that the register has the correct value.
				if (regVal != writeRegisters[WriteChopConf] && (registersToUpdate & (1u << WriteChopConf)) == 0)
				{
					registersToUpdate |= (1u << WriteChopConf);
					++badChopConfErrors;
				}
			}
#if HAS_STALL_DETECT
			else if (registerToRead == ReadSgResult)
			{
				const uint16_t sgResult = regVal & SG_RESULT_MASK;
				if (sgResult < minSgLoadRegister)
				{
					minSgLoadRegister = sgResult;
				}
			}
#endif
			readRegisters[registerToRead] = regVal;

			if (registerToRead == ReadSpecial)
			{
				specialReadRegisterNumber = 0xFE;						// set it to 0xFE to indicate that we have read it and to prevent it being read again
				registerToRead = 0;
			}
			else
			{
				accumulatedReadRegisters[registerToRead] |= regVal;
				++registerToRead;
				if (registerToRead == ReadSpecial && specialReadRegisterNumber >= 0x80)
				{
					registerToRead = 0;
				}
			}
			++numReads;
		}
		else
		{
			++readErrors;
		}
	}
}

// This is called to abandon the current transfer, if any
void TmcDriverState::AbortTransfer() noexcept
{
#if TMC22xx_USES_SERCOM
	DmacManager::DisableChannel(DmacChanTmcTx);
	DmacManager::DisableChannel(DmacChanTmcRx);
	sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);
	while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
	uart->UART_IDR = UART_IDR_ENDRX;				// disable end-of-receive interrupt
	uart_get_pdc_base(uart)->PERIPH_PTCR = (PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);	// disable the PDC
	uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;
#endif
	failedOp = sendData[2];
}

#if TMC22xx_HAS_MUX

// Set up the UART multiplexer to address the selected driver
inline void TmcDriverState::SetUartMux() noexcept
{
# if TMC22xx_USE_SLAVEADDR
	const bool newMuxState = ((driverNumber & 0x04) == 0);		// Duet 3 Mini has the mux outputs swapped, so use == instead of !=
	if (newMuxState == currentMuxState)
	{
		// A TMC2209 turns off its transmitter 4 bit times after the end of the last byte.
		// So if we didn't change the mux, we need a delay here. In fact, even 8 bit times isn't enough delay.
		delay(2);
	}
	else
	{
		digitalWrite(TMC22xxMuxPins[0], newMuxState);
	}
	currentMuxState = newMuxState;
# else
	digitalWrite(TMC22xxMuxPins[0], (driverNumber & 0x01) != 0);
	digitalWrite(TMC22xxMuxPins[1], (driverNumber & 0x02) != 0);
	digitalWrite(TMC22xxMuxPins[2], (driverNumber & 0x04) != 0);
# endif
}

#endif

// This is called from the ISR or elsewhere to start a new SPI transfer. Inlined for ISR speed.
inline void TmcDriverState::StartTransfer() noexcept
{
#if TMC22xx_HAS_MUX
	SetUartMux();
#elif TMC22xx_USE_SLAVEADDR
	delay(2);																				// give the previous TMC22xx driver time to get off the bus
#endif

	// Find which register to send. The common case is when no registers need to be updated.
	if (registersToUpdate != 0)
	{
		const size_t regNum = LowestSetBit(registersToUpdate);

		// Kick off a transfer for the register to write
		AtomicCriticalSectionLocker lock;

		regnumBeingUpdated = regNum;
#if TMC22xx_USES_SERCOM
		sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// disable transmitter and receiver, reset receiver
		while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;										// reset transmitter and receiver
#endif
		const uint8_t regNumber = (regNum < WriteSpecial) ? WriteRegNumbers[regNum] : specialWriteRegisterNumber;
		SetupDMASend(regNumber, writeRegisters[regNum]);									// set up the DMAC
#if TMC22xx_USES_SERCOM
		dmaFinishedReason = DmaCallbackReason::none;
		DmacManager::EnableCompletedInterrupt(DmacChanTmcRx);
		sercom->USART.CTRLB.reg |= (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);		// enable transmitter and receiver
#else
		uart->UART_IER = UART_IER_ENDRX;													// enable end-of-transfer interrupt
		uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;										// enable transmitter and receiver
#endif
	}
	else
	{
		// Read a register
		regnumBeingUpdated = 0xFF;
		AtomicCriticalSectionLocker lock;
#if TMC22xx_USES_SERCOM
		sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// disable transmitter and receiver, reset receiver
		while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;										// reset transmitter and receiver
#endif

		const uint8_t readRegNumber = (registerToRead < NumReadRegisters) ? ReadRegNumbers[registerToRead] : specialReadRegisterNumber;
		SetupDMARead(readRegNumber);														// set up the DMAC

#if TMC22xx_USES_SERCOM
		dmaFinishedReason = DmaCallbackReason::none;
		DmacManager::EnableCompletedInterrupt(DmacChanTmcRx);
		sercom->USART.CTRLB.reg |= (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);		// enable transmitter and receiver
#else
		uart->UART_IER = UART_IER_ENDRX;													// enable end-of-receive interrupt
		uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;										// enable transmitter and receiver
#endif
	}
}

// ISR(s) for the UART(s)

inline void TmcDriverState::UartTmcHandler() noexcept
{
#if !TMC22xx_SINGLE_UART
# if TMC22xx_USES_SERCOM
	DmacManager::DisableCompletedInterrupt(DmacChanTmcRx);
# else
	uart->UART_IDR = UART_IDR_ENDRX;														// disable the PDC interrupt
# endif
#endif
	TransferDone();																			// tidy up after the transfer we just completed
}

#if TMC22xx_SINGLE_UART

# if TMC22xx_USES_SERCOM

// DMA complete callback
void TransferCompleteCallback(CallbackParameter, DmaCallbackReason reason) noexcept
{
	dmaFinishedReason = reason;
	tmcTask.GiveFromISR();
}

# else

#  ifndef TMC22xx_UART_Handler
#   error TMC handler name not defined
#  endif

// ISR for the single UART
extern "C" void TMC22xx_UART_Handler() noexcept SPEED_CRITICAL;

void TMC22xx_UART_Handler() noexcept
{
	UART_TMC22xx->UART_IDR = UART_IDR_ENDRX;			// disable the interrupt
	dmaFinished = true;
	tmcTask.GiveFromISR();
}

# endif

#else

// ISRs for the individual UARTs
extern "C" void UART_TMC_DRV0_Handler() noexcept SPEED_CRITICAL;
void UART_TMC_DRV0_Handler() noexcept
{
	driverStates[0].UartTmcHandler();
}

extern "C" void UART_TMC_DRV1_Handler() noexcept SPEED_CRITICAL;
void UART_TMC_DRV1_Handler() noexcept
{
	driverStates[1].UartTmcHandler();
}

#endif

// Do a UART transaction with the specified driver number. Called from the TMC task loop.
// Returns true if the transaction was completed successfully.
bool DoTransaction(size_t driverNumber)
{
	TmcDriverState *currentDriver;
#if TMC22xx_SINGLE_DRIVER
	currentDriver = driverStates;
#elif TMC22xx_USE_SLAVEADDR && TMC22xx_HAS_MUX
	const size_t mappedDriverNumber = ((driverNumber & 1u) << 2) | (driverNumber >> 1);	// this assumes we have between 5 and 8 drivers and a 2-way multiplexer
	currentDriver = &driverStates[mappedDriverNumber];
#else
	currentDriver = &driverStates[driverNumber];
#endif
#if TMC22xx_USES_SERCOM
	dmaFinishedReason = DmaCallbackReason::none;
#else
	dmaFinished = false;
#endif
	currentDriver->StartTransfer();

	// Wait for the end-of-transfer interrupt
	const bool timedOut = !TaskBase::Take(TransferTimeout);
#if TMC22xx_USES_SERCOM
	DmacManager::DisableCompletedInterrupt(DmacChanTmcRx);
#elif TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
	UART_TMC22xx->UART_IDR = UART_IDR_ENDRX;			// disable the interrupt
#else
	// Multiple UARTS - need to disable the right one
# error code not written
#endif

	if (timedOut)
	{
		currentDriver->TransferTimedOut();
	}
#if TMC22xx_USES_SERCOM
	else if (dmaFinishedReason == DmaCallbackReason::complete)
#else
	else if (dmaFinished)
#endif
	{
		currentDriver->UartTmcHandler();
#if TMC22xx_SINGLE_DRIVER
		delay(2);						// TMC22xx can't handle back-to-back reads, so we need a short delay
#endif
		return true;
	}
#if TMC22xx_USES_SERCOM
	else if (dmaFinishedReason != DmaCallbackReason::none)
	{
		// DMA error, or DMA complete and DMA error
		currentDriver->DmaError();
#if TMC22xx_SINGLE_DRIVER
		delay(2);						// TMC22xx can't handle back-to-back reads, so we need a short delay
#endif
	}
#endif
	return false;
}

void NextDriver(size_t& currentDriverNumber) noexcept
{
#if !TMC22xx_SINGLE_DRIVER
			++currentDriverNumber;
			if (currentDriverNumber == GetNumTmcDrivers())
			{
				currentDriverNumber = 0;
			}
#endif
}

// This is the loop that the TMC task runs
extern "C" [[noreturn]] void TmcLoop(void *) noexcept
{
	size_t currentDriverNumber = 0;
	for (;;)
	{
		switch (driversState)
		{
		case DriversState::noPower:
		case DriversState::shutDown:
			currentDriverNumber = 0;
			TaskBase::Take();
			break;

		case DriversState::notInitialised:
			for (size_t drive = 0; drive < GetNumTmcDrivers(); ++drive)
			{
				driverStates[drive].WriteAll();
#if RESET_MICROSTEP_COUNTERS_AT_INIT
				driverStates[drive].SetMicrostepping256();			// switch to x256 microstepping until we can UpdateChopConfRegister
				driverStates[drive].ClearMicrostepPosition();
#endif
			}
			driversState = DriversState::initialising;
			break;

		case DriversState::initialising:
			// Do a transaction
			if (DoTransaction(currentDriverNumber))
			{
				// If all drivers that share the global enable have been initialised, move on to the next phase
				bool allInitialised = true;
				for (size_t i = 0; i < GetNumTmcDrivers(); ++i)
				{
#if TMC22xx_HAS_ENABLE_PINS
					if (driverStates[i].UsesGlobalEnable() && driverStates[i].UpdatePending())
#else
					if (driverStates[i].UpdatePending())
#endif
					{
						// Drivers 5-6 on the Duet 3 Mini are on the expansion board, which may not be present. So if they consistently time out, ignore them. Also allow for removed/blown drivers.
						if (driverStates[i].DriverAssumedPresent())
						{
							allInitialised = false;
							break;
						}
					}
				}

				if (allInitialised)
				{
#if RESET_MICROSTEP_COUNTERS_AT_INIT
					driversStepped.Clear();
					driversState = DriversState::stepping;
#else
					fastDigitalWriteLow(GlobalTmc22xxEnablePin);
					driversState = DriversState::ready;
#endif
				}
			}

			NextDriver(currentDriverNumber);
			break;

#if RESET_MICROSTEP_COUNTERS_AT_INIT
		case DriversState::stepping:
			if (DoTransaction(currentDriverNumber))
			{
				bool moreNeeded = false;
				for (size_t driver = 0; driver < GetNumTmcDrivers(); ++driver)
				{
					TmcDriverState& drv = driverStates[driver];
					if (drv.DriverAssumedPresent())
					{
						uint32_t count;
						if (drv.GetMicrostepPosition(count))
						{
							if (count != 0)
							{
								if (driversStepped.IsBitSet(driver))
								{
									// We have already stepped this driver, but it didn't adjust the microstep counter as we expected
									drv.RecordStepFailure();
debugPrintf("Driver %u still wrong\n", driver);
								}
								else
								{
									moreNeeded = true;						// we need to check the position again
									driversStepped.SetBit(driver);
									const bool backwards = (count > 512);
									reprap.GetPlatform().SetDriverAbsoluteDirection(driver, backwards);	// a high on DIR decreases the microstep counter
									if (backwards)
									{
										count = 1024 - count;
									}
debugPrintf("Sending %u steps to driver %u\n", (unsigned int)count, driver);
									do
									{
										delayMicroseconds(1);
										const uint32_t driverBitmap = StepPins::CalcDriverBitmap(driver);
										StepPins::StepDriversHigh(driverBitmap);
										delayMicroseconds(1);
										StepPins::StepDriversLow(driverBitmap);
										--count;
									} while (count != 0);
									drv.ClearMicrostepPosition();
									break;									// only do one driver at a time, to avoid using excessive CPU time
								}
							}
							else
							{
debugPrintf("Driver %u ok\n", driver);
							}
						}
						else
						{
							moreNeeded = true;								// we need to wait until the microstep position is read
						}
					}
				}

				if (!moreNeeded)
				{
					driversState = DriversState::reinitialising;
					for (size_t driver = 0; driver < GetNumTmcDrivers(); ++driver)
					{
						driverStates[driver].UpdateChopConfRegister();		// restore usual microstepping
					}
				}
			}
			NextDriver(currentDriverNumber);
			break;

		case DriversState::reinitialising:
			// If all drivers have been initialised, set the global enable
			if (DoTransaction(currentDriverNumber))
			{
				bool allInitialised = true;
				for (size_t i = 0; i < GetNumTmcDrivers(); ++i)
				{
# if TMC22xx_HAS_ENABLE_PINS
					if (driverStates[i].UsesGlobalEnable() && driverStates[i].UpdatePending())
# else
					if (driverStates[i].UpdatePending())
# endif
					{
						if (driverStates[i].DriverAssumedPresent())			// allow for missing/blown drivers
						{
							allInitialised = false;
							break;
						}
					}
				}

				if (allInitialised)
				{
					fastDigitalWriteLow(GlobalTmc22xxEnablePin);
					driversState = DriversState::ready;
				}
			}
			NextDriver(currentDriverNumber);
			break;
#endif

		case DriversState::ready:
			(void)DoTransaction(currentDriverNumber);
			NextDriver(currentDriverNumber);
			break;
		}
	}
}

//--------------------------- Public interface ---------------------------------

// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
#if TMC22xx_VARIABLE_NUM_DRIVERS
void SmartDrivers::Init(size_t numTmcDrivers) noexcept
#else
void SmartDrivers::Init() noexcept
#endif
{
#if TMC22xx_VARIABLE_NUM_DRIVERS
	numTmc22xxDrivers = min<size_t>(numTmcDrivers, MaxSmartDrivers);
#endif

	// Make sure the ENN pins are high
	IoPort::SetPinMode(GlobalTmc22xxEnablePin, OUTPUT_HIGH);

#if TMC22xx_SINGLE_UART
	// Set up the single UART that communicates with all TMC22xx drivers
# if TMC22xx_USES_SERCOM
	SetPinFunction(TMC22xxSercomTxPin, TMC22xxSercomTxPinPeriphMode);
	SetPinFunction(TMC22xxSercomRxPin, TMC22xxSercomRxPinPeriphMode);

	Serial::InitUart(TMC22xxSercomNumber, DriversBaudRate, TMC22xxSercomRxPad, true);
	DmacManager::SetInterruptCallback(DmacChanTmcRx, TransferCompleteCallback, CallbackParameter(0));
# else
	SetPinFunction(TMC22xxUartTxPin, TMC22xxUartPeriphMode);
	SetPinFunction(TMC22xxUartRxPin, TMC22xxUartPeriphMode);
	EnablePullup(TMC22xxUartRxPin);

	// Enable the clock to the UART
	pmc_enable_periph_clk(ID_TMC22xx_UART);

	// Set the UART baud rate, 8 bits, 2 stop bits, no parity
	UART_TMC22xx->UART_IDR = ~0u;
	UART_TMC22xx->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
	UART_TMC22xx->UART_MR = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO;
	UART_TMC22xx->UART_BRGR = SystemPeripheralClock()/(16 * DriversBaudRate);		// set baud rate
	UART_TMC22xx->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;

	NVIC_EnableIRQ(TMC22xx_UART_IRQn);
# endif
#endif

#if TMC22xx_HAS_MUX
	// Set up the multiplexer control pins as outputs
	for (Pin p : TMC22xxMuxPins)
	{
		IoPort::SetPinMode(p, OUTPUT_LOW);
	}

# if TMC22xx_USE_SLAVEADDR
	currentMuxState = false;
# endif
#endif

	driversState = DriversState::noPower;
	for (size_t drive = 0; drive < GetNumTmcDrivers(); ++drive)
	{
#if !TMC22xx_SINGLE_UART
		// Set up the individual UARTs that communicate with each of the TMC22xx drivers
# if TMC22xx_USES_SERCOM
		// Initialise the SERCOM that controls this driver
		gpio_set_pin_function(TMC22xxSercomTxPins[drive], TMC22xxSercomTxPinPeriphModes[drive]);
		gpio_set_pin_function(TMC22xxSercomRxPins[drive], TMC22xxSercomRxPinPeriphModes[drive]);

		Serial::InitUart(TMC22xxUarts[drive], TMC22xxSercomNumbers[drive], DriversBaudRate);
		NVIC_EnableIRQ(TMC22xxSercomIRQns[drive]);
# else
		// Initialise the UART that controls this driver. The pins are already set up for UART use in the pins table
		ConfigurePin(TMC22xxUartPins[drive]);

		// Enable the clock to the UART
		pmc_enable_periph_clk(TMC22xxUartIds[drive]);

		// Set the UART baud rate, 8 bits, 2 stop bits, no parity
		Uart * const uart = TMC22xxUarts[drive];
		uart->UART_IDR = ~0u;
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;
		uart->UART_MR = UART_MR_CHMODE_NORMAL | UART_MR_PAR_NO;
		uart->UART_BRGR = VARIANT_MCK/(16 * DriversBaudRate);		// set baud rate
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS | UART_CR_RSTSTA;
		NVIC_EnableIRQ(TMC22xxUartIRQns[drive]);
# endif
#endif
		driverStates[drive].Init(drive
#if TMC22xx_HAS_ENABLE_PINS
								, ENABLE_PINS[drive]
#endif
#if HAS_STALL_DETECT
								, DriverDiagPins[drive]
#endif
								);
	}

#if HAS_STALL_DETECT
	InitStallDetectionLogic();
#endif

	driversState = DriversState::noPower;
	tmcTask.Create(TmcLoop, "TMC", nullptr, TaskPriority::TmcPriority);
}

// Shut down the drivers and stop any related interrupts
void SmartDrivers::Exit() noexcept
{
	IoPort::SetPinMode(GlobalTmc22xxEnablePin, OUTPUT_HIGH);
#if TMC22xx_SINGLE_UART
# if TMC22xx_USES_SERCOM
	DmacManager::SetInterruptCallback(DmacChanTmcRx, nullptr, CallbackParameter(nullptr));
# else
	NVIC_DisableIRQ(TMC22xx_UART_IRQn);
# endif
#else
	for (size_t drive = 0; drive < numTmc22xxDrivers; ++drive)
	{
		NVIC_DisableIRQ(TMC22xxUartIRQns[drive]);
	}
#endif
	tmcTask.TerminateAndUnlink();
	driversState = DriversState::shutDown;				// prevent Spin() calls from doing anything
}

void SmartDrivers::SetAxisNumber(size_t drive, uint32_t axisNumber) noexcept
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].SetAxisNumber(axisNumber);
	}
}

uint32_t SmartDrivers::GetAxisNumber(size_t drive) noexcept
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].GetAxisNumber() : 0;
}

void SmartDrivers::SetCurrent(size_t drive, float current) noexcept
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].SetCurrent(current);
	}
}

void SmartDrivers::EnableDrive(size_t drive, bool en) noexcept
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].Enable(en);
	}
}

// Set microstepping or chopper control register
bool SmartDrivers::SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolate) noexcept
{
	if (drive < GetNumTmcDrivers() && microsteps > 0)
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
			driverStates[drive].SetMicrostepping(shift, interpolate);
			return true;
		}
	}
	return false;
}

// Get microstepping or chopper control register
unsigned int SmartDrivers::GetMicrostepping(size_t drive, bool& interpolation) noexcept
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].GetMicrostepping(interpolation) : 1;
}

bool SmartDrivers::SetDriverMode(size_t driver, unsigned int mode) noexcept
{
	return driver < GetNumTmcDrivers() && driverStates[driver].SetDriverMode(mode);
}

DriverMode SmartDrivers::GetDriverMode(size_t driver) noexcept
{
	return (driver < GetNumTmcDrivers()) ? driverStates[driver].GetDriverMode() : DriverMode::unknown;
}

// Flag that the the drivers have been powered up or down
void SmartDrivers::Spin(bool powered) noexcept
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
	else if (driversState != DriversState::shutDown)
	{
		driversState = DriversState::noPower;				// flag that there is no power to the drivers
		fastDigitalWriteHigh(GlobalTmc22xxEnablePin);		// disable the drivers
	}
}

// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
void SmartDrivers::TurnDriversOff() noexcept
{
	// When using TMC2660 drivers, this is called when an over-voltage event occurs, so that we can try to protect the drivers by disabling them.
	// We don't use it with TMC22xx drivers.
}

void SmartDrivers::SetStallThreshold(size_t driver, int sgThreshold) noexcept
{
#if HAS_STALL_DETECT
	if (driver < GetNumTmcDrivers())
	{
		driverStates[driver].SetStallDetectThreshold(sgThreshold);
	}
#endif
}

void SmartDrivers::SetStallFilter(size_t driver, bool sgFilter) noexcept
{
	// Not a supported on TMC2209
}

void SmartDrivers::SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept
{
#if HAS_STALL_DETECT
	if (driver < GetNumTmcDrivers())
	{
		driverStates[driver].SetStallMinimumStepsPerSecond(stepsPerSecond);
	}
#endif
}

void SmartDrivers::AppendStallConfig(size_t driver, const StringRef& reply) noexcept
{
#if HAS_STALL_DETECT
	if (driver < GetNumTmcDrivers())
	{
		driverStates[driver].AppendStallConfig(reply);
	}
	else
	{
		reply.cat("no such driver");
	}
#endif
}

void SmartDrivers::AppendDriverStatus(size_t drive, const StringRef& reply) noexcept
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].AppendDriverStatus(reply);
	}
}

float SmartDrivers::GetStandstillCurrentPercent(size_t drive) noexcept
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].GetStandstillCurrentPercent() : 0.0;
}

void SmartDrivers::SetStandstillCurrentPercent(size_t drive, float percent) noexcept
{
	if (drive < GetNumTmcDrivers())
	{
		driverStates[drive].SetStandstillCurrentPercent(percent);
	}
}

bool SmartDrivers::SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept
{
	return (driver < GetNumTmcDrivers()) && driverStates[driver].SetRegister(reg, regVal);
}

uint32_t SmartDrivers::GetRegister(size_t driver, SmartDriverRegister reg) noexcept
{
	return (driver < GetNumTmcDrivers()) ? driverStates[driver].GetRegister(reg) : 0;
}

// Read any register from a driver
// This will return GCodeResult:notFinished for at least the first call, so it must be called repeatedly until it returns a different value.
GCodeResult SmartDrivers::GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept
{
	if (driver < GetNumTmcDrivers())
	{
		return driverStates[driver].GetAnyRegister(reply, regNum);
	}
	reply.copy("Invalid smart driver number");
	return GCodeResult::error;
}

GCodeResult SmartDrivers::SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept
{
	if (driver < GetNumTmcDrivers())
	{
		return driverStates[driver].SetAnyRegister(reply, regNum, regVal);
	}
	reply.copy("Invalid smart driver number");
	return GCodeResult::error;
}

StandardDriverStatus SmartDrivers::GetStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept
{
	if (driver < GetNumTmcDrivers())
	{
		return driverStates[driver].GetStatus(accumulated, clearAccumulated);
	}
	StandardDriverStatus rslt;
	rslt.all = 0;
	return rslt;
}

#if HAS_STALL_DETECT

# ifdef DUET3MINI

// Stall detection for Duet 3 Mini v0.4 and later
// Each TMC2209 DIAG output is routed to its own MCU pin, however we don't have enough EXINTs on the SAME54 to give each one its own interrupt.
// So we route them all to CCL input pins instead, which lets us selectively OR them together in 3 groups and generate an interrupt from the resulting events

// Set up to generate interrupts on the specified drivers stalling
void EnableStallInterrupt(DriversBitmap drivers) noexcept
{
	// Disable all the Diag event interrupts
	for (unsigned int i = 1; i < 3; ++i)
	{
		CCL->LUTCTRL[i].reg &= ~CCL_LUTCTRL_ENABLE;
		EVSYS->Channel[CclLut0Event + i].CHINTENCLR.reg = EVSYS_CHINTENCLR_EVD | EVSYS_CHINTENCLR_OVR;
		EVSYS->Channel[CclLut0Event + i].CHINTFLAG.reg = EVSYS_CHINTFLAG_EVD | EVSYS_CHINTENCLR_OVR;
	}

	if (!drivers.IsEmpty())
	{
		// Calculate the new LUT control values
		constexpr uint32_t lutDefault = CCL_LUTCTRL_TRUTH(0xFE) | CCL_LUTCTRL_LUTEO;		// OR function, disabled, event output enabled
		uint32_t lutInputControls[4] = { lutDefault, lutDefault, lutDefault, lutDefault };
		drivers.IterateWhile([&lutInputControls](unsigned int driver, unsigned int count) -> bool
			{ 	if (driver < GetNumTmcDrivers())
				{
					const uint32_t cclInput = CclDiagInputs[driver];
					lutInputControls[cclInput & 3] |= cclInput & 0x000000FFFFFF0000;
					return true;
				}
				else
				{
					return false;
				}
			} );

		// Now set up the CCL with those inputs. We only use CCL 1-3 so leave 0 alone for possible other applications.
		for (unsigned int i = 1; i < 4; ++i)
		{
			if (lutInputControls[i] & 0x000000FFFFFF0000)		// if any inputs are enabled
			{
				CCL->LUTCTRL[i].reg = lutInputControls[i];
				CCL->LUTCTRL[i].reg = lutInputControls[i] | CCL_LUTCTRL_ENABLE;
				EVSYS->Channel[CclLut0Event + i].CHINTENSET.reg = EVSYS_CHINTENCLR_EVD;
			}
		}
	}
}

// Initialise the stall detection logic that is external to the drivers. Only needs to be called once.
static void InitStallDetectionLogic() noexcept
{
	// Set up the DIAG inputs as CCL inputs
	for (Pin p : DriverDiagPins)
	{
		pinMode(p, INPUT_PULLDOWN);								// enable pulldown in case of missing drivers
		SetPinFunction(p, GpioPinFunction::N);
	}

	// Set up the event channels for CCL LUTs 1 to 3. We only use CCL 1-3 so leave 0 alone for possible other applications.
	for (unsigned int i = 1; i < 4; ++i)
	{
		GCLK->PCHCTRL[EVSYS_GCLK_ID_0 + i].reg = GCLK_PCHCTRL_GEN(GclkNum60MHz) | GCLK_PCHCTRL_CHEN;	// enable the GCLK, needed to use the resynchronised path
		EVSYS->Channel[CclLut0Event + i].CHANNEL.reg = EVSYS_CHANNEL_EVGEN(0x74 + i) | EVSYS_CHANNEL_PATH_RESYNCHRONIZED;
																// LUT output events on the SAME5x are event generator numbers 0x74 to 0x77. Resynchronised path allows interrupts.
		EVSYS->Channel[CclLut0Event + i].CHINTENCLR.reg = EVSYS_CHINTENCLR_EVD | EVSYS_CHINTENCLR_OVR;	// disable interrupts for now
		NVIC_SetPriority((IRQn)(EVSYS_0_IRQn + i), NvicPriorityDriverDiag);
		NVIC_EnableIRQ((IRQn)(EVSYS_0_IRQn + i));				// enable the interrupt for this event channel in the NVIC
	}

	CCL->CTRL.reg = CCL_CTRL_ENABLE;							// enable the CCL
}

# endif

DriversBitmap SmartDrivers::GetStalledDrivers(DriversBitmap driversOfInterest) noexcept
{
	DriversBitmap rslt;
	driversOfInterest.Iterate([&rslt](unsigned int driverNumber, unsigned int count)
								{
									if (driverNumber < ARRAY_SIZE(DriverDiagPins) && digitalRead(DriverDiagPins[driverNumber]))
									{
										rslt.SetBit(driverNumber);
									}
								}
							 );
	return rslt;
}

#endif

#endif

// End
