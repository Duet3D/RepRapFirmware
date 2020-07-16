/*
 * TMC22xx.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 */

#include "RepRapFirmware.h"

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

#include "TMC22xx.h"
#include <RepRap.h>
#include <TaskPriorities.h>
#include <Movement/Move.h>
#include <Movement/StepTimer.h>
#include <Hardware/Cache.h>

#if SAME5x
# include <Hardware/IoPorts.h>
# include <DmacManager.h>
# include <Serial.h>
# include <component/sercom.h>
#else
# include <sam/drivers/pdc/pdc.h>
# include <sam/drivers/uart/uart.h>
#endif

// Important note:
// The TMC2224 does handle a write request immediately followed by a read request.
// The TMC2224 does _not_ handle back-to-back read requests, it needs a short delay between them.

constexpr float MinimumOpenLoadMotorCurrent = 500;			// minimum current in mA for the open load status to be taken seriously
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
constexpr size_t TmcTaskStackWords = 100;

#ifdef DUET_5LC
constexpr uint16_t DriverNotPresentTimeouts = 20;
#endif

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

enum class DriversState : uint8_t
{
	noPower = 0,
	notInitialised,
	initialising,
	ready
};

static DriversState driversState = DriversState::noPower;

#if TMC22xx_USE_SLAVEADDR && !defined(DUET_5LC)
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

constexpr uint32_t DefaultGConfReg = GCONF_UART | GCONF_MSTEP_REG | GCONF_MULTISTEP_FILT;

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

constexpr uint32_t DefaultChopConfReg = 0x10000053 | CHOPCONF_VSENSE_HIGH;	// this is the reset default + CHOPCONF_VSENSE_HIGH - try it until we find something better

// DRV_STATUS register. See the .h file for the bit definitions.
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;

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

#if 1

// Fast table-driven CRC-8. Unfortunately, the result needs ot be reflected.
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
static inline constexpr uint8_t FastCRCAddByte(uint8_t crc, uint8_t currentByte) noexcept
{
	return crc_table[crc ^ currentByte];
}

// Reverse the order of the bits
static inline constexpr uint8_t Reflect(uint8_t b) noexcept
{
#if 1
	b = (b & 0b11110000) >> 4 | (b & 0b00001111) << 4;
	b = (b & 0b11001100) >> 2 | (b & 0b00110011) << 2;
	b = (b & 0b10101010) >> 1 | (b & 0b01010101) << 1;
	return b;
#else
	uint32_t temp = b;
	asm("rbit %1,%0" : "=r" (temp) : "r" (temp));
	return temp >> 24;
#endif
}
#endif

// Add 1 bit to a CRC
static inline constexpr uint8_t CRCAddBit(uint8_t crc, uint8_t currentByte, uint8_t bit) noexcept
{
	return (((crc ^ (currentByte << (7 - bit))) & 0x80) != 0)
			? (crc << 1) ^ 0x07
				: (crc << 1);
}

// Add a byte to a CRC
static inline constexpr uint8_t CRCAddByte(uint8_t crc, uint8_t currentByte) noexcept
{
	crc = CRCAddBit(crc, currentByte, 0);
	crc = CRCAddBit(crc, currentByte, 1);
	crc = CRCAddBit(crc, currentByte, 2);
	crc = CRCAddBit(crc, currentByte, 3);
	crc = CRCAddBit(crc, currentByte, 4);
	crc = CRCAddBit(crc, currentByte, 5);
	crc = CRCAddBit(crc, currentByte, 6);
	crc = CRCAddBit(crc, currentByte, 7);
	return crc;
}

//#endif

static_assert(Reflect(FastCRCAddByte(FastCRCAddByte(FastCRCAddByte(0, 1), 2), 3)) == 0x1E);
static_assert(CRCAddByte(CRCAddByte(CRCAddByte(0, 1), 2), 3) == 0x1E);

// CRC of the first byte we send in any request
static constexpr uint8_t InitialByteCRC = CRCAddByte(0, 0x05);

#if !TMC22xx_USE_SLAVEADDR

// CRC of the first 2 bytes we send in any request
static constexpr uint8_t InitialSendCRC = CRCAddByte(InitialByteCRC, 0x00);

// CRC of a request to read the IFCOUNT register
static constexpr uint8_t ReadIfcountCRC = CRCAddByte(InitialSendCRC, REGNUM_IFCOUNT);

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
	uint8_t GetDriverNumber() const noexcept { return driverNumber; }
	bool UpdatePending() const noexcept;
#if TMC22xx_HAS_ENABLE_PINS
	bool UsesGlobalEnable() const noexcept { return enablePin == NoPin; }
#endif

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(SmartDriverRegister reg) const noexcept;

	float GetStandstillCurrentPercent() const noexcept;
	void SetStandstillCurrentPercent(float percent) noexcept;

#ifdef DUET_5LC
	bool DriverAssumedPresent() const noexcept { return numWrites != 0 || numTimeouts < DriverNotPresentTimeouts; }
#endif

	void TransferDone() noexcept __attribute__ ((hot));		// called by the ISR when the SPI transfer has completed
	void StartTransfer() noexcept __attribute__ ((hot));	// called to start a transfer
	void TransferTimedOut() noexcept
	{
#ifdef DUET_5LC
		if (DriverAssumedPresent())
		{
			++numTimeouts;
		}
#else
		++numTimeouts;
#endif
		AbortTransfer();
	}

	void DmaError() noexcept { ++numDmaErrors; AbortTransfer(); }
	void AbortTransfer() noexcept;

	uint32_t ReadLiveStatus() const noexcept;
	uint32_t ReadAccumulatedStatus(uint32_t bitsToKeep) noexcept;

	// Variables used by the ISR
	static uint32_t transferStartedTime;

	void UartTmcHandler() noexcept;							// core of the ISR for this driver

private:
	bool SetChopConf(uint32_t newVal) noexcept;
	void UpdateRegister(size_t regIndex, uint32_t regVal) noexcept;
	void UpdateChopConfRegister() noexcept;					// calculate the chopper control register and flag it for sending
	void UpdateCurrent() noexcept;
	void UpdateMaxOpenLoadStepInterval() noexcept;
#if HAS_STALL_DETECT
	bool IsTmc2209() const noexcept { return (readRegisters[ReadIoIn] & IOIN_VERSION_MASK) == (IOIN_VERSION_2209 << IOIN_VERSION_SHIFT); }
	void ResetLoadRegisters() noexcept
	{
		minSgLoadRegister = 1023;
		maxSgLoadRegister = 0;
	}
#endif

#if TMC22xx_HAS_MUX
	void SetUartMux() noexcept;
#endif
#if (TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER) && !TMC22xx_USE_SLAVEADDR
	static void SetupDMASend(uint8_t regnum, uint32_t outVal) noexcept __attribute__ ((hot));	// set up the PDC or DMAC to send a register
	static void SetupDMAReceive(uint8_t regnum) noexcept __attribute__ ((hot));					// set up the PDC or DMAC to receive a register
#else
	void SetupDMASend(uint8_t regnum, uint32_t outVal) noexcept __attribute__ ((hot));			// set up the PDC or DMAC to send a register
	void SetupDMAReceive(uint8_t regnum) noexcept __attribute__ ((hot));						// set up the PDC or DMAC to receive a register
#endif

#if HAS_STALL_DETECT
	static constexpr unsigned int NumWriteRegisters = 9;		// the number of registers that we write to on a TMC2209
	static constexpr unsigned int NumWriteRegistersNon09 = 6;	// the number of registers that we write to on a TMC2208/2224
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

#if HAS_STALL_DETECT
	static constexpr unsigned int NumReadRegisters = 7;			// the number of registers that we read from on a TMC2209
	static constexpr unsigned int NumReadRegistersNon09 = 6;	// the number of registers that we read from on a TMC2208/2224
#else
	static constexpr unsigned int NumReadRegisters = 6;			// the number of registers that we read from on a TMC2208/2224
#endif
	static const uint8_t ReadRegNumbers[NumReadRegisters];		// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadIoIn = 0;				// includes the version which we use to distinguish TMC2209 from 2208/2224
	static constexpr unsigned int ReadGStat = 1;			// global status
	static constexpr unsigned int ReadDrvStat = 2;			// drive status
	static constexpr unsigned int ReadMsCnt = 3;			// microstep counter
	static constexpr unsigned int ReadPwmScale = 4;			// PWM scaling
	static constexpr unsigned int ReadPwmAuto = 5;			// PWM scaling
#if HAS_STALL_DETECT
	static constexpr unsigned int ReadSgResult = 6;			// stallguard result, TMC2209 only
#endif

	volatile uint32_t writeRegisters[NumWriteRegisters];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters];		// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedReadRegisters[NumReadRegisters];

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	volatile uint32_t registersToUpdate;					// bitmap of register indices whose values need to be sent to the driver chip

	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	float motorCurrent;										// the configured motor current
	uint32_t maxOpenLoadStepInterval;						// the maximum step pulse interval for which we consider open load detection to be reliable

#if HAS_STALL_DETECT
	uint32_t minSgLoadRegister;								// the minimum value of the StallGuard bits we read
	uint32_t maxSgLoadRegister;								// the maximum value of the StallGuard bits we read
#endif

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
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
	uint8_t readIfCountCRC;
#endif
	bool enabled;											// true if driver is enabled
};

// Static data members of class TmcDriverState

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
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
alignas(4) volatile uint8_t TmcDriverState::receiveData[20];

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
	return registersToUpdate != 0
#if HAS_STALL_DETECT
		&& (IsTmc2209() || LowestSetBit(registersToUpdate) < NumWriteRegistersNon09)
#endif
		;
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

	sendData[SendDataCRCIndex0] = crc;

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

// Set up the PDC or DMAC to send a register and receive the status
inline void TmcDriverState::SetupDMAReceive(uint8_t regNum) noexcept
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
	uint8_t crc = InitialSendCRC;
#endif
	sendData[2] = regNum;
	sendData[3] = CRCAddByte(crc, regNum);

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
	const uint32_t defaultMaxInterval = StepTimer::StepClockRate/MinimumOpenLoadFullStepsPerSec;
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
		constexpr uint32_t conversionFactor = ((256 - 51) * (StepTimer::StepClockRate/1000000))/12;
		const uint32_t fullStepClocks = tpwmthrs * conversionFactor;
		maxOpenLoadStepInterval = min<uint32_t>(fullStepClocks, defaultMaxInterval);
	}
}

// Set a register value and flag it for updating
void TmcDriverState::UpdateRegister(size_t regIndex, uint32_t regVal) noexcept
{
	writeRegisters[regIndex] = regVal;
	registersToUpdate |= (1u << regIndex);								// flag it for sending
	if (regIndex == WriteGConf || regIndex == WriteTpwmthrs)
	{
		UpdateMaxOpenLoadStepInterval();
	}
}

// Calculate the chopper control register and flag it for sending
void TmcDriverState::UpdateChopConfRegister() noexcept
{
	UpdateRegister(WriteChopConf, (enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK);
}

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
	IoPort::SetPinMode(p_diagPin, INPUT_PULLUP);
#endif

#if !(TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER)
# if TMC22xx_USES_SERCOM
	sercom = TMC22xxSercoms[p_driverNumber];
	sercomNumber = TMC22xxSercomNumbers[p_driverNumber];
# else
	uart = TMC22xxUarts[p_driverNumber];
# endif
#endif

#if TMC22xx_USE_SLAVEADDR
	initialSendCRC = CRCAddByte(InitialByteCRC, driverNumber & 3u);		// CRC of the first 2 bytes of any transmission
	readIfCountCRC = CRCAddByte(initialSendCRC, REGNUM_IFCOUNT);
#endif
	enabled = false;
	registersToUpdate = 0;
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
	regnumBeingUpdated = 0xFF;
	failedOp = 0xFF;
	registerToRead = 0;
	lastIfCount = 0;
	readErrors = writeErrors = numReads = numWrites = numTimeouts = numDmaErrors = 0;
#if HAS_STALL_DETECT
	ResetLoadRegisters();
#endif
}

#if HAS_STALL_DETECT

void TmcDriverState::SetStallDetectThreshold(int sgThreshold) noexcept
{
	const uint32_t sgthrs = (uint32_t)(constrain<int>(sgThreshold, -64, 63) + 64);
	UpdateRegister(WriteSgthrs, sgthrs);
}

void TmcDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept
{
	UpdateRegister(WriteTcoolthrs, (12000000 + (128 * stepsPerSecond))/(256 * stepsPerSecond));
}

void TmcDriverState::AppendStallConfig(const StringRef& reply) const noexcept
{
	const int threshold = (int)(writeRegisters[WriteSgthrs] - 64);
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
					(writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRunCsBits << IHOLDIRUN_IRUN_SHIFT) | (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
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

// Read the status
uint32_t TmcDriverState::ReadLiveStatus() const noexcept
{
	uint32_t ret = readRegisters[ReadDrvStat] & (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST | TMC_RR_TEMPBITS);
	if (!enabled)
	{
		ret &= ~(TMC_RR_OLA | TMC_RR_OLB);
	}
#if HAS_STALL_DETECT
	if (IoPort::ReadPin(diagPin))
	{
		ret |= TMC_RR_SG;
	}
#endif
	return ret;
}

// Read the status
uint32_t TmcDriverState::ReadAccumulatedStatus(uint32_t bitsToKeep) noexcept
{
	const uint32_t mask = (enabled) ? 0xFFFFFFFF : ~(TMC_RR_OLA | TMC_RR_OLB);
	bitsToKeep &= mask;
	const irqflags_t flags = cpu_irq_save();
	uint32_t status = accumulatedReadRegisters[ReadDrvStat];
	accumulatedReadRegisters[ReadDrvStat] = (status & bitsToKeep) | readRegisters[ReadDrvStat];		// so that the next call to ReadAccumulatedStatus isn't missing some bits
	cpu_irq_restore(flags);
	status &= (TMC_RR_OT | TMC_RR_OTPW | TMC_RR_S2G | TMC_RR_OLA | TMC_RR_OLB | TMC_RR_STST | TMC_RR_TEMPBITS) & mask;
#if HAS_STALL_DETECT
	if (IoPort::ReadPin(diagPin))
	{
		status |= TMC_RR_SG;
	}
#endif
	return status;
}

// Append the driver status to a string, and reset the min/max load values
void TmcDriverState::AppendDriverStatus(const StringRef& reply) noexcept
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
	if (lastReadStatus & TMC_RR_OLA)
	{
		reply.cat(" open-load-A");
	}
	if (lastReadStatus & TMC_RR_OLB)
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

#if HAS_STALL_DETECT
	if (minSgLoadRegister <= maxSgLoadRegister)
	{
		reply.catf(", SG min/max %" PRIu32 "/%" PRIu32, minSgLoadRegister, maxSgLoadRegister);
	}
	else
	{
		reply.cat(", SG min/max not available");
	}
	ResetLoadRegisters();
#endif

	reply.catf(", read errors %u, write errors %u, ifcnt %u, reads %u, writes %u, timeouts %u, DMA errors %u",
					readErrors, writeErrors, lastIfCount, numReads, numWrites, numTimeouts, numDmaErrors);
	if (failedOp != 0xFF)
	{
		reply.catf(", failedOp 0x%02x", failedOp);
		failedOp = 0xFF;
	}
#ifdef DUET_5LC
	if (!DriverAssumedPresent())
	{
		reply.cat(", assumed not present");
	}
#endif
	readErrors = writeErrors = numReads = numWrites = numTimeouts = numDmaErrors = 0;
}

// This is called by the ISR when the SPI transfer has completed
inline void TmcDriverState::TransferDone() noexcept
{
	Cache::InvalidateAfterDMAReceive(receiveData, sizeof(receiveData));
	if (sendData[2] & 0x80)								// if we were writing a register
	{
		const uint8_t currentIfCount = receiveData[18];
		// Note, the TMC2209 IFCNT register seems to start at a random value, so we expect to get a write error on the first write.
		// We could read IFCNT once to get the initial value, but doing the first write once does no harm.
		if (regnumBeingUpdated < NumWriteRegisters && currentIfCount == (uint8_t)(lastIfCount + 1) && (sendData[2] & 0x7F) == WriteRegNumbers[regnumBeingUpdated])
		{
			registersToUpdate &= ~(1u << regnumBeingUpdated);
			++numWrites;
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
		if (sendData[2] == ReadRegNumbers[registerToRead] && ReadRegNumbers[registerToRead] == receiveData[6] && receiveData[4] == 0x05 && receiveData[5] == 0xFF)
		{
			// We asked to read the scheduled read register, and the sync byte, slave address and register number in the received message match
			//TODO here we could check the CRC of the received message, but for now we assume that we won't get any corruption in the 32-bit received data
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
#if HAS_STALL_DETECT
			else if (registerToRead == ReadSgResult)
			{
				const uint32_t sgResult = regVal & SG_RESULT_MASK;
				if (sgResult < minSgLoadRegister)
				{
					minSgLoadRegister = sgResult;
				}
				if (sgResult > maxSgLoadRegister)
				{
					maxSgLoadRegister = sgResult;
				}
			}
#endif
			readRegisters[registerToRead] = regVal;
			accumulatedReadRegisters[registerToRead] |= regVal;

			++registerToRead;
			if (   registerToRead >= NumReadRegisters
#if HAS_STALL_DETECT
				|| (registerToRead >= NumReadRegistersNon09 && !IsTmc2209())
#endif
			   )
			{
				registerToRead = 0;
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
#if TMC22xx_USE_SLAVEADDR
# if defined(DUET_5LC)
	// Duet 5LC has a 1-bit mux to select between 2 banks of 4 drivers. High selects the first bank.
	// We have changed TmcLoop to address drivers in alternate banks, so we don't need to insert any delays here
	digitalWrite(TMC22xxMuxPins[0], (driverNumber & 0x04) == 0);
# else
	const bool newMuxState = ((driverNumber & 0x04) != 0);
	if (newMuxState == currentMuxState)
	{
		// A TMC2209 turns off its transmitter 4 bit times after the end of the last byte.
		// So if we didn't change the mux, we need a delay here.
		// In fact, even 8 bit times isn't enough delay.
		delay(2);
	}
	else
	{
		digitalWrite(TMC22xxMuxPins[0], newMuxState);
	}
	currentMuxState = newMuxState;
# endif
#else
	digitalWrite(TMC22xxMuxPins[0], (driverNumber & 0x01) != 0);
	digitalWrite(TMC22xxMuxPins[1], (driverNumber & 0x02) != 0);
	digitalWrite(TMC22xxMuxPins[2], (driverNumber & 0x04) != 0);
#endif
}

#endif

// This is called from the ISR or elsewhere to start a new SPI transfer. Inlined for ISR speed.
inline void TmcDriverState::StartTransfer() noexcept
{
#if TMC22xx_HAS_MUX
	SetUartMux();
#endif

	// Find which register to send. The common case is when no registers need to be updated.
#if HAS_STALL_DETECT
	size_t regNum;
	if (registersToUpdate != 0 && ((regNum = LowestSetBit(registersToUpdate)) < NumWriteRegistersNon09 || IsTmc2209()))
	{
		// Write a register
#else
	if (registersToUpdate != 0)
	{
		const size_t regNum = LowestSetBit(registersToUpdate);
#endif

		// Kick off a transfer for the register to write
		const irqflags_t flags = cpu_irq_save();		// avoid race condition
		regnumBeingUpdated = regNum;
#if TMC22xx_USES_SERCOM
		sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// disable transmitter and receiver, reset receiver
		while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;	// reset transmitter and receiver
#endif
		SetupDMASend(WriteRegNumbers[regNum], writeRegisters[regNum]);	// set up the PDC
#if TMC22xx_USES_SERCOM
		dmaFinishedReason = DmaCallbackReason::none;
		DmacManager::EnableCompletedInterrupt(DmacChanTmcRx);
		sercom->USART.CTRLB.reg |= (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// enable transmitter and receiver
#else
		uart->UART_IER = UART_IER_ENDRX;				// enable end-of-transfer interrupt
		uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;	// enable transmitter and receiver
#endif
		cpu_irq_restore(flags);
	}
	else
	{
		// Read a register
		regnumBeingUpdated = 0xFF;
		const irqflags_t flags = cpu_irq_save();		// avoid race condition
#if TMC22xx_USES_SERCOM
		sercom->USART.CTRLB.reg &= ~(SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// disable transmitter and receiver, reset receiver
		while (sercom->USART.SYNCBUSY.bit.CTRLB) { }
#else
		uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX;	// reset transmitter and receiver
#endif
		SetupDMAReceive(ReadRegNumbers[registerToRead]);	// set up the PDC
#if TMC22xx_USES_SERCOM
		dmaFinishedReason = DmaCallbackReason::none;
		DmacManager::EnableCompletedInterrupt(DmacChanTmcRx);
		sercom->USART.CTRLB.reg |= (SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN);	// enable transmitter and receiver
#else
		uart->UART_IER = UART_IER_ENDRX;				// enable end-of-receive interrupt
		uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;	// enable transmitter and receiver
#endif
		cpu_irq_restore(flags);
	}
}

// ISR(s) for the UART(s)

inline void TmcDriverState::UartTmcHandler() noexcept
{
#if !(TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER)
# if TMC22xx_USES_SERCOM
	DmacManager::DisableCompletedInterrupt(DmacChanTmcRx);
# else
	uart->UART_IDR = UART_IDR_ENDRX;					// disable the PDC interrupt
# endif
#endif
	TransferDone();										// tidy up after the transfer we just completed
}

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER

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
extern "C" void TMC22xx_UART_Handler() noexcept __attribute__ ((hot));

void TMC22xx_UART_Handler() noexcept
{
	UART_TMC22xx->UART_IDR = UART_IDR_ENDRX;			// disable the interrupt
	dmaFinished = true;
	tmcTask.GiveFromISR();
}

# endif

#else

// ISRs for the individual UARTs
extern "C" void UART_TMC_DRV0_Handler() noexcept __attribute__ ((hot));
void UART_TMC_DRV0_Handler() noexcept
{
	driverStates[0].UartTmcHandler();
}

extern "C" void UART_TMC_DRV1_Handler() noexcept __attribute__ ((hot));
void UART_TMC_DRV1_Handler() noexcept
{
	driverStates[1].UartTmcHandler();
}

#endif

extern "C" void TmcLoop(void *) noexcept
{
	TmcDriverState * currentDriver = nullptr;
#ifdef DUET_5LC
	size_t currentDriverNumber;
#endif
	for (;;)
	{
		if (driversState == DriversState::noPower)
		{
			currentDriver = nullptr;
			TaskBase::Take();
		}
		else
		{
			if (driversState == DriversState::notInitialised)
			{
				for (size_t drive = 0; drive < GetNumTmcDrivers(); ++drive)
				{
					driverStates[drive].WriteAll();
				}
				driversState = DriversState::initialising;
			}

			// Do a transaction
#if TMC22xx_SINGLE_DRIVER
			currentDriver = driverStates;
#elif defined(DUET_5LC)
			// To avoid having to insert delays between addressing drivers on the same multiplexer channel,
			// address drivers on alternate multiplexer channels, i.e in the order 04152637
			if (currentDriver == nullptr || currentDriverNumber == GetNumTmcDrivers() - 1)
			{
				currentDriverNumber = 0;
			}
			else
			{
				++currentDriverNumber;
			}
			static constexpr TmcDriverState *drivers[] =
			{
				&driverStates[0], &driverStates[4], &driverStates[1], &driverStates[5], &driverStates[2], &driverStates[6], &driverStates[3], &driverStates[7]
			};
			static_assert(ARRAY_SIZE(drivers) == MaxSmartDrivers);
			currentDriver = drivers[currentDriverNumber];
#else
			currentDriver = (currentDriver == nullptr || currentDriver + 1 == driverStates + GetNumTmcDrivers())
								? driverStates
									: currentDriver + 1;
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

				if (driversState == DriversState::initialising)
				{
					// If all drivers that share the global enable have been initialised, set the global enable
					bool allInitialised = true;
					for (size_t i = 0; i < GetNumTmcDrivers(); ++i)
					{
#if TMC22xx_HAS_ENABLE_PINS
						if (driverStates[i].UsesGlobalEnable() && driverStates[i].UpdatePending())
#else
						if (driverStates[i].UpdatePending())
#endif
						{
#ifdef DUET_5LC
							// Drivers 5-7 are on the expansion board, which may not be present. So if they consistently time out, ignore them.
							if (i < 5 || driverStates[i].DriverAssumedPresent())
							{
#endif
								allInitialised = false;
								break;
#ifdef DUET_5LC
							}
#endif
						}
					}

					if (allInitialised)
					{
						fastDigitalWriteLow(GlobalTmc22xxEnablePin);
						driversState = DriversState::ready;
					}
				}
#if TMC22xx_SINGLE_DRIVER
				delay(2);						// TMC22xx can't handle back-to-back reads, so we need a short delay
#endif
			}
#if TMC22xx_USES_SERCOM
			else if (dmaFinishedReason != DmaCallbackReason::none)
			{
				// DMA error, or DMA complete and DMA error
				currentDriver->DmaError();
# if TMC22xx_SINGLE_DRIVER
				delay(2);						// TMC22xx can't handle back-to-back reads, so we need a short delay
# endif
			}
#endif
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

#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
# if TMC22xx_USES_SERCOM
	// Set up the single UART that communicates with all TMC22xx drivers
	SetPinFunction(TMC22xxSercomTxPin, TMC22xxSercomTxPinPeriphMode);
	SetPinFunction(TMC22xxSercomRxPin, TMC22xxSercomRxPinPeriphMode);

	Serial::InitUart(TMC22xxSercomNumber, DriversBaudRate, TMC22xxSercomRxPad, true);
	DmacManager::SetInterruptCallback(DmacChanTmcRx, TransferCompleteCallback, CallbackParameter(nullptr));
# else
	// Set up the single UART that communicates with all TMC22xx drivers
	ConfigurePin(TMC22xx_UART_PINS);									// the pins are already set up for UART use in the pins table

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
#endif

#if TMC22xx_USE_SLAVEADDR && !defined(DUET_5LC)
	currentMuxState = false;
#endif

	driversState = DriversState::noPower;
	for (size_t drive = 0; drive < GetNumTmcDrivers(); ++drive)
	{
#if !(TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER)
# if TMC22xx_USES_SERCOM
		// Initialise the SERCOM that controls this driver
		gpio_set_pin_function(TMC22xxSercomTxPins[drive], TMC22xxSercomTxPinPeriphModes[drive]);
		gpio_set_pin_function(TMC22xxSercomRxPins[drive], TMC22xxSercomRxPinPeriphModes[drive]);

		Serial::InitUart(TMC22xxUarts[drive], TMC22xxSercomNumbers[drive], DriversBaudRate);
		NVIC_EnableIRQ(TMC22xxSercomIRQns[drive]);
# else
		// Initialise the UART that controls this driver
		// The pins are already set up for UART use in the pins table
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

	tmcTask.Create(TmcLoop, "TMC", nullptr, TaskPriority::TmcPriority);
}

// Shut down the drivers and stop any related interrupts. Don't call Spin() again after calling this as it may re-enable them.
void SmartDrivers::Exit() noexcept
{
	IoPort::SetPinMode(GlobalTmc22xxEnablePin, OUTPUT_HIGH);
#if TMC22xx_HAS_MUX || TMC22xx_SINGLE_DRIVER
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
	driversState = DriversState::noPower;
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

uint32_t SmartDrivers::GetLiveStatus(size_t drive) noexcept
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].ReadLiveStatus() : 0;
}

uint32_t SmartDrivers::GetAccumulatedStatus(size_t drive, uint32_t bitsToKeep) noexcept
{
	return (drive < GetNumTmcDrivers()) ? driverStates[drive].ReadAccumulatedStatus(bitsToKeep) : 0;
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
// Before the first call to this function with 'powered' true, you must call Init()
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
	else
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

#endif

// End
