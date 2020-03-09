/*
 * SoftwareReset.h
 *
 *  Created on: 15 Nov 2019
 *      Author: David
 */

#ifndef SRC_SOFTWARERESET_H_
#define SRC_SOFTWARERESET_H_

#include "RepRapFirmware.h"

#if SAM3XA
# include <DueFlashStorage.h>
#endif

// Enumeration describing the reasons for a software reset.
// The spin state gets or'ed into this, so keep the lower 5 bits unused.
// IMPORTANT! When changing this, also update table SoftwareResetReasonText
enum class SoftwareResetReason : uint16_t
{
	user = 0u,						// M999 command
	erase = 1u << 5,				// special M999 command to erase firmware and reset
	NMI = 2u << 5,
	hardFault = 3u << 5,			// most exceptions get escalated to a hard fault
	stuckInSpin = 4u << 5,			// we got stuck in a Spin() function in the Main task for too long
	wdtFault = 5u << 5,				// secondary watchdog
	usageFault = 6u << 5,
	otherFault = 7u << 5,
	stackOverflow = 8u << 5,		// FreeRTOS detected stack overflow
	assertCalled = 9u << 5,			// FreeRTOS assertion failure
	heaterWatchdog = 10u << 5,		// the Heat task didn't kick the watchdog often enough
	memFault = 11u << 5,			// the MPU raised a fault
	terminateCalled = 12u << 5,		// std::terminate was called
	pureVirtual = 13u << 5,
	deletedVirtual = 14u << 5,

	mainReasonMask = 0x0F << 5,		// mask to pick out the  main reason in a uint16_t

	// Bits that are or'ed in
	unusedBit = 0x0200,				// spare bit
	unused2 = 0x0400,				// spare bit
	inAuxOutput = 0x0800,			// this bit is or'ed in if we were in aux output at the time
	unused_was_inLwipSpin = 0x2000,	// no longer used
	inUsbOutput = 0x4000,			// this bit is or'ed in if we were in USB output at the time
	deliberate = 0x8000				// this but it or'ed in if we deliberately caused a fault
};

// These are the structures used to hold our non-volatile data.
// The SAM3X and SAM4E don't have EEPROM so we save the data to flash. This unfortunately means that it gets cleared
// every time we reprogram the firmware via bossa, but it can be retained when firmware updates are performed
// via the web interface. That's why it's a good idea to implement versioning here - increase these values
// whenever the fields of the following structs have changed.
//
// The SAM4E has a large page erase size (8K). For this reason we store the software reset data in the 512-byte user signature area
// instead, which doesn't get cleared when the Erase button is pressed. The SoftareResetData struct must have at least one 32-bit
// field to guarantee that values of this type will be 32-bit aligned. It must have no virtual members because it is read/written
// directly from/to flash memory.
struct SoftwareResetData
{
	uint16_t magic;								// the magic number, including the version
	uint16_t resetReason;						// this records why we did a software reset, for diagnostic purposes
	uint32_t neverUsedRam;						// the amount of never used RAM at the last abnormal software reset
	uint32_t hfsr;								// hard fault status register
	uint32_t cfsr;								// configurable fault status register
	uint32_t icsr;								// interrupt control and state register
	uint32_t bfar;								// bus fault address register
	uint32_t sp;								// stack pointer
	uint32_t when;								// value of the RTC when the software reset occurred
	uint32_t taskName;							// first 4 bytes of the task name
	uint32_t stack[23];							// stack when the exception occurred, with the program counter at the bottom

	bool isVacant() const noexcept;				// return true if this struct can be written without erasing it first
	void Populate(uint16_t reason, uint32_t time, const uint32_t *stk) noexcept;

	static const uint16_t versionValue = 8;		// increment this whenever this struct changes
	static const uint16_t magicValue = 0x7D00 | versionValue;	// value we use to recognise that all the flash data has been written

#ifdef __LPC17xx__
	// Software Reset Data is stored in Flash. Since the LPC1768/9 doesn't have the page erase IAP command,
	// so we have to use the whole sector (last sector of flash is used)
	// The last sector size is 32k. IAP requires us to write at least 256 bytes and the destination address needs to be on a 256 byte boundary.
	// Therefore can have 32k/256=128 slots and we will fill the entire sector before erasing it
	static const size_t numberOfSlots = 128;    // number of storage slots used to implement wear levelling
#else
    static const size_t numberOfSlots = 4;		// number of storage slots used to implement wear levelling - must fit in 512 bytes
#endif

#if SAM3XA
	static const uint32_t nvAddress = 0;		// must be 4-byte aligned
#endif

	static const char *const ReasonText[];
	static uint8_t extraDebugInfo;				// extra info for debugging can be stored here
};

#if SAM4E || SAM4S || SAME70
static_assert(SoftwareResetData::numberOfSlots * sizeof(SoftwareResetData) <= 512, "Can't fit software reset data in user signature area");
#else
static_assert(SoftwareResetData::numberOfSlots * sizeof(SoftwareResetData) <= FLASH_DATA_LENGTH, "NVData too large");
#endif

#endif /* SRC_SOFTWARERESET_H_ */
