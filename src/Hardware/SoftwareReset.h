/*
 * SoftwareReset.h
 *
 *  Created on: 15 Nov 2019
 *      Author: David
 */

#ifndef SRC_SOFTWARERESET_H_
#define SRC_SOFTWARERESET_H_

#include <RepRapFirmware.h>

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
	pureOrDeletedVirtual = 13u << 5,
	outOfMemory = 14u << 5,
	//unused = 15u << 5,

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
// We store the software reset data in the 512-byte user signature area of the SAM4E, SAM4S and SAME70 processors.
// It must be a multiple of 4 bytes long.
struct SoftwareResetData
{
	uint16_t magic;								// the magic number, including the version
	uint16_t resetReason;						// this records why we did a software reset, for diagnostic purposes
	int32_t neverUsedRam;						// the amount of never used RAM at the last abnormal software reset
	uint32_t hfsr;								// hard fault status register
	uint32_t cfsr;								// configurable fault status register
	uint32_t icsr;								// interrupt control and state register
	uint32_t bfar;								// bus fault address register
	uint32_t sp;								// stack pointer
	uint32_t when;								// value of the RTC when the software reset occurred
	uint32_t taskName;							// first 4 bytes of the task name, or 'none'
	uint32_t stackOffset;						// how many spare words of stack the running task has
	uint32_t stackMarkerValid : 1,				// true if the stack low marker wasn't overwritten
			 spare : 31;						// unused at present
	// The stack length is set to 27 words because that is the most we can print in a single message using our 256-byte format buffer
	uint32_t stack[27];							// stack when the exception occurred, with the link register and program counter at the bottom

	bool IsVacant() const noexcept;				// return true if this struct can be written without erasing it first
	bool IsValid() const noexcept { return magic == magicValue; }
	void Clear() noexcept;
	void Populate(uint16_t reason, const uint32_t *stk) noexcept;
	void Printit(MessageType mtype, unsigned int slot) const noexcept;

	static constexpr uint16_t versionValue = 9;		// increment this whenever this struct changes
	static constexpr uint16_t magicValue = 0x7D00 | versionValue;	// value we use to recognise that all the flash data has been written

	static const char *const ReasonText[];
	static uint8_t extraDebugInfo;				// 3 bits of extra info for debugging can be stored here
};

#endif /* SRC_SOFTWARERESET_H_ */
