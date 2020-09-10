/*
 * SoftwareReset.cpp
 *
 *  Created on: 15 Nov 2019
 *      Author: David
 */

#include "SoftwareReset.h"
#include <Tasks.h>
#include <RepRap.h>
#include <Platform.h>

extern uint32_t _estack;			// defined in the linker script

// The following must be kept in line with enum class SoftwareResetReason
const char *const SoftwareResetData::ReasonText[] =
{
	"User",
	"Erase",
	"NMI",
	"HardFault",
	"StuckInSpinLoop",
	"WatchdogTimeout",
	"UsageFault",
	"OtherFault",
	"StackOverflow",
	"AssertionFailed",
	"HeatTaskStuck",
	"MemoryProtectionFault",
	"TerminateCalled",
	"PureVirtualFunctionCalled",
	"DeletedVirtualFunctionCalled",
	"Unknown"
};

uint8_t SoftwareResetData::extraDebugInfo;			// extra info for debugging

// Return true if this struct can be written without erasing it first
bool SoftwareResetData::IsVacant() const noexcept
{
	const uint32_t *p = reinterpret_cast<const uint32_t*>(this);
	for (size_t i = 0; i < sizeof(*this)/sizeof(uint32_t); ++i)
	{
		if (*p != 0xFFFFFFFF)
		{
			return false;
		}
		++p;
	}
	return true;
}

void SoftwareResetData::Clear() noexcept
{
	memset(this, 0xFF, sizeof(*this));
}

// Populate this reset data from the parameters passed and the CPU state
void SoftwareResetData::Populate(uint16_t reason, const uint32_t *stk) noexcept
{
	magic = magicValue;
	resetReason = reason | ((extraDebugInfo & 0x07) << 5);
	when = (uint32_t)reprap.GetPlatform().GetDateTime();
	neverUsedRam = Tasks::GetNeverUsedRam();
	hfsr = SCB->HFSR;
	cfsr = SCB->CFSR;
	icsr = SCB->ICSR;
#if USE_MPU
	if ((reason & (uint16_t)SoftwareResetReason::mainReasonMask) == (uint16_t)SoftwareResetReason::memFault)
	{
		bfar = SCB->MMFAR;				// on a memory fault we store the MMFAR instead of the BFAR
	}
	else
	{
		bfar = SCB->BFAR;
	}
#else
	bfar = SCB->BFAR;
#endif
	// Get the task name if we can. There may be no task executing, so we must allow for this.
	const TaskHandle_t currentTask = xTaskGetCurrentTaskHandle();
	taskName = (currentTask == nullptr) ? 0 : *reinterpret_cast<const uint32_t*>(pcTaskGetName(currentTask));

	if (stk != nullptr)
	{
		sp = reinterpret_cast<uint32_t>(stk);
		for (uint32_t& stval : stack)
		{
			stval = (stk < &_estack) ? *stk++ : 0xFFFFFFFF;
		}
	}
}

void SoftwareResetData::Print(MessageType mtype, unsigned int slot) const noexcept
{
	String<StringLength256> scratchString;		// long enough to print 28 stack entries @ 9 bytes each, but not 29
	scratchString.copy("Last software reset ");
	if (when != 0)
	{
		const time_t whenTime = (time_t)when;
		tm timeInfo;
		gmtime_r(&whenTime, &timeInfo);
		scratchString.catf("at %04u-%02u-%02u %02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
	}
	else
	{
		scratchString.cat("time unknown");
	}
	scratchString.cat(", reason: ");
	if (resetReason & (uint32_t)SoftwareResetReason::deliberate)
	{
		scratchString.cat("deliberate ");
	}
	scratchString.cat(ReasonText[(resetReason >> 5) & 0x0F]);

	// If it's a forced hard fault or a memory access fault, provide some more information
	if ((resetReason & (uint16_t)SoftwareResetReason::mainReasonMask) == (uint16_t)SoftwareResetReason::hardFault && (hfsr & 1u << 30) != 0)
	{
		if (cfsr & (1u << 25)) { scratchString.cat(" zeroDiv"); }
		if (cfsr & (1u << 24)) { scratchString.cat(" unaligned"); }
		if (cfsr & (1u << 18)) { scratchString.cat(" invPC"); }
		if (cfsr & (1u << 17)) { scratchString.cat(" invState"); }
		if (cfsr & (1u << 16)) { scratchString.cat(" undefInstr"); }
		if (cfsr & (1u << 15)) { scratchString.cat(" bfarValid"); }
		if (cfsr & (1u << 12)) { scratchString.cat(" stkErr"); }
		if (cfsr & (1u << 11)) { scratchString.cat(" unstkErr"); }
		if (cfsr & (1u << 10)) { scratchString.cat(" imprec"); }
		if (cfsr & (1u << 9)) { scratchString.cat(" precise"); }
		if (cfsr & (1u << 8)) { scratchString.cat(" ibus"); }
	}
#if USE_MPU
	else if ((resetReason & (uint16_t)SoftwareResetReason::mainReasonMask) == (uint16_t)SoftwareResetReason::memFault)
	{
		if (cfsr & (1u << 7)) { scratchString.cat(" mmarValid"); }
		if (cfsr & (1u << 4)) { scratchString.cat(" mstkErr"); }
		if (cfsr & (1u << 3)) { scratchString.cat(" munstkErr"); }
		if (cfsr & (1u << 1)) { scratchString.cat(" daccViol"); }
		if (cfsr & (1u << 0)) { scratchString.cat(" iaccViol"); }
	}
#endif
	reprap.GetPlatform().MessageF(mtype, "%s, %s spinning, available RAM %" PRIu32 ", slot %u\n",
						scratchString.c_str(),
						GetModuleName(resetReason & 0x1F),
						neverUsedRam,
						slot);

	// Our format buffer is only 256 characters long, so the next 2 lines must be written separately
	// The task name may include nulls at the end, so print it as a string
	const uint32_t taskNameWords[2] = { taskName, 0u };
	reprap.GetPlatform().MessageF(mtype,
			"Software reset code 0x%04x HFSR 0x%08" PRIx32 " CFSR 0x%08" PRIx32 " ICSR 0x%08" PRIx32 " BFAR 0x%08" PRIx32 " SP 0x%08" PRIx32 " Task %s\n",
			resetReason, hfsr, cfsr, icsr, bfar, sp, (const char *)taskNameWords
		);
	if (sp != 0xFFFFFFFF)
	{
		// We saved a stack dump, so print it
		scratchString.Clear();
		for (uint32_t stval : stack)
		{
			scratchString.catf(" %08" PRIx32, stval);
		}
		reprap.GetPlatform().MessageF(mtype, "Stack:%s\n", scratchString.c_str());
	}
}

// End
