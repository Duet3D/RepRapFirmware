/*
 * Dmac.cpp
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#include <utils.h>
#include "Peripherals.h"

#if defined(SAME51)
# include <hri_dmac_e51.h>
#elif defined(SAMC21)
# include <hri_dmac_c21.h>
#else
# error Unsupported processor
#endif

#include <Hardware/DmacManager.h>
#include <RTOSIface/RTOSIface.h>

// Descriptors for all used DMAC channels
COMPILER_ALIGNED(16)
static DmacDescriptor descriptor_section[NumDmaChannelsUsed];

// Write back descriptors for all used DMAC channels
static DmacDescriptor write_back_section[NumDmaChannelsUsed];

// Array containing callbacks for DMAC channels
static DmaCallbackFunction dmaChannelCallbackFunctions[NumDmaChannelsUsed];
static CallbackParameter callbackParams[NumDmaChannelsUsed];

// Initialize the DMA controller
void DmacManager::Init()
{
	hri_mclk_set_AHBMASK_DMAC_bit(MCLK);

	hri_dmac_clear_CTRL_DMAENABLE_bit(DMAC);
#if defined(SAME51)
	hri_dmac_clear_CRCCTRL_reg(DMAC, DMAC_CRCCTRL_CRCSRC_Msk);
#elif defined(SAMC21)
	hri_dmac_clear_CTRL_CRCENABLE_bit(DMAC);
#else
# error Unsupported processor
#endif
	hri_dmac_set_CTRL_SWRST_bit(DMAC);
	while (hri_dmac_get_CTRL_SWRST_bit(DMAC)) { }

	hri_dmac_write_CTRL_reg(DMAC, DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 | DMAC_CTRL_LVLEN2 | DMAC_CTRL_LVLEN3);
	hri_dmac_write_DBGCTRL_DBGRUN_bit(DMAC, 0);

#if defined(SAME51)
	hri_dmac_write_PRICTRL0_reg(DMAC, DMAC_PRICTRL0_RRLVLEN0 | DMAC_PRICTRL0_RRLVLEN1 | DMAC_PRICTRL0_RRLVLEN2 | DMAC_PRICTRL0_RRLVLEN3
									| DMAC_PRICTRL0_QOS0(0x02) | DMAC_PRICTRL0_QOS1(0x02)| DMAC_PRICTRL0_QOS2(0x02)| DMAC_PRICTRL0_QOS3(0x02));
#elif defined(SAMC21)
	hri_dmac_write_PRICTRL0_reg(DMAC, DMAC_PRICTRL0_RRLVLEN0 | DMAC_PRICTRL0_RRLVLEN1 | DMAC_PRICTRL0_RRLVLEN2 | DMAC_PRICTRL0_RRLVLEN3);
	hri_dmac_write_QOSCTRL_reg(DMAC, DMAC_QOSCTRL_WRBQOS(0x02) | DMAC_QOSCTRL_FQOS(0x02) | DMAC_QOSCTRL_DQOS(0x02));
#endif

	hri_dmac_write_BASEADDR_reg(DMAC, (uint32_t)descriptor_section);
	hri_dmac_write_WRBADDR_reg(DMAC, (uint32_t)write_back_section);

#if defined(SAME51)
	// SAME5x DMAC has 5 contiguous IRQ numbers
	for (unsigned int i = 0; i < 5; i++)
	{
		NVIC_DisableIRQ((IRQn)(DMAC_0_IRQn + i));
		NVIC_ClearPendingIRQ((IRQn)(DMAC_0_IRQn + i));
		NVIC_SetPriority((IRQn)(DMAC_0_IRQn + i), NvicPriorityDmac);
		NVIC_EnableIRQ((IRQn)(DMAC_0_IRQn + i));
	}
#elif defined(SAMC21)
	NVIC_DisableIRQ(DMAC_IRQn);
	NVIC_ClearPendingIRQ(DMAC_IRQn);
	NVIC_SetPriority(DMAC_IRQn, NvicPriorityDmac);
	NVIC_EnableIRQ(DMAC_IRQn);
#else
# error Unsupported processor
#endif

	hri_dmac_set_CTRL_DMAENABLE_bit(DMAC);
}

void DmacManager::SetBtctrl(const uint8_t channel, const uint16_t val)
{
	hri_dmacdescriptor_write_BTCTRL_reg(&descriptor_section[channel], val);
}

void DmacManager::SetDestinationAddress(const uint8_t channel, volatile void *const dst)
{
	hri_dmacdescriptor_write_DSTADDR_reg(&descriptor_section[channel], reinterpret_cast<uint32_t>(dst));
}

void DmacManager::SetSourceAddress(const uint8_t channel, const volatile void *const src)
{
	hri_dmacdescriptor_write_SRCADDR_reg(&descriptor_section[channel], reinterpret_cast<uint32_t>(src));
}

void DmacManager::SetDataLength(const uint8_t channel, const uint32_t amount)
{
	const uint8_t beat_size = hri_dmacdescriptor_read_BTCTRL_BEATSIZE_bf(&descriptor_section[channel]);

	const uint32_t dstAddress = hri_dmacdescriptor_read_DSTADDR_reg(&descriptor_section[channel]);
	if (hri_dmacdescriptor_get_BTCTRL_DSTINC_bit(&descriptor_section[channel]))
	{
		hri_dmacdescriptor_write_DSTADDR_reg(&descriptor_section[channel], dstAddress + amount * (1 << beat_size));
	}

	const uint32_t srcAddress = hri_dmacdescriptor_read_SRCADDR_reg(&descriptor_section[channel]);
	if (hri_dmacdescriptor_get_BTCTRL_SRCINC_bit(&descriptor_section[channel]))
	{
		hri_dmacdescriptor_write_SRCADDR_reg(&descriptor_section[channel], srcAddress + amount * (1 << beat_size));
	}

	hri_dmacdescriptor_write_BTCNT_reg(&descriptor_section[channel], amount);
}

void DmacManager::SetTriggerSource(uint8_t channel, DmaTrigSource source)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC((uint32_t)source) | DMAC_CHCTRLA_TRIGACT_BURST
											| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC((uint8_t)source) | DMAC_CHCTRLB_TRIGACT_BEAT;
#else
# error Unsupported processor
#endif
}

void DmacManager::SetTriggerSourceSercomRx(uint8_t channel, uint8_t sercomNumber)
{
	const uint32_t source = GetSercomRxTrigSource(sercomNumber);
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(source) | DMAC_CHCTRLA_TRIGACT_BURST
											| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC((uint8_t)source) | DMAC_CHCTRLB_TRIGACT_BEAT;
#else
# error Unsupported processor
#endif
}

// Transmit
void DmacManager::SetTriggerSourceSercomTx(uint8_t channel, uint8_t sercomNumber)
{
	const uint32_t source = GetSercomTxTrigSource(sercomNumber);
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(source) | DMAC_CHCTRLA_TRIGACT_BURST
											| DMAC_CHCTRLA_BURSTLEN_SINGLE | DMAC_CHCTRLA_THRESHOLD_1BEAT;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC((uint8_t)source) | DMAC_CHCTRLB_TRIGACT_BEAT;
#else
# error Unsupported processor
#endif
}

void DmacManager::SetArbitrationLevel(uint8_t channel, uint8_t level)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHPRILVL.reg = level;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.reg = (DMAC->CHCTRLB.reg & ~DMAC_CHCTRLB_LVL_Msk) | (level << DMAC_CHCTRLB_LVL_Pos);
#else
# error Unsupported processor
#endif
}

void DmacManager::EnableChannel(const uint8_t channel, uint8_t priority)
{
	hri_dmacdescriptor_set_BTCTRL_VALID_bit(&descriptor_section[channel]);
#if defined(SAME51)
	DMAC->Channel[channel].CHPRILVL.reg = priority;
	DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 1;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLB.bit.LVL = priority;
	DMAC->CHCTRLA.bit.ENABLE = 1;
#else
# error Unsupported processor
#endif
}

// Disable a channel. Also clears its status and disables its interrupts.
void DmacManager::DisableChannel(const uint8_t channel)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHCTRLA.bit.ENABLE = 0;
	DMAC->Channel[channel].CHINTENCLR.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHCTRLA.bit.ENABLE = 0;
	DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;
#else
# error Unsupported processor
#endif
}

void DmacManager::SetInterruptCallback(uint8_t channel, DmaCallbackFunction fn, CallbackParameter param)
{
	AtomicCriticalSectionLocker lock;
	dmaChannelCallbackFunctions[channel] = fn;
	callbackParams[channel] = param;
}

void DmacManager::EnableCompletedInterrupt(const uint8_t channel)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;
	DMAC->Channel[channel].CHINTENSET.reg = DMAC_CHINTENSET_TCMPL | DMAC_CHINTENSET_TERR;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHINTENCLR.reg = DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP;
	DMAC->CHINTENSET.reg = DMAC_CHINTENSET_TCMPL | DMAC_CHINTENSET_TERR;
#else
# error Unsupported processor
#endif
}

void DmacManager::DisableCompletedInterrupt(const uint8_t channel)
{
#if defined(SAME51)
	DMAC->Channel[channel].CHINTENCLR.reg = DMAC_CHINTENSET_TCMPL | DMAC_CHINTENSET_TERR;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	DMAC->CHINTENCLR.reg = DMAC_CHINTENSET_TCMPL | DMAC_CHINTENSET_TERR;
#else
# error Unsupported processor
#endif
}

uint8_t DmacManager::GetChannelStatus(uint8_t channel)
{
#if defined(SAME51)
	return DMAC->Channel[channel].CHINTFLAG.reg;
#elif defined(SAMC21)
	AtomicCriticalSectionLocker lock;
	DMAC->CHID.reg = channel;
	return DMAC->CHINTFLAG.reg;
#else
# error Unsupported processor
#endif
}

uint16_t DmacManager::GetBytesTransferred(uint8_t channel)
{
	return descriptor_section[channel].BTCNT.reg - write_back_section[channel].BTCNT.reg;
}

#if defined(SAME51)

// Internal DMAC interrupt handler
static inline void CommonDmacHandler(uint8_t channel)
{
	const uint8_t intflag = DMAC->Channel[channel].CHINTFLAG.reg & DMAC->Channel[channel].CHINTENSET.reg & (DMAC_CHINTFLAG_SUSP | DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR);
	if (intflag != 0)					// should always be true
	{
		DMAC->Channel[channel].CHINTFLAG.reg = intflag;
		const DmaCallbackFunction fn = dmaChannelCallbackFunctions[channel];
		if (fn != nullptr)
		{
			fn(callbackParams[channel], (DmaCallbackReason)intflag);
		}
	}
}

extern "C" void DMAC_0_Handler()
{
	CommonDmacHandler(0);
}

extern "C" void DMAC_1_Handler()
{
	CommonDmacHandler(1);
}

extern "C" void DMAC_2_Handler()
{
	CommonDmacHandler(2);
}

extern "C" void DMAC_3_Handler()
{
	CommonDmacHandler(3);
}

extern "C" void DMAC_4_Handler()
{
	hri_dmac_intpend_reg_t intPend;
	while ((intPend = DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk) > 3)
	{
		CommonDmacHandler(intPend);
	}
}

#elif defined(SAMC21)

extern "C" void DMAC_Handler()
{
	hri_dmac_intpend_reg_t intPend;
	while (((intPend = DMAC->INTPEND.reg) & (DMAC_INTPEND_SUSP | DMAC_INTPEND_TCMPL | DMAC_INTPEND_TERR)) != 0)
	{
		const size_t channel = intPend & DMAC_INTPEND_ID_Msk;
		DMAC->CHID.reg = channel;
		const uint8_t intflag = DMAC->CHINTFLAG.reg & DMAC->CHINTENSET.reg & (DMAC_CHINTFLAG_SUSP | DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_TERR);
		if (intflag != 0)					// should always be true
		{
			DMAC->CHINTFLAG.reg = intflag;
			const DmaCallbackFunction fn = dmaChannelCallbackFunctions[channel];
			if (fn != nullptr)
			{
				fn(callbackParams[channel], (DmaCallbackReason)intflag);
			}
		}
	}
}

#else
# error Unsupported processor
#endif

// End
