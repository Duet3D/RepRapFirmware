/*
 * Dmac.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_SAME5X_DMACMANAGER_H_
#define SRC_HARDWARE_SAME5X_DMACMANAGER_H_

#include "RepRapFirmware.h"

// Status code indicating why a DMAC callback is happening
enum class DmaCallbackReason : uint8_t
{
	none = 0,
	error = DMAC_CHINTFLAG_TERR,
	complete = DMAC_CHINTFLAG_TCMPL,
	completeAndError = DMAC_CHINTFLAG_TERR | DMAC_CHINTFLAG_TCMPL
};

typedef void (*DmaCallbackFunction)(CallbackParameter cb, DmaCallbackReason reason);

enum class DmaTrigSource : uint8_t
{
#if defined(SAME51)

	disable = 0,
	rtc,
	dsu_dcc0,
	dsu_dcc1,

	sercom0_rx,
	sercom0_tx,
	sercom1_rx,
	sercom1_tx,
	sercom2_rx,
	sercom2_tx,
	sercom3_rx,
	sercom3_tx,
	sercom4_rx,
	sercom4_tx,
	sercom5_rx,
	sercom5_tx,
	sercom6_rx,
	sercom6_tx,
	sercom7_rx,
	sercom7_tx,

	can0_debug,
	can1_debug,

	tcc0_ovf = 0x16,
	tcc0_mc = 0x17,
	tcc1_ovf = 0x1D,
	tcc1_mc = 0x1E,
	tcc2_ovf = 0x22,
	tcc2_mc = 0x23,
	tcc3_ovf = 0x26,
	tcc3_mc = 0x27,
	tcc4_ovf = 0x29,
	tcc4_mc = 0x2A,

	tc0_ovf = 0x2C,
	tc0_mc = 0x2D,
	tc1_ovf = 0x2F,
	tc1_mc = 0x30,
	tc2_ovf = 0x32,
	tc2_mc = 0x33,
	tc3_ovf = 0x35,
	tc3_mc = 0x36,
	tc4_ovf = 0x38,
	tc4_mc = 0x39,
	tc5_ovf = 0x3B,
	tc5_mc = 0x3C,
	tc6_ovf = 0x3E,
	tc6_mc = 0x3F,
	tc7_ovf = 0x40,
	tc7_mc = 0x41,

	adc0_resrdy = 0x44,
	adc0_seq,
	adc1_resrdy,
	adc1_seq,

	dac_empty = 0x48,
	dac_resrdy = 0x4A,

	i2s_rx = 0x4C,
	i2s_tx = 0x4E,

	pcc = 0x50,

	aes_wr = 0x51,
	aes_rd,

	qspi_rx = 0x53,
	qspi_tx

#elif defined(SAMC21)

	disable = 0,
	tsens,

	sercom0_rx,
	sercom0_tx,
	sercom1_rx,
	sercom1_tx,
	sercom2_rx,
	sercom2_tx,
	sercom3_rx,
	sercom3_tx,
	sercom4_rx,
	sercom4_tx,
	sercom5_rx,
	sercom5_tx,

	can0_debug,
	can1_debug,

	tcc0_ovf,
	tcc0_mc0,
	tcc0_mc1,
	tcc0_mc2,
	tcc0_mc3,
	tcc1_ovf,
	tcc1_mc0,
	tcc1_mc1,
	tcc2_ovf,
	tcc2_mc0,
	tcc2_mc1,

	tc0_ovf,
	tc0_mc0,
	tc0_mc1,
	tc1_ovf,
	tc1_mc0,
	tc1_mc1,
	tc2_ovf,
	tc2_mc0,
	tc2_mc1,
	tc3_ovf,
	tc3_mc0,
	tc3_mc1,
	tc4_ovf,
	tc4_mc0,
	tc4_mc1,

	adc0_resrdy,
	adc1_resrdy,
	sdadc_resrdy,

	dac_empty,
	ptc_eoc,
	ptc_wcomp,
	ptc_seq,

# if 0	// these are only available on the SAMC21N, which we don't support
	sercom6_rx,
	sercom6_tx,
	sercom7_rx,
	sercom7_tx,

	tc5_ovf,
	tc5_mc0,
	tc5_mc1,
	tc6_ovf,
	tc6_mc0,
	tc6_mc1,
	tc7_ovf,
	tc7_mc0,
	tc7_mc1

# endif
#else
# error Unsupported processor
#endif
};

#if defined(SAMC21)
static_assert((uint8_t)DmaTrigSource::ptc_seq == 0x30, "Error in DmaTrigSource enumeration");
#endif

// The following works for all sercoms on the SAME51 and sercoms 1 to 5 on the SAMC21. We don't support sercoms 6-7 on the SAMC21 because they only exist on the 100-pin version.
static inline uint8_t GetSercomTxTrigSource(uint8_t sercomNumber)
{
	return (uint8_t)DmaTrigSource::sercom0_tx + (sercomNumber * 2);
}

// The following works for all sercoms on the SAME51 and sercoms 1 to 5 on the SAMC21. We don't support sercoms 6-7 on the SAMC21 because they only exist on the 100-pin version.
static inline uint8_t GetSercomRxTrigSource(uint8_t sercomNumber)
{
	return (uint8_t)DmaTrigSource::sercom0_rx + (sercomNumber * 2);
}

namespace DmacManager
{
	void Init();
	void SetDestinationAddress(uint8_t channel, volatile void *const dst);
	void SetSourceAddress(uint8_t channel, const volatile void *const src);
	void SetDataLength(uint8_t channel, uint32_t amount);
	void SetBtctrl(uint8_t channel, uint16_t val);
	void SetTriggerSource(uint8_t channel, DmaTrigSource source);
	void SetTriggerSourceSercomTx(uint8_t channel, uint8_t sercomNumber);
	void SetTriggerSourceSercomRx(uint8_t channel, uint8_t sercomNumber);
	void SetArbitrationLevel(uint8_t channel, uint8_t level);
	void EnableChannel(uint8_t channel, uint8_t priority);
	void DisableChannel(uint8_t channel);
	void SetInterruptCallback(uint8_t channel, DmaCallbackFunction fn, CallbackParameter param);
	void EnableCompletedInterrupt(uint8_t channel);
	void DisableCompletedInterrupt(uint8_t channel);
	uint8_t GetChannelStatus(uint8_t channel);
	uint16_t GetBytesTransferred(uint8_t channel);
}

#endif /* SRC_HARDWARE_SAME5X_DMACMANAGER_H_ */
