#include <peripheral_clk_config.h>
#include <hpl_oscctrl_config.h>
#include <hal_init.h>

#include "RepRapFirmware.h"
#include "Tasks.h"

extern "C" void AppInit() noexcept
{
	// We don't know which bootloader we entered from, and they use different clocks, so configure the clocks.
	// First reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }

	// Disable DPLL0 so that we can reprogram it
	OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE) { }

	// Also disable DPLL1
	OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.ENABLE) { }

	// Now it's safe to configure the clocks
	init_mcu();

	// Now that GLK1 is running, reconfigure DFLL48M in closed loop mode
	hri_gclk_write_PCHCTRL_reg(GCLK, OSCCTRL_GCLK_ID_DFLL48, GCLK_PCHCTRL_GEN(GclkNum32KHz) | GCLK_PCHCTRL_CHEN);

	const uint8_t tmp = (CONF_DFLL_WAITLOCK << OSCCTRL_DFLLCTRLB_WAITLOCK_Pos) | (CONF_DFLL_BPLCKC << OSCCTRL_DFLLCTRLB_BPLCKC_Pos)
					  | (CONF_DFLL_QLDIS << OSCCTRL_DFLLCTRLB_QLDIS_Pos) | (CONF_DFLL_CCDIS << OSCCTRL_DFLLCTRLB_CCDIS_Pos)
					  | (CONF_DFLL_USBCRM << OSCCTRL_DFLLCTRLB_USBCRM_Pos) | (CONF_DFLL_LLAW << OSCCTRL_DFLLCTRLB_LLAW_Pos)
					  | (CONF_DFLL_STABLE << OSCCTRL_DFLLCTRLB_STABLE_Pos) | (1u << OSCCTRL_DFLLCTRLB_MODE_Pos);
	hri_oscctrl_write_DFLLCTRLB_reg(OSCCTRL, tmp);
	while (hri_oscctrl_get_DFLLSYNC_DFLLCTRLB_bit(OSCCTRL)) { }

	// All done
	SystemCoreClock = SystemCoreClockFreq;
}

// End
