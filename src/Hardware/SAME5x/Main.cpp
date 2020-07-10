#include <peripheral_clk_config.h>
#include <hpl_oscctrl_config.h>
#include <hal_init.h>

#include "RepRapFirmware.h"
#include "Tasks.h"

int main(void)
{
#if SUPPORT_CAN_EXPANSION
	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);
	CAN0->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN0->ILE.reg = 0;
	CAN1->IR.reg = 0xFFFFFFFF;			// clear all interrupt sources for when the device gets enabled by the main firmware
	CAN1->ILE.reg = 0;
#endif

	// If we have entered via the bootloader then we have already configured the clocks.
	// We could just leave them alone, but only if we definitely want to use the same clock configuration.
	// So instead we reset the clock configuration.
	// First reset the generic clock generator. This sets all clock generators to default values and the CPU clock to the 48MHz DFLL output.
	GCLK->CTRLA.reg = GCLK_CTRLA_SWRST;
	while ((GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) != 0) { }

	// Function init_mcu sets up DPLL0 to generate the main clock from XOSC0. If we don't disable DPLL0 here then that fails in a release build.
	OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[0].DPLLSYNCBUSY.bit.ENABLE) { }

	// Also disable DPLL1 because we are going to program it to generate the 48MHz CAN clock
	OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0;
	while (OSCCTRL->Dpll[1].DPLLSYNCBUSY.bit.ENABLE) { }

	// Now it's safe to configure the clocks
	init_mcu();

	// Now that GLK1 is running, reconfigure DFLL48M in closed loop mode
	hri_gclk_write_PCHCTRL_reg(GCLK, OSCCTRL_GCLK_ID_DFLL48, (1 << GCLK_PCHCTRL_CHEN_Pos) | GCLK_PCHCTRL_GEN(CONF_DFLL_GCLK));

	const uint8_t tmp = (CONF_DFLL_WAITLOCK << OSCCTRL_DFLLCTRLB_WAITLOCK_Pos) | (CONF_DFLL_BPLCKC << OSCCTRL_DFLLCTRLB_BPLCKC_Pos)
					  | (CONF_DFLL_QLDIS << OSCCTRL_DFLLCTRLB_QLDIS_Pos) | (CONF_DFLL_CCDIS << OSCCTRL_DFLLCTRLB_CCDIS_Pos)
					  | (CONF_DFLL_USBCRM << OSCCTRL_DFLLCTRLB_USBCRM_Pos) | (CONF_DFLL_LLAW << OSCCTRL_DFLLCTRLB_LLAW_Pos)
					  | (CONF_DFLL_STABLE << OSCCTRL_DFLLCTRLB_STABLE_Pos) | (1u << OSCCTRL_DFLLCTRLB_MODE_Pos);
	hri_oscctrl_write_DFLLCTRLB_reg(OSCCTRL, tmp);
	while (hri_oscctrl_get_DFLLSYNC_DFLLCTRLB_bit(OSCCTRL)) { }

	// All done
	SystemCoreClock = CONF_CPU_FREQUENCY;			// GCLK0
	SystemPeripheralClock = CONF_CPU_FREQUENCY/2;	// GCLK3

	AppMain();
}

// End
