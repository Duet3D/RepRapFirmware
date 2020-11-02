#include <CoreIO.h>
#include <hri_oscctrl_e54.h>
#include <hri_gclk_e54.h>

void AppInit() noexcept
{
	// Initialise FDPLL1. (25MHz / 10) * 72 = 180MHz which we will divide by 2 to get 90MHz.
	hri_oscctrl_write_DPLLRATIO_reg(OSCCTRL, 1,
			  OSCCTRL_DPLLRATIO_LDRFRAC(0)
			| OSCCTRL_DPLLRATIO_LDR(71));
	hri_oscctrl_write_DPLLCTRLB_reg(OSCCTRL, 1,
			  OSCCTRL_DPLLCTRLB_DIV(4)
			| (0 << OSCCTRL_DPLLCTRLB_DCOEN_Pos)
			| OSCCTRL_DPLLCTRLB_DCOFILTER(0)
			| (0 << OSCCTRL_DPLLCTRLB_LBYPASS_Pos)
			| OSCCTRL_DPLLCTRLB_LTIME(0)
			| OSCCTRL_DPLLCTRLB_REFCLK_XOSC1
			| (0 << OSCCTRL_DPLLCTRLB_WUF_Pos)
			| OSCCTRL_DPLLCTRLB_FILTER(0));
	hri_oscctrl_write_DPLLCTRLA_reg(OSCCTRL, 1,
			  (0 << OSCCTRL_DPLLCTRLA_RUNSTDBY_Pos)
			| (1 << OSCCTRL_DPLLCTRLA_ENABLE_Pos));

	while (!(hri_oscctrl_get_DPLLSTATUS_LOCK_bit(OSCCTRL, 1) || hri_oscctrl_get_DPLLSTATUS_CLKRDY_bit(OSCCTRL, 1))) { }

	// Initialise the GCLKs we use that are not initialised by CoreNG

	// GCLK2 is used by the Ethernet version only. We initialise it in GmacInterface.

	// GCLK5: FDPLL1, 90MHz for SDHC
	hri_gclk_write_GENCTRL_reg(GCLK, 5,
			  GCLK_GENCTRL_DIV(2) | (0 << GCLK_GENCTRL_RUNSTDBY_Pos)
			| (0 << GCLK_GENCTRL_DIVSEL_Pos) | (0 << GCLK_GENCTRL_OE_Pos)
			| (0 << GCLK_GENCTRL_OOV_Pos) | (0 << GCLK_GENCTRL_IDC_Pos)
			| GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DPLL1);
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
	return 25;
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
	return 1;
}

// Return get the SDHC peripheral clock speed in Hz. This must be provided by the client project if using SDHC.
uint32_t AppGetSdhcClockSpeed() noexcept
{
	return 90000000;
}

// End
