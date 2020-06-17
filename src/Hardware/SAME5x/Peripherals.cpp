/*
 * SAME5x.cpp
 *
 *  Created on: 1 Jul 2019
 *      Author: David
 */

#include "Peripherals.h"

void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal)
{
	static constexpr uint8_t TcClockIDs[] =
	{
		TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID,
		TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TcClockIDs[tcNumber], gclkVal | (1 << GCLK_PCHCTRL_CHEN_Pos));

	switch (tcNumber)
	{
	case 0:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0; break;
	case 1:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1; break;
	case 2:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2; break;
	case 3:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3; break;
	case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
	case 5:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5; break;
	case 6: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC6; break;
	case 7: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC7; break;
	}
}

void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal)
{
	static constexpr uint8_t TccClockIDs[] =
	{
		TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID,
		TCC3_GCLK_ID, TCC4_GCLK_ID
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TccClockIDs[tccNumber], gclkVal | (1 << GCLK_PCHCTRL_CHEN_Pos));

	switch (tccNumber)
	{
	case 0:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0; break;
	case 1:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1; break;
	case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2; break;
	case 3:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC3; break;
	case 4:	MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4; break;
	}
}

// End
