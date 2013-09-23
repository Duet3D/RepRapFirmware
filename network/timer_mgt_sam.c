/**
 * \file
 *
 * \brief Timer management for lwIP example.
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

//#include "board.h"
#include "timer_mgt_sam.h"
#include "include/tc.h"
#include "include/pmc.h"
//#include "sysclk.h"
#include "lwip/src/include/lwip/sys.h"
#include "lwip_test.h"

volatile int ledState;
/** Clock tick count */
static volatile uint32_t gs_ul_clk_tick;

/**
 *  Interrupt handler for TC0 interrupt.
 */
//void TC0_Handler(void)
//{
	/* Remove warnings */
//	volatile uint32_t ul_dummy;

	/* Clear status bit to acknowledge interrupt */
//	ul_dummy = TC0->TC_CHANNEL[0].TC_SR;

	/* Increase tick */
//	gs_ul_clk_tick++;
//}

void TC4_Handler()
{
  // You must do TC_GetStatus to "accept" interrupt
  // As parameters use the first two parameters used in startTimer (TC1, 0 in this case)
  TC_GetStatus(TC1, 1);

	/* Increase tick */
	gs_ul_clk_tick++;

	ledState = !ledState;
}

/**
 * \brief Initialize for timing operation.
 */
void sys_init_timing(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;

	/* Clear tick value */
	gs_ul_clk_tick = 0;

	startTimer(TC1,1,TC4_IRQn,4);

}
// Start timer. Parameters are:

  // TC1 : timer counter. Can be TC0, TC1 or TC2
  // 0   : channel. Can be 0, 1 or 2
  // TC3_IRQn: irq number. See table.
  // 40  : frequency (in Hz)
  // The interrupt service routine is TC3_Handler. See table.
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
        TC_SetRA(tc, channel, rc/2); //50% high, 50% low
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}
// Paramters table:
// TC0, 0, TC0_IRQn  =>  TC0_Handler()
// TC0, 1, TC1_IRQn  =>  TC1_Handler()
// TC0, 2, TC2_IRQn  =>  TC2_Handler()
// TC1, 0, TC3_IRQn  =>  TC3_Handler()
// TC1, 1, TC4_IRQn  =>  TC4_Handler()
// TC1, 2, TC5_IRQn  =>  TC5_Handler()
// TC2, 0, TC6_IRQn  =>  TC6_Handler()
// TC2, 1, TC7_IRQn  =>  TC7_Handler()
// TC2, 2, TC8_IRQn  =>  TC8_Handler()


/**
 * \brief Read for clock time (ms).
 */
uint32_t sys_get_ms(void)
{
	return gs_ul_clk_tick;
}

#if (THIRDPARTY_LWIP_VERSION != 132)

/* See lwip/sys.h for more information
   Returns number of milliseconds expired
   since lwip is initialized
*/
u32_t sys_now(void)
{
	return (sys_get_ms());
}

#endif
