/*
 * AnalogOut.cpp
 *
 *  Created on: 9 Jul 2019
 *      Author: David
 */

#include "AnalogOut.h"
#include <Hardware/IoPorts.h>

# include "hri_tc_e54.h"
# include "hri_tcc_e54.h"
# include "hri_mclk_e54.h"

namespace AnalogOut
{

	// Convert a float in 0..1 to unsigned integer in 0..N
	static inline uint32_t ConvertRange(float f, uint32_t top)
	pre(0.0 <= ulValue; ulValue <= 1.0)
	post(result <= top)
	{
		return lrintf(f * (float)(top + 1));
	}

	// Choose the most appropriate prescaler for the PWM frequency we want.
	// Some TCs share a clock selection, so we always use GCLK1 as the clock
	// 'counterBits' is either 16 or 8
	// Return the prescaler register value
	static uint32_t ChoosePrescaler(uint16_t freq, unsigned int counterBits, uint32_t& top)
	{
		static const unsigned int PrescalerShifts[] = { 0, 1, 2, 3, 4, 6, 8, 10 };		// available prescalers are 1 2 4 8 16 64 256 1024
		for (uint32_t i = 0; i < ARRAY_SIZE(PrescalerShifts); ++i)
		{
			if ((SystemPeripheralClock >> (PrescalerShifts[i] + counterBits)) <= (uint32_t)freq)
			{
				top = ((SystemPeripheralClock >> PrescalerShifts[i])/(uint32_t)freq) - 1;
				return i;
			}
		}
		top = (1ul << counterBits) - 1;
		return ARRAY_SIZE(PrescalerShifts) - 1;
	}

	// Write PWM to the specified TC device. 'output' may be 0 or 1.
	static bool AnalogWriteTc(Pin pin, unsigned int device, unsigned int output, float val, PwmFrequency freq)
	{
		static volatile Tc* const TcDevices[] =
		{
			TC0, TC1, TC2, TC3, TC4,
			TC5		// TC6 and TC7 exist but are reserved for the step clock
		};
		static uint16_t tcFreq[ARRAY_SIZE(TcDevices)] = { 0 };
		static uint32_t tcTop[ARRAY_SIZE(TcDevices)] = { 0 };

		if (device < ARRAY_SIZE(TcDevices))
		{
			if (freq == 0)
			{
				tcFreq[device] = freq;
				return false;
			}

			volatile Tc * const tcdev = TcDevices[device];
			if (freq != tcFreq[device])
			{
				const uint32_t prescaler = ChoosePrescaler(freq, 16, tcTop[device]);
				if (output == 0)
				{
					// We need to use CC0 for the compare output, so we can't use it to define TOP. We will get a lower frequency than requested.
					// TODO see if we can use 8-bit mode instead
					tcTop[device] = 0xFFFF;
				}

				const uint32_t cc = ConvertRange(val, tcTop[device]);

				if (tcFreq[device] == 0)
				{
					EnableTcClock(device, GCLK_PCHCTRL_GEN_GCLK1_Val);

					// Initialise the TC
					hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
					hri_tc_set_CTRLA_SWRST_bit(tcdev);
					tcdev->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16_Val;
					if (output == 0)
					{
						tcdev->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_NPWM_Val;
					}
					else
					{
						tcdev->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MPWM_Val;
					}
				}
				else
				{
					hri_tc_clear_CTRLA_ENABLE_bit(tcdev);
				}
				tcdev->COUNT16.CTRLA.bit.PRESCALER = prescaler;
				if (output != 0)
				{
					tcdev->COUNT16.CC[0].bit.CC = tcTop[device];
					tcdev->COUNT16.CCBUF[0].bit.CCBUF = tcTop[device];
				}
				tcdev->COUNT16.CC[output].bit.CC = cc;
				tcdev->COUNT16.CCBUF[output].bit.CCBUF = cc;
				hri_tc_set_CTRLA_ENABLE_bit(tcdev);
				hri_tccount16_write_COUNT_COUNT_bf(tcdev, 0);
				tcFreq[device] = freq;
			}
			else
			{
				// Just update the compare register
				const uint16_t cc = ConvertRange(val, tcTop[device]);
				hri_tccount16_write_CCBUF_CCBUF_bf(tcdev, output, cc);
			}

			SetPinFunction(pin, GpioPinFunction::E);			// TCs are all on peripheral select E
			return true;
		}
		return false;
	}

	// Write PWM to the specified TCC device. 'output' may be 0..5.
	static bool AnalogWriteTcc(Pin pin, unsigned int device, unsigned int output, GpioPinFunction peri, float val, PwmFrequency freq)
	{
		static volatile Tcc* const TccDevices[] =
		{
			TCC0, TCC1, TCC2,
			TCC3, TCC4
		};
		static constexpr unsigned int TccCounterBits[ARRAY_SIZE(TccDevices)] =
		{
			24, 24, 16,
			16, 16
		};
		static uint16_t tccFreq[ARRAY_SIZE(TccDevices)] = { 0 };
		static uint32_t tccTop[ARRAY_SIZE(TccDevices)] = { 0 };

		if (device < ARRAY_SIZE(TccDevices))
		{
			if (freq == 0)
			{
				tccFreq[device] = freq;
				return false;
			}

			volatile Tcc * const tccdev = TccDevices[device];
			if (freq != tccFreq[device])
			{
				const uint32_t prescaler = ChoosePrescaler(freq, TccCounterBits[device], tccTop[device]);
				const uint32_t cc = ConvertRange(val, tccTop[device]);

				if (tccFreq[device] == 0)
				{
					EnableTccClock(device, GCLK_PCHCTRL_GEN_GCLK1_Val);

					// Initialise the TCC
					hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
					hri_tcc_set_CTRLA_SWRST_bit(tccdev);
					tccdev->CTRLA.bit.PRESCALER = prescaler;
					tccdev->CTRLA.bit.RESOLUTION = 0;
					hri_tcc_write_WAVE_WAVEGEN_bf(tccdev, TCC_WAVE_WAVEGEN_NPWM_Val);
				}
				else
				{
					hri_tcc_clear_CTRLA_ENABLE_bit(tccdev);
					hri_tcc_write_CTRLA_PRESCALER_bf(tccdev, prescaler);
				}

				tccdev->PERBUF.bit.PERBUF = tccTop[device];
				tccdev->PER.bit.PER = tccTop[device];
				tccdev->CCBUF[output].bit.CCBUF = cc;
				tccdev->CC[output].bit.CC = cc;
				hri_tcc_set_CTRLA_ENABLE_bit(tccdev);
				hri_tcc_write_COUNT_reg(tccdev, 0);

				tccFreq[device] = freq;
			}
			else
			{
				// Just update the compare register
				const uint32_t cc = ConvertRange(val, tccTop[device]);
				hri_tcc_write_CCBUF_CCBUF_bf(tccdev, output, cc);
			}

			SetPinFunction(pin, peri);
			return true;
		}
		return false;
	}
}

// Initialise this module
void AnalogOut::Init()
{
	// Nothing to do yet
}

// Analog write to DAC, PWM, TC or plain output pin
// Setting the frequency of a TC or PWM pin to zero resets it so that the next call to AnalogOut with a non-zero frequency
// will re-initialise it. The pinMode function relies on this.
void AnalogOut::Write(Pin pin, float val, PwmFrequency freq)
{
	if (pin >= ARRAY_SIZE(PinTable) || std::isnan(val))
	{
		return;
	}

	// if writing 0 or 1, do plain digital output for faster response
	if (val > 0.0 && val < 1.0)
	{
		{
			const TcOutput tc = PinTable[pin].tc;
			if (tc != TcOutput::none)
			{
				if (AnalogWriteTc(pin, GetDeviceNumber(tc), GetOutputNumber(tc), val, freq))
				{
					return;
				}
			}
		}

		{
			const TccOutput tcc = PinTable[pin].tcc;
			if (tcc != TccOutput::none)
			{
				if (AnalogWriteTcc(pin, GetDeviceNumber(tcc), GetOutputNumber(tcc), GetPeriNumber(tcc), val, freq))
				{
					return;
				}
			}
		}
	}

	// Fall back to digital write
	IoPort::SetPinMode(pin, (val < 0.5) ? OUTPUT_LOW : OUTPUT_HIGH);
}

// End
