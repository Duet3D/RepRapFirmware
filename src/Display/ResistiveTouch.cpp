/*
  Resistive touch screen support
  Based using the approach described in TI app note http://www.ti.com/lit/pdf/sbaa036.
*/

#include "ResistiveTouch.h"

#if SUPPORT_RESISTIVE_TOUCH

#include <Hardware/Spi/SharedSpiDevice.h>

ResistiveTouch::ResistiveTouch(Pin csp, Pin irqp) noexcept
	: spiDev(SharedSpiDevice::GetMainSharedSpiDevice(), SpiFrequency, SpiMode::mode0, csp, false),
	  csPin(csp), irqPin(irqp)
{
	pinMode(csPin, OUTPUT_HIGH);
	pinMode(irqPin, INPUT_PULLUP);
}

void ResistiveTouch::Init(uint16_t xp, uint16_t yp, DisplayOrientation orientationAdjust) noexcept
{
	orientAdjust			= orientationAdjust;
	disp_x_size				= xp;
	disp_y_size				= yp;
	offsetX					= 0;
	scaleX					= (uint16_t)(((uint32_t)(disp_x_size - 1) << 16)/4095);
	offsetY					= 0;
	scaleY					= (uint16_t)(((uint32_t)(disp_y_size - 1) << 16)/4095);

	pressed = false;
}

// If the panel is touched, return the coordinates in x and y and return true; else return false
bool ResistiveTouch::Read(uint16_t &px, uint16_t &py, bool &repeat, uint16_t *null rawX, uint16_t *null rawY) noexcept
{
	bool ret = false;

	repeat = false;

	if (!digitalRead(irqPin))				// if screen is touched
	{
		spiDev.Select();
		delayMicroseconds(100);				// allow the screen to settle
		uint16_t tx;
		if (GetData(false, tx))
		{
			uint16_t ty;
			if (GetData(true, ty))
			{
				if (!digitalRead(irqPin))
				{
					int16_t valx = (orientAdjust & SwapXY) ? ty : tx;
					if (orientAdjust & ReverseX)
					{
						valx = 4095 - valx;
					}

					int16_t cx = (int16_t)(((uint32_t)valx * (uint32_t)scaleX) >> 16) - offsetX;
					px = (cx < 0) ? 0 : (cx >= disp_x_size) ? disp_x_size - 1 : (uint16_t)cx;

					int16_t valy = (orientAdjust & SwapXY) ? tx : ty;
					if (orientAdjust & ReverseY)
					{
						valy = 4095 - valy;
					}

					int16_t cy = (int16_t)(((uint32_t)valy * (uint32_t)scaleY) >> 16) - offsetY;
					py = (cy < 0) ? 0 : (cy >= disp_y_size) ? disp_y_size - 1 : (uint16_t)cy;
					if (rawX != nullptr)
					{
						*rawX = valx;
					}
					if (rawY != nullptr)
					{
						*rawY = valy;
					}
					ret = true;
				}
			}
		}
		spiDev.Deselect();
	}


	if (ret && pressed)
	{
		repeat = true;
	}

	pressed = ret;
	return ret;
}

// Get data from the touch chip. CS has already been set low.
// We need to allow the touch chip ADC input to settle. See TI app note http://www.ti.com/lit/pdf/sbaa036.
bool ResistiveTouch::GetData(bool wantY, uint16_t &rslt) noexcept
{
	uint8_t command = (wantY) ? 0xD3 : 0x93;		// start, channel 5 (y) or 1 (x), 12-bit, differential mode, don't power down between conversions
	WriteCommand(command);							// send the command
	ReadData(command);								// discard the first result and send the same command again

	const size_t NumReadings = 8;
	const uint16_t MaxDiff = 40;					// needs to be big enough to handle jitter.
													// 8 was OK for the 4.3 and 5 inch displays but not the 7 inch.
													// 25 is OK for most 7" displays.
	const unsigned int MaxAttempts = 16;

	uint16_t ring[NumReadings];
	uint32_t sum = 0;

	// Take enough readings to fill the ring buffer
	for (size_t i = 0; i < NumReadings; ++i)
	{
		const uint16_t val = ReadData(command);
		ring[i] = val;
		sum += val;
	}

	// Test whether every reading is within 'maxDiff' of the average reading.
	// If it is, return the average reading.
	// If not, take another reading and try again, up to 'maxAttempts' times.
	uint16_t avg;
	size_t last = 0;
	bool ok;
	for (unsigned int k = 0; k < MaxAttempts; ++k)
	{
		avg = (uint16_t)(sum/NumReadings);
		ok = true;
		for (size_t i = 0; ok && i < NumReadings; ++i)
		{
			if (Diff(avg, ring[i]) > MaxDiff)
			{
				ok = false;
				break;
			}
		}
		if (ok)
		{
			break;
		}

		// Take another reading
		sum -= ring[last];
		uint16_t val = ReadData(command);
		ring[last] = val;
		sum += val;
		last = (last + 1) % NumReadings;
	}

	ReadData(command & 0xF8);			// tell it to power down between conversions
	ReadData(0);						// read the final data
	rslt = avg;
	return ok;
}

// Send the first command in a chain. The chip latches the data bit on the rising edge of the clock. We have already set CS low.
void ResistiveTouch::WriteCommand(uint8_t command) noexcept
{
	spiDev.WritePacket(&command, 1);
}

// Read the data, and write another command at the same time. We have already set CS low.
// The chip produces its data bit after the falling edge of the clock. After sending 8 clocks, we can send a command again.
uint16_t ResistiveTouch::ReadData(uint8_t command) noexcept
{
	uint16_t cmd = (uint16_t)command;
	uint16_t data = 0;
	spiDev.TransceivePacket((const uint8_t *_ecv_array)&cmd, (uint8_t *_ecv_array)&data, 2);
	return data >> 3;
}

void ResistiveTouch::Calibrate(uint16_t xlow, uint16_t xhigh, uint16_t ylow, uint16_t yhigh, uint16_t margin) noexcept
{
	scaleX = (uint16_t)(((uint32_t)(disp_x_size - 1 - 2 * margin) << 16)/(xhigh - xlow));
	offsetX = (int16_t)(((uint32_t)xlow * (uint32_t)scaleX) >> 16) - (int16_t)margin;
	scaleY = (uint16_t)(((uint32_t)(disp_y_size - 1 - 2 * margin) << 16)/(yhigh - ylow));
	offsetY = (int16_t)(((uint32_t)ylow * (uint32_t)scaleY) >> 16) - (int16_t)margin;
}

#endif

// End
