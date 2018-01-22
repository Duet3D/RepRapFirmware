#include "RepRapFirmware.h"
#include "Pins.h"

#if SUPPORT_12864_LCD

#include <Display/RotaryEncoder.h>

RotaryEncoder::RotaryEncoder(Pin p0, Pin p1, Pin pb, int pulsesPerClick)
	: pin0(p0), pin1(p1), pinButton(pb), ppc(pulsesPerClick), encoderChange(0), encoderState(0), newPress(false) {}

inline unsigned int RotaryEncoder::ReadEncoderState() const
{
	return (digitalRead(pin0) ? 1u : 0u) | (digitalRead(pin1) ? 2u : 0u);
}

void RotaryEncoder::Init()
{
	// Set up pins
	pinMode(pin0, INPUT_PULLUP);
	pinMode(pin1, INPUT_PULLUP);
	pinMode(pinButton, INPUT_PULLUP);
	delay(2);                  // ensure we read the initial state correctly

	// Initialise encoder variables
	encoderChange = 0;
	encoderState = ReadEncoderState();

	// Initialise button variables
	buttonState = !digitalRead(pinButton);
	whenSame = millis();
	newPress = false;
}

void RotaryEncoder::Poll()
{
	// State transition table. Each entry has the following meaning:
	// 0 - the encoder hasn't moved
	// 1 - the encoder has moved 1 unit clockwise
	// -1 = the encoder has moved 1 unit anticlockwise
	// 2 = illegal transition, we must have missed a state
	static const int tbl[16] =
	{
		 0, +1, -1,  0,		// position 3 = 00 to 11, can't really do anything, so 0
		-1,  0, -2, +1,		// position 2 = 01 to 10, assume it was a bounce and should be 01 -> 00 -> 10
		+1, +2,  0, -1,		// position 1 = 10 to 01, assume it was a bounce and should be 10 -> 00 -> 01
		 0, -1, +1,  0		// position 0 = 11 to 10, can't really do anything
	};

	// Poll the encoder
	const unsigned int t = ReadEncoderState();
	int movement = tbl[(encoderState << 2) | t];
	if (movement != 0)
	{
		encoderChange += movement;
		encoderState = t;
	}

	// Poll the button
	const uint32_t now = millis();
	bool b = !digitalRead(pinButton);
	if (b == buttonState)
	{
		whenSame = now;
	}
	else if (now - whenSame > DebounceMillis)
	{
		buttonState = b;
		whenSame = now;
		if (buttonState)
		{
			newPress = true;
		}
	}
}

int RotaryEncoder::GetChange()
{
	int r;
	noInterrupts();
	if (encoderChange >= ppc - 1)
	{
		r = (encoderChange + 1)/ppc;
	}
	else if (encoderChange <= 1 - ppc)
	{
		r = -((1 - encoderChange)/ppc);
	}
	else
	{
		r = 0;
	}
	encoderChange -= (r * ppc);
	interrupts();
	return r;
}

#endif

// End
