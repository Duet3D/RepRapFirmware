#include "RotaryEncoder.h"

#if SUPPORT_12864_LCD

#include "Pins.h"
#include "Hardware/IoPorts.h"

RotaryEncoder::RotaryEncoder(Pin p0, Pin p1, Pin pb)
	: pin0(p0), pin1(p1), pinButton(pb), ppc(1), encoderChange(0), encoderState(0), newPress(false), reverseDirection(false) {}

inline unsigned int RotaryEncoder::ReadEncoderState() const
{
	return (digitalRead(pin0) ? 1u : 0u) | (digitalRead(pin1) ? 2u : 0u);
}

void RotaryEncoder::Init(int pulsesPerClick)
{
	ppc = max<unsigned int>(abs(pulsesPerClick), 1);
	reverseDirection = (pulsesPerClick < 0);

	// Set up pins
	IoPort::SetPinMode(pin0, INPUT_PULLUP);
	IoPort::SetPinMode(pin1, INPUT_PULLUP);
	IoPort::SetPinMode(pinButton, INPUT_PULLUP);
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
	// 1 or 2 - the encoder has moved 1 or 2 units clockwise
	// -1 or -2 = the encoder has moved 1 or 2 units anticlockwise
	static const int tbl[16] =
	{
		 0, +1, -1,  0,		// position 3 = 00 to 11, can't really do anything, so 0
		-1,  0, -2, +1,		// position 2 = 01 to 10, assume it was a bounce and should be 01 -> 00 -> 10
		+1, +2,  0, -1,		// position 1 = 10 to 01, assume it was a bounce and should be 10 -> 00 -> 01
		 0, -1, +1,  0		// position 0 = 11 to 10, can't really do anything
	};

	// Poll the encoder
	const unsigned int t = ReadEncoderState();
	const int movement = tbl[(encoderState << 2) | t];
	if (movement != 0)
	{
		encoderChange += movement;
		encoderState = t;
	}

	// Poll the button
	const uint32_t now = millis();
	const bool b = !digitalRead(pinButton);
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
	return (reverseDirection) ? -r : r;
}

bool RotaryEncoder::GetButtonPress()
{
	const bool ret = newPress;
	newPress = false;
	return ret;
}

#endif

// End
