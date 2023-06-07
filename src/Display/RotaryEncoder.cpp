#include "RotaryEncoder.h"

#if SUPPORT_ROTARY_ENCODER

#include <Hardware/IoPorts.h>

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)		OBJECT_MODEL_FUNC_BODY(RotaryEncoder, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(...)	OBJECT_MODEL_FUNC_IF_BODY(RotaryEncoder, __VA_ARGS__)

constexpr ObjectModelTableEntry RotaryEncoder::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. RotaryEncoder members
	{ "pulsesPerClick",			OBJECT_MODEL_FUNC((int32_t)self->ppc), 	ObjectModelEntryFlags::none },
};

constexpr uint8_t RotaryEncoder::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE(RotaryEncoder)

RotaryEncoder::RotaryEncoder(Pin p0, Pin p1, Pin pb) noexcept
	: pin0(p0), pin1(p1), pinButton(pb), ppc(1), encoderChange(0), encoderState(0), newPress(false), reverseDirection(false) {}

inline unsigned int RotaryEncoder::ReadEncoderState() const noexcept
{
	return (digitalRead(pin0) ? 1u : 0u) | (digitalRead(pin1) ? 2u : 0u);
}

void RotaryEncoder::Init(int pulsesPerClick) noexcept
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

void RotaryEncoder::Poll() noexcept
{
	// State transition table. Each entry has the following meaning:
	// 0 - the encoder hasn't moved
	// 1 or 2 - the encoder has moved 1 or 2 units clockwise
	// -1 or -2 = the encoder has moved 1 or 2 units anticlockwise
	static const int8_t tbl[16] =
	{
		 0, +1, -1,  0,		// position 3 = 00 to 11, can't really do anything, so 0
		-1,  0,  0, +1,		// position 2 = 01 to 10, can't really do anything, so 0
		+1,  0,  0, -1,		// position 1 = 10 to 01, can't really do anything, so 0
		 0, -1, +1,  0		// position 0 = 11 to 00, can't really do anything, so 0
	};

	// Poll the encoder
	const unsigned int t = ReadEncoderState();
	const int8_t movement = tbl[(encoderState << 2) | t];
	if (movement != 0)
	{
		encoderChange += (int)movement;
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

int RotaryEncoder::GetChange() noexcept
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

bool RotaryEncoder::GetButtonPress() noexcept
{
	const bool ret = newPress;
	newPress = false;
	return ret;
}

#endif

// End
