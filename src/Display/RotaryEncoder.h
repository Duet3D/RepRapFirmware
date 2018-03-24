#ifndef __RotaryEncoderIncluded
#define __RotaryEncoderIncluded

#include "RepRapFirmware.h"

// Class to manage a rotary encoder with a push button
class RotaryEncoder
{
	const Pin pin0, pin1, pinButton;
	int ppc;
	int encoderChange;
	unsigned int encoderState;
	bool buttonState;
	bool newPress;
	bool reverseDirection;
	uint32_t whenSame;

	unsigned int ReadEncoderState() const;

	static constexpr uint32_t DebounceMillis = 5;

public:
	RotaryEncoder(Pin p0, Pin p1, Pin pb);

	void Init(int pulsesPerClick);
	void Poll();
	int GetChange();
	bool GetButtonPress();
};

#endif
