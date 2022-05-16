#ifndef __RotaryEncoderIncluded
#define __RotaryEncoderIncluded

#include "RepRapFirmware.h"

#if SUPPORT_ROTARY_ENCODER

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

	unsigned int ReadEncoderState() const noexcept;

	static constexpr uint32_t DebounceMillis = 5;

public:
	RotaryEncoder(Pin p0, Pin p1, Pin pb) noexcept;

	void Init(int pulsesPerClick) noexcept;
	void Poll() noexcept;
	int GetChange() noexcept;
	bool GetButtonPress() noexcept;
	int GetPulsesPerClick() const noexcept { return ppc; }
};

#endif

#endif
