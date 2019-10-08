/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include "RepRapFirmware.h"

// Class to represent a port
class IoPort
{
public:
	IoPort();
	bool SetMode(PinAccess access);
	void Release();
	void AppendDetails(const StringRef& str) const;

	static size_t AssignPorts(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	void AppendPinName(const StringRef& str) const;
	bool IsValid() const { return logicalPin < NumNamedPins; }
	bool GetInvert() const;
	void SetInvert(bool pInvert);
	void ToggleInvert(bool pInvert);

	bool Read() const;
	bool AttachInterrupt(StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param) const;
	void DetachInterrupt() const;

	uint16_t ReadAnalog() const;
	AnalogChannelNumber GetAnalogChannel() const { return PinToAdcChannel(PinTable[logicalPin].pin); }

	void WriteDigital(bool high) const;

	Pin GetPin() const;

	// Initialise static data
	static void Init();

	static void AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[]);

#if SUPPORT_CAN_EXPANSION
	static CanAddress RemoveBoardAddress(const StringRef& portName);
#endif

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode);
	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

protected:
	bool Allocate(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	static const char* TranslatePinAccess(PinAccess access);

	LogicalPin logicalPin;									// the logical pin number
	uint8_t hardwareInvert : 1,								// true if the hardware includes inversion
			totalInvert : 1,								// true if the value should be inverted when reading/writing the pin
			isSharedInput : 1;								// true if we are using this pin as a shared input

	static PinUsedBy portUsedBy[NumNamedPins];				// the list of what each logical port is used by
	static int8_t logicalPinModes[NumNamedPins];			// what mode each logical pin is set to - would ideally be class PinMode not int8_t
};

static_assert(sizeof(IoPort) == 2, "Unexpected size for class IoPort");		// try to keep these small because triggers have arrays of them

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();

	void AppendDetails(const StringRef& str) const;			// hides the one in IoPort
	void SetFrequency(PwmFrequency freq) { frequency = freq; }
	void WriteAnalog(float pwm) const;

private:
	PwmFrequency frequency;
};

#endif /* SRC_PORT_H_ */
