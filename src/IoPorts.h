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
	void AppendDetails(const StringRef& str);

	static size_t AssignPorts(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	void AppendPinName(const StringRef& str) const;
	bool IsValid() const { return logicalPin < NumLogicalPins; }
	bool GetInvert() const;
	void SetInvert(bool pInvert);
	void ToggleInvert(bool pInvert);

	void WriteDigital(bool high) const;
	bool Read() const;
	uint16_t ReadAnalog() const;
	bool AttachInterrupt(StandardCallbackFunction callback, enum InterruptMode mode, CallbackParameter param) const;
	void DetachInterrupt() const;

	// Initialise static data
	static void Init();

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode);
	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

protected:
	bool Allocate(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access);

	static const char* TranslatePinAccess(PinAccess access);

	LogicalPin logicalPin;									// the logical pin number
	AnalogChannelNumber analogChannel;						// the analog channel number if it is an analog input, or -1 if not
	bool hardwareInvert;									// whether the hardware includes inversion
	bool totalInvert;										// whether the input or output should be inverted

	static PinUsedBy portUsedBy[NumLogicalPins];			// the list of what each logical port is used by
	static int8_t logicalPinModes[NumLogicalPins];			// what mode each logical pin is set to - would ideally be class PinMode not int8_t
};

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();

	void AppendDetails(const StringRef& str);

	void SetFrequency(PwmFrequency freq) { frequency = freq; }
	void WriteAnalog(float pwm) const;

private:
	PwmFrequency frequency;
};

#endif /* SRC_PORT_H_ */
