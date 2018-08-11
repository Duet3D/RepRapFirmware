/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include "RepRapFirmware.h"

// Enumeration to describe what we want to do with a logical pin
enum class PinAccess : int
{
	read,
	write,
	pwm,
	servo
};

// Class to represent a port
class IoPort
{
public:
	IoPort();
	void Clear();
	bool Set(LogicalPin lp, PinAccess access, bool pInvert);

	LogicalPin GetLogicalPin() const { return logicalPort; }
	LogicalPin GetLogicalPin(bool& pInvert) const { pInvert = invert; return logicalPort; }
	void WriteDigital(bool high) const { if (pin != NoPin) { WriteDigital(pin, (invert) ? !high : high); } }

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode);
	static bool ReadPin(Pin p);
	static void WriteDigital(Pin p, bool high);
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency);

protected:
	LogicalPin logicalPort;
	Pin pin;
	bool invert;
};

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort();
	void SetFrequency(float freq);
	float GetFrequency() const { return (float)frequency; }
	void WriteAnalog(float pwm) const;

private:
	uint16_t frequency;
};

#endif /* SRC_PORT_H_ */
