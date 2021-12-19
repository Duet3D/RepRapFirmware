/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include <RepRapFirmware.h>

#include <Interrupts.h>
#include <AnalogIn.h>

// Class to represent a port
class IoPort
{
public:
	IoPort() noexcept;
	~IoPort() { Release(); }

	bool SetMode(PinAccess access) noexcept;
	void Release() noexcept;
	void AppendBasicDetails(const StringRef& str) const noexcept;

	static size_t AssignPorts(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]) THROWS(GCodeException);
	bool AssignPort(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, PinAccess access) THROWS(GCodeException);

	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]) noexcept;
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept;

	void AppendPinName(const StringRef& str) const noexcept;
	bool IsValid() const noexcept { return logicalPin < NumNamedPins; }
	bool GetInvert() const noexcept;
	void SetInvert(bool pInvert) noexcept;
	void ToggleInvert(bool pInvert) noexcept;
	bool IsHardwareInverted() const noexcept { return hardwareInvert; }

	bool ReadDigital() const noexcept;
	bool AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const noexcept;
	void DetachInterrupt() const noexcept;
#if SAME5x
	bool SetAnalogCallback(AnalogInCallbackFunction fn, CallbackParameter cbp, uint32_t ticksPerCall) noexcept;
#endif

	uint16_t ReadAnalog() const noexcept;

	AnalogChannelNumber GetAnalogChannel() const noexcept { return PinToAdcChannel(GetPin()); }

	void WriteDigital(bool high) const noexcept;

	// Get the physical pin, or NoPin if the logical pin is not valid
	Pin GetPin() const noexcept;

	// Initialise static data
	static void Init() noexcept;

	static void AppendPinNames(const StringRef& str, size_t numPorts, const IoPort * const ports[]) noexcept;

#if SUPPORT_CAN_EXPANSION
	// Remove the board address if present and return it, else return the default address
	static CanAddress RemoveBoardAddress(const StringRef& portName) noexcept;
#else
	// Remove the board address if present, returning true if it was zero or not present
	static bool RemoveBoardAddress(const StringRef& portName) noexcept;
#endif

	// Low level port access
	static void SetPinMode(Pin p, PinMode mode) noexcept;
	static bool ReadPin(Pin p) noexcept;
	static void WriteDigital(Pin p, bool high) noexcept;
	static void WriteAnalog(Pin p, float pwm, uint16_t frequency) noexcept;

protected:
	bool Allocate(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept;

	// Get the physical pin without checking the validity of the logical pin
	Pin GetPinNoCheck() const noexcept
	{
		// New-style pin table is indexed by pin number
		return logicalPin;
	}

	static const char* TranslatePinAccess(PinAccess access) noexcept;

	LogicalPin logicalPin;									// the logical pin number
	uint8_t hardwareInvert : 1,								// true if the hardware includes inversion
			totalInvert : 1,								// true if the value should be inverted when reading/writing the pin
			isSharedInput : 1,								// true if we are using this pin as a shared input
			alternateConfig : 1;							// true if we are using the alternate configuration of this pin, e.g. SDADC instyead of ADC

	static PinUsedBy portUsedBy[NumNamedPins];				// the list of what each logical port is used by
	static int8_t logicalPinModes[NumNamedPins];			// what mode each logical pin is set to - would ideally be class PinMode not int8_t
};

static_assert(sizeof(IoPort) == 2, "Unexpected size for class IoPort");		// try to keep these small because triggers have arrays of them

// Class to represent a PWM output port
class PwmPort : public IoPort
{
public:
	PwmPort() noexcept;

	void AppendFullDetails(const StringRef& str) const noexcept;
	void AppendFrequency(const StringRef& str) const noexcept;		// append the frequency if the port is valid
	void SetFrequency(PwmFrequency freq) noexcept { frequency = freq; }
	PwmFrequency GetFrequency() const noexcept { return frequency; }
	void WriteAnalog(float pwm) const noexcept;

private:
	PwmFrequency frequency;
};

#endif /* SRC_PORT_H_ */
