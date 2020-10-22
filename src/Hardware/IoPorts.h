/*
 * IoPort.h
 *
 *  Created on: 30 Sep 2017
 *      Author: David
 */

#ifndef SRC_IOPORTS_H_
#define SRC_IOPORTS_H_

#include <RepRapFirmware.h>

#if SAME5x
# include <Interrupts.h>
#endif

// Class to represent a port
class IoPort
{
public:
	IoPort() noexcept;
	~IoPort() { Release(); }

	bool SetMode(PinAccess access) noexcept;
	void Release() noexcept;
	void AppendDetails(const StringRef& str) const noexcept;

	static size_t AssignPorts(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]);
	bool AssignPort(GCodeBuffer& gb, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept;

	static size_t AssignPorts(const char *pinNames, const StringRef& reply, PinUsedBy neededFor, size_t numPorts, IoPort * const ports[], const PinAccess access[]) noexcept;
	bool AssignPort(const char *pinName, const StringRef& reply, PinUsedBy neededFor, PinAccess access) noexcept;

	void AppendPinName(const StringRef& str) const noexcept;
	bool IsValid() const noexcept { return logicalPin < NumNamedPins; }
	bool GetInvert() const noexcept;
	void SetInvert(bool pInvert) noexcept;
	void ToggleInvert(bool pInvert) noexcept;

	bool ReadDigital() const noexcept;
	bool AttachInterrupt(StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param) const noexcept;
	void DetachInterrupt() const noexcept;

	uint16_t ReadAnalog() const noexcept;

	AnalogChannelNumber GetAnalogChannel() const noexcept { return PinToAdcChannel(GetPin()); }

	void WriteDigital(bool high) const noexcept;

	// Get the physical pin, or NoPin if the logical pin is not valid
	Pin GetPin() const noexcept;

	// Initialise static data
	static void Init() noexcept;

	static void AppendPinNames(const StringRef& str, size_t numPorts, IoPort * const ports[]) noexcept;

#if SUPPORT_CAN_EXPANSION
	static CanAddress RemoveBoardAddress(const StringRef& portName) noexcept;
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
#if SAME5x
		// New-style pin table is indexed by pin number
		return logicalPin;
#else
		return PinTable[logicalPin].pin;
#endif
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

	void AppendDetails(const StringRef& str) const noexcept;			// hides the one in IoPort
	void SetFrequency(PwmFrequency freq) noexcept { frequency = freq; }
	void WriteAnalog(float pwm) const noexcept;

private:
	PwmFrequency frequency;
};

#endif /* SRC_PORT_H_ */
