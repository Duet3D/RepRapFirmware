/*
 * GpioPorts.h
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#ifndef SRC_GPIO_GPIOPORTS_H_
#define SRC_GPIO_GPIOPORTS_H_

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <GCodes/GCodeResult.h>
#include <ObjectModel/ObjectModel.h>

#if SUPPORT_CAN_EXPANSION
# include <RemoteInputHandle.h>
#endif

class GpInputPort INHERIT_OBJECT_MODEL
{
public:
	GpInputPort() noexcept :
#if SUPPORT_CAN_EXPANSION
		boardAddress (CanId::MasterAddress),
#endif
		currentState(false) { }

	bool GetState() const noexcept;
	bool IsUnused() const noexcept;

#if SUPPORT_CAN_EXPANSION
	void SetState(CanAddress src, bool b) noexcept { if (src == boardAddress) { currentState = b; } }
#endif

	GCodeResult Configure(uint32_t gpinNumber, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

protected:
	DECLARE_OBJECT_MODEL

private:
	IoPort port;									// will be initialised by PwmPort default constructor
#if SUPPORT_CAN_EXPANSION
	RemoteInputHandle handle;
	CanAddress boardAddress;
#endif
	bool currentState;
};

class GpOutputPort
{
public:
#if SUPPORT_CAN_EXPANSION
	GpOutputPort() noexcept { boardAddress = CanId::MasterAddress; }
#endif

	GCodeResult WriteAnalog(uint32_t gpioPortNumber, bool isServo, float pwm, const GCodeBuffer& gb, const StringRef& reply) const noexcept;
	GCodeResult Configure(uint32_t gpioNumber, bool isServo, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#ifdef PCCB
	void Assign(const char *pinName) noexcept;
#endif

private:
	PwmPort port;									// will be initialised by PwmPort default constructor
#if SUPPORT_CAN_EXPANSION
	CanAddress boardAddress;
#endif
};

#endif /* SRC_GPIO_GPIOPORTS_H_ */
