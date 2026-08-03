/*
 * GpInPort.h
 *
 *  Created on: 11 Feb 2020
 *      Author: David
 */

#ifndef SRC_GPIO_GPINPORT_H_
#define SRC_GPIO_GPINPORT_H_

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <ObjectModel/ObjectModel.h>

#if SUPPORT_CAN_EXPANSION
# include <RemoteInputHandle.h>
#endif

class GpInputPort INHERIT_OBJECT_MODEL
{
public:
	GpInputPort() noexcept :
#if SUPPORT_CAN_EXPANSION
		boardAddress(CanInterface::GetCanAddress()),
#endif
		source(InputSource::physicalPort), fmExtruder(0), fmInvert(false), currentState(false) { }
	GpInputPort(const GpInputPort&) = delete;

	bool GetState() const noexcept;
	bool IsUnused() const noexcept;

#if SUPPORT_CAN_EXPANSION
	void SetState(CanAddress src, bool b) noexcept { if (src == boardAddress) { currentState = b; } }
#endif

	GCodeResult Configure(uint32_t gpinNumber, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

protected:
	DECLARE_OBJECT_MODEL

private:
	enum class InputSource : uint8_t
	{
		physicalPort = 0,							// the state comes from a local or remote input pin
		fmSwitch,									// the state comes from the filament present indication of a filament monitor
		fmMotion									// the state comes from the motion detection of a filament monitor
	};

	IoPort port;									// will be initialised by PwmPort default constructor
#if SUPPORT_CAN_EXPANSION
	RemoteInputHandle handle;
	CanAddress boardAddress;
#endif
	InputSource source;
	uint8_t fmExtruder;								// the extruder number of the filament monitor if the source is a filament monitor
	bool fmInvert;									// whether to invert the state if the source is a filament monitor
	bool currentState;
};

#endif /* SRC_GPIO_GPINPORT_H_ */
