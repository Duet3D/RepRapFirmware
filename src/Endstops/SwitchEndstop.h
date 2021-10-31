/*
 * LocalSwitchEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_SWITCHENDSTOP_H_
#define SRC_ENDSTOPS_SWITCHENDSTOP_H_

#include "Endstop.h"

// Switch-type endstop, either on the main board or on a CAN-connected board
class SwitchEndstop : public Endstop
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<SwitchEndstop>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<SwitchEndstop>(p); }
	~SwitchEndstop() override;

	SwitchEndstop(uint8_t p_axis, EndStopPosition pos) noexcept;

	EndStopType GetEndstopType() const noexcept override;
	bool Stopped() const noexcept override;
	bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) noexcept override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;
	void AppendDetails(const StringRef& str) noexcept override;

#if SUPPORT_CAN_EXPANSION
	// Process a remote endstop input change that relates to this endstop. Return true if the buffer has been freed.
	void HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool state) noexcept override;
#endif

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult Configure(const char *pinNames, const StringRef& reply) noexcept;

private:
	typedef Bitmap<uint16_t> PortsBitmap;

	void ReleasePorts() noexcept;

	inline bool IsTriggered(size_t index) const noexcept
	{
#if SUPPORT_CAN_EXPANSION
		return (boardNumbers[index] == CanInterface::GetCanAddress()) ? ports[index].ReadDigital() : states[index];
#else
		return ports[index].ReadDigital();
#endif
	}

#if SUPPORT_CAN_EXPANSION
	static constexpr uint16_t MinimumSwitchReportInterval = 30;
#endif

	IoPort ports[MaxDriversPerAxis];
#if SUPPORT_CAN_EXPANSION
	CanAddress boardNumbers[MaxDriversPerAxis];
	bool states[MaxDriversPerAxis];
#endif
	size_t numPortsUsed;
	PortsBitmap portsLeftToTrigger;
	size_t numPortsLeftToTrigger;
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_SWITCHENDSTOP_H_ */
