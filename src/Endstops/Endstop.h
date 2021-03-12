/*
 * Endstop.h
 *
 *  Created on: 4 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOP_H_
#define SRC_ENDSTOPS_ENDSTOP_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include "EndstopDefs.h"
#include <Hardware/IoPorts.h>
#include <General/FreelistManager.h>

#if SUPPORT_TMC22xx && HAS_STALL_DETECT
# include <Movement/StepperDrivers/TMC22xx.h>
#endif

class AxisDriversConfig;
class CanMessageBuffer;

// This is the base class for all types of endstops and for ZProbe.
class EndstopOrZProbe INHERIT_OBJECT_MODEL
{
public:
	EndstopOrZProbe() noexcept : next(nullptr) {}
	EndstopOrZProbe(const EndstopOrZProbe&) = delete;
	virtual ~EndstopOrZProbe() noexcept {}

	virtual bool Stopped() const noexcept = 0;
	virtual EndstopHitDetails CheckTriggered() noexcept = 0;
	virtual bool Acknowledge(EndstopHitDetails what) noexcept = 0;

	EndstopOrZProbe *GetNext() const noexcept { return next; }
	void SetNext(EndstopOrZProbe *e) noexcept { next = e; }

#if HAS_STALL_DETECT && (SUPPORT_TMC2660 || SUPPORT_TMC51xx)
	static void SetDriversStalled(DriversBitmap drivers) noexcept;
	static void SetDriversNotStalled(DriversBitmap drivers) noexcept;
#endif

protected:

#if HAS_STALL_DETECT
	static DriversBitmap GetStalledDrivers(DriversBitmap driversOfInterest) noexcept;
#endif

private:
	EndstopOrZProbe *next;								// next endstop in linked list

#if HAS_STALL_DETECT && (SUPPORT_TMC2660 || SUPPORT_TMC51xx)
	static DriversBitmap stalledDrivers;				// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes
#endif
};

#if HAS_STALL_DETECT

# if SUPPORT_TMC2660 || SUPPORT_TMC51xx

// This is called by the TMC driver to tell us which drivers are stalled or not stalled
inline void EndstopOrZProbe::SetDriversStalled(DriversBitmap drivers) noexcept
{
	stalledDrivers |= drivers;
}

// This is called by the TMC driver to tell us which drivers are stalled or not stalled
inline void EndstopOrZProbe::SetDriversNotStalled(DriversBitmap drivers) noexcept
{
	stalledDrivers &= ~drivers;
}

// Return which drivers out of the set of interest are stalled
inline DriversBitmap EndstopOrZProbe::GetStalledDrivers(DriversBitmap driversOfInterest) noexcept
{
	return stalledDrivers & driversOfInterest;
}

# elif SUPPORT_TMC22xx

// Return which drivers out of the set of interest are stalled
inline DriversBitmap EndstopOrZProbe::GetStalledDrivers(DriversBitmap driversOfInterest) noexcept
{
	return SmartDrivers::GetStalledDrivers(driversOfInterest);
}

# endif
#endif

class Endstop : public EndstopOrZProbe
{
public:
	virtual EndStopType GetEndstopType() const noexcept = 0;
	virtual bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) noexcept = 0;
	virtual void AppendDetails(const StringRef& str) noexcept = 0;

#if SUPPORT_CAN_EXPANSION
	// Process a remote endstop input change that relates to this endstop
	virtual void HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool state) noexcept { }
#endif

	unsigned int GetAxis() const noexcept { return axis; }
	bool GetAtHighEnd() const noexcept { return atHighEnd; }
	void SetAtHighEnd(bool b) noexcept { atHighEnd = b; }

protected:
	Endstop(uint8_t p_axis, EndStopPosition pos) noexcept;

	DECLARE_OBJECT_MODEL

private:
	uint8_t axis;										// which axis this endstop is on
	bool atHighEnd;										// whether this endstop is at the max (true) or the min (false)
};

#endif /* SRC_ENDSTOPS_ENDSTOP_H_ */
