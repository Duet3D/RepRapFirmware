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

class AxisDriversConfig;
class CanMessageBuffer;

// This is the base class for all types of endstops and for ZProbe.
class EndstopOrZProbe INHERIT_OBJECT_MODEL
{
public:
	EndstopOrZProbe() noexcept : next(nullptr) {}
	virtual ~EndstopOrZProbe() noexcept {}

	virtual EndStopHit Stopped() const noexcept = 0;
	virtual EndstopHitDetails CheckTriggered(bool goingSlow) noexcept = 0;
	virtual bool Acknowledge(EndstopHitDetails what) noexcept = 0;

	EndstopOrZProbe *GetNext() const noexcept { return next; }
	void SetNext(EndstopOrZProbe *e) noexcept { next = e; }

	static void UpdateStalledDrivers(DriversBitmap drivers, bool isStalled) noexcept;

protected:
	static DriversBitmap GetStalledDrivers() noexcept { return stalledDrivers; }

private:
	EndstopOrZProbe *next;								// next endstop in linked list

	static DriversBitmap stalledDrivers;				// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes
};

inline void EndstopOrZProbe::UpdateStalledDrivers(DriversBitmap drivers, bool isStalled) noexcept
{
	if (isStalled)
	{
		stalledDrivers |= drivers;
	}
	else
	{
		stalledDrivers &= ~drivers;
	}
}

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
	Endstop(uint8_t axis, EndStopPosition pos) noexcept;

	DECLARE_OBJECT_MODEL

private:
	uint8_t axis;										// which axis this endstop is on
	bool atHighEnd;										// whether this endstop is at the max (true) or the min (false)
};

#endif /* SRC_ENDSTOPS_ENDSTOP_H_ */
