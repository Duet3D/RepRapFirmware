/*
 * Endstop.h
 *
 *  Created on: 4 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOP_H_
#define SRC_ENDSTOPS_ENDSTOP_H_

#include "RepRapFirmware.h"
#include "EndstopDefs.h"
#include "IoPorts.h"
#include <General/FreelistManager.h>

class AxisDriversConfig;

// This is the base class for all types of endstops and for ZProbe.
class EndstopOrZProbe
{
public:
	EndstopOrZProbe() : next(nullptr) {}
	virtual ~EndstopOrZProbe() {}

	virtual EndStopHit Stopped() const = 0;
	virtual EndstopHitDetails CheckTriggered(bool goingSlow) = 0;
	virtual bool Acknowledge(EndstopHitDetails what) = 0;

	EndstopOrZProbe *GetNext() const { return next; }
	void SetNext(EndstopOrZProbe *e) { next = e; }

	static void UpdateStalledDrivers(uint32_t driverMask, bool isStalled);

protected:
	static DriversBitmap GetStalledDrivers() { return stalledDrivers; }

private:
	EndstopOrZProbe *next;								// next endstop in linked list

	static DriversBitmap stalledDrivers;				// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes
};

inline void EndstopOrZProbe::UpdateStalledDrivers(uint32_t driverMask, bool isStalled)
{
	if (isStalled)
	{
		stalledDrivers |= driverMask;
	}
	else
	{
		stalledDrivers &= ~driverMask;
	}
}

class Endstop : public EndstopOrZProbe
{
public:
	virtual EndStopInputType GetEndstopType() const = 0;
	virtual void Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) = 0;
	virtual void AppendPinNames(const StringRef& str) { }

	unsigned int GetAxis() const { return axis; }
	bool GetAtHighEnd() const { return atHighEnd; }

protected:
	Endstop(uint8_t axis, EndStopPosition pos);

	void SetAtHighEnd(bool b) { atHighEnd = b; }

private:
	uint8_t axis;										// which axis this endstop is on
	bool atHighEnd;										// whether this endstop is at the max (true) or the min (false)
};

// Switch-type endstop
class SwitchEndstop final : public Endstop
{
public:
	void* operator new(size_t sz) { return Allocate<SwitchEndstop>(); }
	void operator delete(void* p) { Release<SwitchEndstop>(p); }
	~SwitchEndstop() override;

	SwitchEndstop(uint8_t axis, EndStopPosition pos);

	bool Configure(GCodeBuffer& gb, const StringRef& reply, EndStopInputType inputType);
	bool Configure(const char *pinNames, const StringRef& reply, EndStopInputType inputType);
	void Reconfigure(EndStopPosition pos, EndStopInputType inputType);

	EndStopInputType GetEndstopType() const override;
	EndStopHit Stopped() const override;
	void Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;
	void AppendPinNames(const StringRef& str) override;

private:
	typedef uint16_t PortsBitmap;

	IoPort ports[MaxDriversPerAxis];
	size_t numPortsUsed;
	PortsBitmap portsLeftToTrigger;
	size_t numPortsLeftToTrigger;
	bool stopAll;
};

// Motor stall detection endstop
class StallDetectionEndstop final : public Endstop
{
public:
	void* operator new(size_t sz) { return Allocate<StallDetectionEndstop>(); }
	void operator delete(void* p) { Release<StallDetectionEndstop>(p); }

	StallDetectionEndstop(uint8_t axis, EndStopPosition pos, bool p_individualMotors);

	EndStopInputType GetEndstopType() const override { return (individualMotors) ? EndStopInputType::motorStallIndividual : EndStopInputType::motorStallAny; }
	EndStopHit Stopped() const override;
	void Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;

private:
	DriversBitmap driversMonitored;
	unsigned int numDriversLeft;
	bool individualMotors;
	bool stopAll;
};

class ZProbeEndstop final : public Endstop
{
public:
	void* operator new(size_t sz) { return Allocate<ZProbeEndstop>(); }
	void operator delete(void* p) { Release<ZProbeEndstop>(p); }

	ZProbeEndstop(uint8_t axis, EndStopPosition pos);

	EndStopInputType GetEndstopType() const override { return EndStopInputType::zProbeAsEndstop; }
	EndStopHit Stopped() const override;
	void Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;

private:
	size_t zProbeNumber;					// which Z probe to use, always 0 for now
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_ENDSTOP_H_ */
