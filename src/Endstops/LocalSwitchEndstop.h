/*
 * LocalSwitchEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_LOCALSWITCHENDSTOP_H_
#define SRC_ENDSTOPS_LOCALSWITCHENDSTOP_H_

#include "Endstop.h"

// Switch-type endstop
class LocalSwitchEndstop : public Endstop
{
public:
	void* operator new(size_t sz) { return Allocate<LocalSwitchEndstop>(); }
	void operator delete(void* p) { Release<LocalSwitchEndstop>(p); }
	~LocalSwitchEndstop() override;

	LocalSwitchEndstop(uint8_t axis, EndStopPosition pos);

	bool Configure(GCodeBuffer& gb, const StringRef& reply, EndStopInputType inputType);
	bool Configure(const char *pinNames, const StringRef& reply, EndStopInputType inputType);
	void Reconfigure(EndStopPosition pos, EndStopInputType inputType);

	EndStopInputType GetEndstopType() const override;
	EndStopHit Stopped() const override;
	void Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;
	void AppendDetails(const StringRef& str) override;

private:
	typedef uint16_t PortsBitmap;

	IoPort ports[MaxDriversPerAxis];
	size_t numPortsUsed;
	PortsBitmap portsLeftToTrigger;
	size_t numPortsLeftToTrigger;
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_LOCALSWITCHENDSTOP_H_ */
