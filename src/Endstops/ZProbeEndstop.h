/*
 * ZProbeEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ZPROBEENDSTOP_H_
#define SRC_ENDSTOPS_ZPROBEENDSTOP_H_

#include "Endstop.h"

class ZProbeEndstop final : public Endstop
{
public:
	void* operator new(size_t sz) { return Allocate<ZProbeEndstop>(); }
	void operator delete(void* p) { Release<ZProbeEndstop>(p); }

	ZProbeEndstop(uint8_t axis, EndStopPosition pos);

	EndStopInputType GetEndstopType() const override { return EndStopInputType::zProbeAsEndstop; }
	EndStopHit Stopped() const override;
	bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;
	void AppendDetails(const StringRef& str) override;

private:
	size_t zProbeNumber;					// which Z probe to use, always 0 for now
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_ZPROBEENDSTOP_H_ */
