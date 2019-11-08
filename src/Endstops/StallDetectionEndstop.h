/*
 * StallDetectionEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_
#define SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_

#include "Endstop.h"

// Motor stall detection endstop
class StallDetectionEndstop final : public Endstop
{
public:
	void* operator new(size_t sz) { return Allocate<StallDetectionEndstop>(); }
	void operator delete(void* p) { Release<StallDetectionEndstop>(p); }

	StallDetectionEndstop(uint8_t axis, EndStopPosition pos, bool p_individualMotors);

	EndStopInputType GetEndstopType() const override { return (individualMotors) ? EndStopInputType::motorStallIndividual : EndStopInputType::motorStallAny; }
	EndStopHit Stopped() const override;
	bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) override;
	EndstopHitDetails CheckTriggered(bool goingSlow) override;
	bool Acknowledge(EndstopHitDetails what) override;
	void AppendDetails(const StringRef& str) override;

private:
	DriversBitmap driversMonitored;
	unsigned int numDriversLeft;
	bool individualMotors;
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_ */
