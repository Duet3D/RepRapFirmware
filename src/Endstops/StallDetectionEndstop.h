/*
 * StallDetectionEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_
#define SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_

#include "Endstop.h"

#if HAS_STALL_DETECT

// Motor stall detection endstop
class StallDetectionEndstop final : public Endstop
{
public:
	DECLARE_FREELIST_NEW_DELETE(StallDetectionEndstop)

	StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept;		// for creating axis endstops
	StallDetectionEndstop() noexcept;							// for creating the single extruders endstop

	EndStopType GetEndstopType() const noexcept override { return (individualMotors) ? EndStopType::motorStallIndividual : EndStopType::motorStallAny; }
	bool Stopped() const noexcept override;
	bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) noexcept override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;
	void AppendDetails(const StringRef& str) noexcept override;
	bool ShouldReduceAcceleration() const noexcept override { return true; }
	EndstopValidationResult Validate(const DDA& dda, uint8_t& failingDriver) const noexcept override;

	void SetDrivers(DriversBitmap extruderDrivers) noexcept;	// for setting which local extruder drives are active extruder endstops

private:
	DriversBitmap driversMonitored;
	unsigned int numDriversLeft;
	bool individualMotors;
	bool stopAll;
};

#endif

#endif /* SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_ */
