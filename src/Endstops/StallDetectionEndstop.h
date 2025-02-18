/*
 * StallDetectionEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_
#define SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_

#include "Endstop.h"

#if SUPPORT_CAN_EXPANSION
# include <General/Vector.h>
#endif

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION

// Motor stall detection endstop
class StallDetectionEndstop final : public Endstop
{
public:
	DECLARE_FREELIST_NEW_DELETE(StallDetectionEndstop)

	StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept;		// for creating axis endstops
	StallDetectionEndstop() noexcept;																	// for creating the single extruders endstop

	EndStopType GetEndstopType() const noexcept override { return (individualMotors) ? EndStopType::motorStallIndividual : EndStopType::motorStallAny; }
	bool Stopped() const noexcept override;
	void PrimeAxis(const Kinematics &_ecv_from kin, const AxisDriversConfig& axisDrivers, float speed) THROWS(GCodeException) override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;
	void AppendDetails(const StringRef& str) noexcept override;
	bool ShouldReduceAcceleration() const noexcept override { return true; }

	void SetDrivers(LocalDriversBitmap extruderDrivers) noexcept;										// for setting which local extruder drives are active extruder endstops
	void PrimeExtruders(ExtrudersBitmap extruders, const float speeds[MaxExtruders]) THROWS(GCodeException);

#if SUPPORT_CAN_EXPANSION
	void HandleStalledRemoteDrivers(CanAddress boardAddress, RemoteDriversBitmap driversReportedStalled) noexcept override;	// record any stalled remote drivers that are meant for us
	void DeleteRemoteStallEndstops() noexcept override;
#endif

private:
	EndstopHitDetails GetResult(
#if SUPPORT_CAN_EXPANSION
								CanAddress boardAddress,
#endif
								unsigned int driverWithinBoard) noexcept;
	void AddDriverToMonitoredList(DriverId did, float speed) THROWS(GCodeException);

	LocalDriversBitmap localDriversMonitored;
#if SUPPORT_CAN_EXPANSION
	static constexpr size_t MaxRemoteDrivers = max<size_t>(MaxDriversPerAxis, MaxExtrudersPerTool);		// the maximum number of drivers that we may have to monitor
	struct RemoteDriversMonitored																		// struct to represent a remote board and the drivers on it that we are interested in
	{
		CanAddress boardId;
		RemoteDriversBitmap driversMonitored;
		RemoteDriversBitmap driversStalled;

		RemoteDriversMonitored(CanAddress p_boardId, RemoteDriversBitmap p_driversMonitored) noexcept
			: boardId(p_boardId), driversMonitored(p_driversMonitored) { }								// driverStalled will be cleared by its default constructor

		RemoteDriversMonitored() { }
	};
	Vector<RemoteDriversMonitored, MaxRemoteDrivers> remoteDriversMonitored;							// list of relevant remote boards and the drivers we monitor on them
	std::atomic<bool> newStallReported;																	// if this is true then a new remote stall may have been reported since we last reset it
#endif
	unsigned int numDriversLeft;
	bool individualMotors;
	bool stopAll;
};

#endif

#endif /* SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_ */
