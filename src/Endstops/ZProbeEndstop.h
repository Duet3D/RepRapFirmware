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
	DECLARE_FREELIST_NEW_DELETE(ZProbeEndstop)

	ZProbeEndstop(uint8_t p_axis, EndStopPosition pos) noexcept;

	EndStopType GetEndstopType() const noexcept override { return EndStopType::zProbeAsEndstop; }
	bool Stopped() const noexcept override;
	bool Prime(const Kinematics& kin, const AxisDriversConfig& axisDrivers) noexcept override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;
	void AppendDetails(const StringRef& str) noexcept override;

private:
	size_t zProbeNumber;					// which Z probe to use, always 0 for now
	bool stopAll;
};

#endif /* SRC_ENDSTOPS_ZPROBEENDSTOP_H_ */
