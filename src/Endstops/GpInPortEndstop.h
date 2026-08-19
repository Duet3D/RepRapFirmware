/*
 * GpInPortEndstop.h
 *
 *  Created on: 3 Aug 2026
 *      Author: Christian
 */

#ifndef SRC_ENDSTOPS_GPINPORTENDSTOP_H_
#define SRC_ENDSTOPS_GPINPORTENDSTOP_H_

#include "Endstop.h"

// Endstop fed by general purpose input ports, used to terminate extruder moves when loading or unloading filament
class GpInPortEndstop final : public Endstop
{
public:
	DECLARE_FREELIST_NEW_DELETE(GpInPortEndstop)

	GpInPortEndstop() noexcept;

	EndStopType GetEndstopType() const noexcept override { return EndStopType::inputPin; }
	bool Stopped() const noexcept override;
	void PrimeAxis(const Kinematics &_ecv_from kin, const AxisDriversConfig& axisDrivers, float speed) THROWS(GCodeException) override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;
	void AppendDetails(const StringRef& str) noexcept override;

	void PrimeInputs() noexcept { numInputs = 0; }
	void AddInput(uint8_t gpinNumber, bool stopWhenActive) noexcept;

private:
	struct MonitoredInput
	{
		uint8_t gpinNumber;								// the number of the general purpose input port to monitor
		bool stopWhenActive;							// true to stop the move when the port becomes active, false to stop it when the port becomes inactive
	};

	MonitoredInput inputs[MaxExtruders];
	size_t numInputs = 0;
};

#endif /* SRC_ENDSTOPS_GPINPORTENDSTOP_H_ */
