/*
 * SimpleFilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_

#include "FilamentMonitor.h"

class SimpleFilamentMonitor : public FilamentMonitor
{
public:
	SimpleFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept;

protected:
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) override;
#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
	void GetLiveData(FilamentMonitorDataNew& data) const noexcept override;
	void Diagnostics(const StringRef& reply) noexcept override;
#endif
	FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept override;
	FilamentSensorStatus Clear() noexcept override;

#if SUPPORT_CAN_EXPANSION
	void UpdateLiveData(const FilamentMonitorDataNew& data) noexcept override;
#endif

	void Diagnostics(MessageType mtype, unsigned int extruder) noexcept override;
	bool Interrupt() noexcept override;
	const char *_ecv_array GetTypeText() const noexcept override { return "simple"; }

private:
	void Poll() noexcept;

	bool highWhenNoFilament;
	bool filamentPresent;
};

#endif /* SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_ */
