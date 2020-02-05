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
	SimpleFilamentMonitor(unsigned int extruder, unsigned int type) noexcept;

	bool Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) override;
	FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept override;
	FilamentSensorStatus Clear() noexcept override;
	void Diagnostics(MessageType mtype, unsigned int extruder) noexcept override;
	bool Interrupt() noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:
	void Poll() noexcept;

	bool highWhenNoFilament;
	bool filamentPresent;
	bool enabled;
};

#endif /* SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_ */
