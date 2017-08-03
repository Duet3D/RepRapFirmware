/*
 * SimpleFilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_SIMPLEFILAMENTSENSOR_H_
#define SRC_FILAMENTSENSORS_SIMPLEFILAMENTSENSOR_H_

#include <FilamentSensors/FilamentSensor.h>

class SimpleFilamentSensor : public FilamentSensor
{
public:
	SimpleFilamentSensor(int type);

	bool Configure(GCodeBuffer& gb, StringRef& reply, bool& seen) override;
	void Poll() override;
	const char *Check(float filamentConsumed) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;
	void Interrupt() override;

private:
	bool highWhenNoFilament;
	bool filamentPresent;
};

#endif /* SRC_FILAMENTSENSORS_SIMPLEFILAMENTSENSOR_H_ */
