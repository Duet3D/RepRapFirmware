/*
 * Duet3DFilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_DUET3DFILAMENTSENSOR_H_
#define SRC_FILAMENTSENSORS_DUET3DFILAMENTSENSOR_H_

#include "FilamentSensor.h"

class Duet3DFilamentSensor : public FilamentSensor
{
public:
	Duet3DFilamentSensor(int type);

	bool Configure(GCodeBuffer& gb, StringRef& reply, bool& seen) override;
	void Interrupt() override;

private:
	static constexpr float DefaultMmPerRev = 15.0;
	static constexpr float DefaultTolerance = 0.2;

	float mmPerRev;
	float tolerance;
	bool withSwitch;
};

#endif /* SRC_FILAMENTSENSORS_DUET3DFILAMENTSENSOR_H_ */
