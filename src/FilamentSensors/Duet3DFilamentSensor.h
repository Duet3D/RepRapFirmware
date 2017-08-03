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
	void Poll() override;
	const char *Check(float filamentConsumed) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;
	void Interrupt() override;

private:
	static constexpr float DefaultMmPerRev = 15.0;
	static constexpr float DefaultTolerance = 0.2;

	static constexpr uint16_t SwitchOpenBit = 0x4000u;
	static constexpr uint16_t ErrorBit = 0x8000u;

	float mmPerRev;
	float tolerance;
	bool withSwitch;

	uint16_t sensorValue;

	static constexpr size_t MaxEdgeCaptures = 30;
	volatile size_t numberOfEdgesCaptured;
	uint32_t edgeCaptures[MaxEdgeCaptures];

	enum class RxdState : uint8_t
	{
		waitingForStartBit,
		waitingForNibble,
		errorRecovery
	};

	RxdState state;
	uint32_t startBitLength;
	uint32_t errorRecoveryStartTime;
	size_t bitChangeIndex;
	uint16_t valueBeingAssembled;
	uint8_t nibblesAssembled;
	bool dataReceived;
};

#endif /* SRC_FILAMENTSENSORS_DUET3DFILAMENTSENSOR_H_ */
