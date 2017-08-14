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
	FilamentSensorStatus Check(bool full, float filamentConsumed) override;
	FilamentSensorStatus Clear(bool full) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;
	void Interrupt() override;

private:
	static constexpr float DefaultMmPerRev = 28.8;
	static constexpr float DefaultTolerance = 0.2;
	static constexpr float DefaultMinimumExtrusionCheckLength = 3.0;

	static constexpr uint16_t SwitchOpenBit = 0x4000u;
	static constexpr uint16_t ErrorBit = 0x8000u;
	static constexpr uint16_t AngleMask = 0x03FF;			// 10-bit sensor angle

	static constexpr size_t EdgeCaptureBufferSize = 64;		// must be a power of 2

	void Init();
	void Poll();
	float GetCurrentAngle() const;
	FilamentSensorStatus CheckFilament(float amountCommanded, bool overdue);

	// Configuration parameters
	float mmPerRev;
	float tolerance;
	float minimumExtrusionCheckLength;
	bool withSwitch;

	// Other data
	uint16_t sensorValue;
	float accumulatedExtrusionCommanded;
	float accumulatedRevsMeasured;
	float extrusionCommandedAtLastMeasurement;
	float tentativeExtrusionCommanded;

	uint32_t lastMeasurementTime;
	uint32_t edgeCaptures[EdgeCaptureBufferSize];
	size_t edgeCaptureReadPointer;
	volatile size_t edgeCaptureWritePointer;
	float minMovementRatio, maxMovementRatio;
	float totalExtrusionCommanded, totalRevsMeasured;

	enum class RxdState : uint8_t
	{
		waitingForStartBit,
		waitingForEndOfStartBit,
		waitingForNibble,
		errorRecovery1,
		errorRecovery2,
		errorRecovery3,
		errorRecovery4
	};

	RxdState state;
	uint32_t startBitLength;
	uint32_t errorRecoveryStartTime;
	size_t lastBitChangeIndex;
	uint16_t valueBeingAssembled;
	uint8_t nibblesAssembled;
	uint8_t samplesReceived;
	bool dataReceived;
	bool comparisonStarted;
	bool calibrationStarted;
};

#endif /* SRC_FILAMENTSENSORS_DUET3DFILAMENTSENSOR_H_ */
