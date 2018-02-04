/*
 * PulsedFilamentSensor.h
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_PULSEDFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_PULSEDFILAMENTMONITOR_H_

#include "FilamentMonitor.h"

class PulsedFilamentMonitor : public FilamentMonitor
{
public:
	PulsedFilamentMonitor(unsigned int extruder, int type);

	bool Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) override;
	FilamentSensorStatus Check(bool full, bool hadNonPrintingMove, bool fromIsr, float filamentConsumed) override;
	FilamentSensorStatus Clear(bool full) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;
	bool Interrupt() override;

private:
	static constexpr float DefaultMmPerPulse = 28.8;
	static constexpr float DefaultTolerance = 0.25;
	static constexpr float DefaultMinimumExtrusionCheckLength = 3.0;

	void Init();
	void Reset();
	void Poll();
	float GetCurrentPosition() const;
	FilamentSensorStatus CheckFilament(float amountCommanded, bool overdue);

	// Configuration parameters
	float mmPerPulse;
	float tolerance;
	float minimumExtrusionCheckLength;

	// Other data
	uint16_t sensorValue;									// last known pulses received (10 bits)
	uint32_t lastMeasurementTime;							// the last time we received a value

	float extrusionCommanded;								// the amount of extrusion commanded since we last did a comparison
	float extrusionCommandedAtLastMeasurement;				// the amount of extrusion commanded up to the start but of the last received measurement
	float movementMeasured;									// the accumulated revs (magnet), position (laser), or pulses since the previous comparison
	float movementMeasuredAtLastCheck;						// the accumulated movement measured before non-printing moves

	// Values measured for calibration
	float minMovementRatio, maxMovementRatio;
	float totalExtrusionCommanded;
	float totalMovementMeasured;

	uint8_t samplesReceived;
	bool dataReceived;
	bool comparisonStarted;
	bool calibrationStarted;
};

#endif /* SRC_FILAMENTSENSORS_PULSEDFILAMENTMONITOR_H_ */
