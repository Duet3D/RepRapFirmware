/*
 * RotatingMagnetFilamentMonitor.h
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_ROTATINGMAGNETFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_ROTATINGMAGNETFILAMENTMONITOR_H_

#include "Duet3DFilamentMonitor.h"

class RotatingMagnetFilamentMonitor : public Duet3DFilamentMonitor
{
public:
	RotatingMagnetFilamentMonitor(int type);

	bool Configure(GCodeBuffer& gb, StringRef& reply, bool& seen) override;
	FilamentSensorStatus Check(bool full, bool hadNonPrintingMove, float filamentConsumed) override;
	FilamentSensorStatus Clear(bool full) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;

protected:
	void OnStartBitReceived();
	void ProcessReceivedWord(uint16_t val);

private:
	static constexpr float DefaultMmPerRev = 28.8;
	static constexpr float DefaultTolerance = 0.25;
	static constexpr float DefaultMinimumExtrusionCheckLength = 3.0;

	static constexpr uint16_t TypeMagnetErrorMask = 0x8000u;
	static constexpr uint16_t TypeMagnetSwitchOpenMask = 0x4000u;
	static constexpr uint16_t TypeMagnetAngleMask = 0x03FF;			// we use a 10-bit sensor angle

	void Init();
	void Reset();
	float GetCurrentPosition() const;
	FilamentSensorStatus CheckFilament(float amountCommanded, float amountMeasured, bool overdue);

	// Configuration parameters
	float mmPerRev;
	float tolerance;
	float minimumExtrusionCheckLength;
	bool checkNonPrintingMoves;

	// Other data
	uint16_t sensorValue;									// last known filament position (10 bits)
	uint32_t lastMeasurementTime;							// the last time we received a value
	uint16_t switchOpenMask;								// mask to isolate the switch open bit(s) from the sensor value

	float extrusionCommanded;								// the amount of extrusion commanded since we last did a comparison
	float extrusionCommandedAtStartBit;						// the amount of extrusion commanded since the previous comparison when we received the start bit
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

#endif /* SRC_FILAMENTSENSORS_ROTATINGMAGNETFILAMENTMONITOR_H_ */
