/*
 * LaserFilamentMonitor.h
 *
 *  Created on: 9 Jan 2018
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_LASERFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_LASERFILAMENTMONITOR_H_

#include "Duet3DFilamentMonitor.h"

class LaserFilamentMonitor : public Duet3DFilamentMonitor
{
public:
	LaserFilamentMonitor(int type);

	bool Configure(GCodeBuffer& gb, StringRef& reply, bool& seen) override;
	FilamentSensorStatus Check(bool full, bool hadNonPrintingMove, float filamentConsumed) override;
	FilamentSensorStatus Clear(bool full) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;

protected:
	void OnStartBitReceived();
	void ProcessReceivedWord(uint16_t val);

private:
	static constexpr float DefaultMinMovementAllowed = 0.6;
	static constexpr float DefaultMaxMovementAllowed = 1.6;
	static constexpr float DefaultMinimumExtrusionCheckLength = 3.0;

	static constexpr uint16_t TypeLaserParityMask = 0x8000u;
	static constexpr uint16_t TypeLaserQualityMask = 0x4000u;
	static constexpr uint16_t TypeLaserErrorMask = 0x2000u;
	static constexpr uint16_t TypeLaserSwitchOpenMask = 0x1000u;
	static constexpr uint16_t TypeLaserPositionMask = 0x03FF;		// we use a 10-bit sensor position

	static constexpr size_t EdgeCaptureBufferSize = 64;				// must be a power of 2

	void Init();
	void Reset();
	float GetCurrentPosition() const;
	FilamentSensorStatus CheckFilament(float amountCommanded, float amountMeasured, bool overdue);

	// Configuration parameters
	float minMovementAllowed, maxMovementAllowed;
	float minimumExtrusionCheckLength;

	// Other data
	uint16_t sensorValue;									// last known filament position (10 bits)
	uint16_t qualityWord;									// last received quality data
	uint32_t lastMeasurementTime;							// the last time we received a value
	uint16_t switchOpenMask;								// mask to isolate the switch open bit(s) from the sensor value
	uint32_t parityErrorCount;								// the number of words with bad parity we received

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
	bool backwards;
	bool comparisonEnabled;

	enum class LaserMonitorState : uint8_t
	{
		idle,
		calibrating,
		comparing
	};
	LaserMonitorState laserMonitorState;
};

#endif /* SRC_FILAMENTSENSORS_LASERFILAMENTMONITOR_H_ */
