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
	RotatingMagnetFilamentMonitor(unsigned int extruder, int type);

	bool Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) override;
	FilamentSensorStatus Check(bool full, bool hadNonPrintingMove, bool fromIsr, float filamentConsumed) override;
	FilamentSensorStatus Clear(bool full) override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;

private:
	static constexpr float DefaultMmPerRev = 28.8;
	static constexpr float DefaultMinMovementAllowed = 0.6;
	static constexpr float DefaultMaxMovementAllowed = 1.6;
	static constexpr float DefaultMinimumExtrusionCheckLength = 3.0;

	static constexpr uint16_t TypeMagnetErrorMask = 0x8000u;
	static constexpr uint16_t TypeMagnetSwitchOpenMask = 0x4000u;
	static constexpr uint16_t TypeMagnetAngleMask = 0x03FF;			// we use a 10-bit sensor angle

	void Init();
	void Reset();
	void HandleIncomingData();
	float GetCurrentPosition() const;
	FilamentSensorStatus CheckFilament(float amountCommanded, float amountMeasured, bool overdue);

	// Configuration parameters
	float mmPerRev;
	float minMovementAllowed, maxMovementAllowed;
	float minimumExtrusionCheckLength;
	bool comparisonEnabled;
	bool checkNonPrintingMoves;

	// Other data
	uint16_t sensorValue;									// last known filament position (10 bits)
	uint32_t lastMeasurementTime;							// the last time we received a value
	uint16_t switchOpenMask;								// mask to isolate the switch open bit(s) from the sensor value
	uint32_t framingErrorCount;								// the number of framing errors we received

	float extrusionCommandedAtStartBit;						// the amount of extrusion commanded (mm) when we received the start bit since the last sync
	float extrusionCommandedSinceLastSync;					// the amount of extrusion commanded (mm) since the last sync
	float movementMeasuredSinceLastSync;					// the amount of movement in complete rotations of the wheel since the last sync
	bool hadNonPrintingMoveAtStartBit;
	bool hadNonPrintingMoveSinceLastSync;
	bool haveStartBitData;

	float extrusionCommandedThisSegment;					// the amount of extrusion commanded (mm) since we last did a comparison
	float movementMeasuredThisSegment;						// the accumulated movement in complete rotations since the previous comparison

	// Values measured for calibration
	float minMovementRatio, maxMovementRatio;
	float totalExtrusionCommanded;
	float totalMovementMeasured;

	bool dataReceived;
	bool comparisonStarted;
	bool calibrationStarted;
};

#endif /* SRC_FILAMENTSENSORS_ROTATINGMAGNETFILAMENTMONITOR_H_ */
