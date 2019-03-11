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
	LaserFilamentMonitor(unsigned int extruder, int type);

	bool Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) override;
	FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) override;
	FilamentSensorStatus Clear() override;
	void Diagnostics(MessageType mtype, unsigned int extruder) override;

private:
	static constexpr float DefaultMinMovementAllowed = 0.6;
	static constexpr float DefaultMaxMovementAllowed = 1.6;
	static constexpr float DefaultMinimumExtrusionCheckLength = 3.0;

	// This is the received data format:
	//  v1 Data word:			P00S 00pp pppp pppp		S = switch open, pppppppppp = 10-bit filament position (50 counts/mm)
	//  v2 Data word:			P00S 1ppp pppp pppp		S = switch open, ppppppppppp = 11-bit filament position (100 counts/mm)
	//  v1 Error word:			P010 0000 0000 0000
	//  v2 Error word:			P010 0000 0000 eeee		eeee = error code, will not be zero
	//  v1 Quality word:		P10s ssss bbbb bbbb		sssss = shutter, bbbbbbbb = brightness
	//	v2 Version word:		P110 0000 vvvv vvvv		vvvvvvvv = sensor/firmware version, at least 2
	//  v2 Image quality word:	P110 0001 qqqq qqqq		qqqqqqqq = image quality
	//  v2 Brightness word:		P110 0010 bbbb bbbb		bbbbbbbb = brightness
	//  v2 Shutter word:		P110 0011 ssss ssss		ssssssss = shutter

	// Definitions for all messages
	static constexpr uint16_t TypeLaserParityMask = 0x8000;

	// Definitions for identifying the top level type of a message
	static constexpr uint16_t TypeLaserMessageTypeMask = 0x6000;
	static constexpr uint16_t TypeLaserMessageTypePosition = 0x0000;
	static constexpr uint16_t TypeLaserMessageTypeError = 0x2000;
	static constexpr uint16_t TypeLaserMessageTypeQuality = 0x4000;
	static constexpr uint16_t TypeLaserMessageTypeInfo = 0x6000;

	// Definitions for position data messages
	static constexpr uint16_t TypeLaserSwitchOpenBitMask = 0x1000;
	static constexpr uint16_t TypeLaserLargeDataRangeBitMask = 0x0800;
	static constexpr uint16_t TypeLaserDefaultRange = 1024;			// 10-bit sensor position
	static constexpr uint16_t TypeLaserLargeRange = 2048;			// 11-bit sensor position

	// Definitions for info message types
	static constexpr uint16_t TypeLaserInfoTypeMask = 0x1F00;
	static constexpr uint16_t TypeLaserInfoTypeVersion = 0x0000;
	static constexpr uint16_t TypeLaserInfoTypeImageQuality = 0x0100;
	static constexpr uint16_t TypeLaserInfoTypeBrightness = 0x0200;
	static constexpr uint16_t TypeLaserInfoTypeShutter = 0x0300;

	static constexpr size_t EdgeCaptureBufferSize = 64;				// must be a power of 2

	void Init();
	void Reset();
	void HandleIncomingData();
	float GetCurrentPosition() const;
	FilamentSensorStatus CheckFilament(float amountCommanded, float amountMeasured, bool overdue);

	// Configuration parameters
	float minMovementAllowed, maxMovementAllowed;
	float minimumExtrusionCheckLength;
	bool comparisonEnabled;
	bool checkNonPrintingMoves;

	// Other data
	uint32_t framingErrorCount;								// the number of framing errors we received
	uint32_t parityErrorCount;								// the number of words with bad parity we received
	uint32_t overdueCount;									// the number of times a position report was overdue

	uint32_t candidateStartBitTime;							// the time that we received a possible start bit
	float extrusionCommandedAtCandidateStartBit;			// the amount of extrusion commanded since the previous comparison when we received the possible start bit

	uint32_t lastSyncTime;									// the last time we took a measurement that was synced to a start bit
	float extrusionCommandedSinceLastSync;
	float movementMeasuredSinceLastSync;

	uint32_t lastMeasurementTime;							// the last time we received a value
	uint16_t sensorValue;									// last known filament position (10 or 11 bits)
	uint16_t switchOpenMask;								// mask to isolate the switch open bit(s) from the sensor value
	uint8_t version;										// sensor/firmware version
	uint8_t imageQuality;									// image quality returned by version 2 prototype sensor
	uint8_t shutter;										// shutter value returned by sensor
	uint8_t brightness;										// brightness returned by sensor
	uint8_t lastErrorCode;									// the last error code received
	bool sensorError;										// true if received an error report (cleared by a position report)

	bool wasPrintingAtStartBit;
	bool haveStartBitData;
	bool synced;

	float extrusionCommandedThisSegment;					// the amount of extrusion commanded since we last did a comparison
	float movementMeasuredThisSegment;						// the accumulated movement since the previous comparison

	// Values measured for calibration
	float minMovementRatio, maxMovementRatio;
	float totalExtrusionCommanded;
	float totalMovementMeasured;

	bool dataReceived;
	bool backwards;

	enum class LaserMonitorState : uint8_t
	{
		idle,
		calibrating,
		comparing
	};
	LaserMonitorState laserMonitorState;
};

#endif /* SRC_FILAMENTSENSORS_LASERFILAMENTMONITOR_H_ */
