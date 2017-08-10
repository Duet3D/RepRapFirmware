/*
 * Duet3DFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "Duet3DFilamentSensor.h"
#include "GCodes/GCodeBuffer.h"
#include "Platform.h"
#include "Movement/DDA.h"					// for stepClockRate

// Constructors
Duet3DFilamentSensor::Duet3DFilamentSensor(int type)
	: FilamentSensor(type), mmPerRev(DefaultMmPerRev),
	  tolerance(DefaultTolerance), absTolerance(DefaultAbsTolerance), minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), withSwitch(type == 4),
	  sensorValue(0), accumulatedExtrusionCommanded(0.0), accumulatedExtrusionMeasured(0.0), numberOfEdgesCaptured(0), state(RxdState::waitingForStartBit), dataReceived(false)
{
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool Duet3DFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	gb.TryGetFValue('S', mmPerRev, seen);
	gb.TryGetFValue('T', absTolerance, seen);
	gb.TryGetFValue('E', minimumExtrusionCheckLength, seen);

	if (gb.Seen('R'))
	{
		seen = true;
		const float tol = gb.GetFValue();
		if (tolerance < 0.0 || tolerance >= 100.0)
		{
			reply.copy("Relative tolerance must be between 0 and 100%");
			return true;
		}
		tolerance = tol * 0.01;
	}

	if (seen)
	{
		dataReceived = false;
		numberOfEdgesCaptured = 0;
		accumulatedExtrusionCommanded = accumulatedExtrusionMeasured = 0.0;
	}
	else
	{
		reply.printf("Duet3D filament sensor on endstop %u, %s microswitch, %.1fmm per rev, check every %.1fmm, tolerance %.1f%% + %.1fmm",
						GetEndstopNumber(), (withSwitch) ? "with" : "no", mmPerRev, minimumExtrusionCheckLength, tolerance * 100.0, absTolerance);
	}

	return false;
}

// ISR for when the pin state changes
void Duet3DFilamentSensor::Interrupt()
{
	const size_t numEdgesCaptured = numberOfEdgesCaptured;							// capture volatile variable
	if (numEdgesCaptured < MaxEdgeCaptures && (numEdgesCaptured % 2u) != (unsigned int)Platform::ReadPin(GetPin()))	// low-to-high transitions must be stored at even indices
	{
		edgeCaptures[numEdgesCaptured] = Platform::GetInterruptClocks();			// record the time at which this edge was detected
		numberOfEdgesCaptured = numEdgesCaptured + 1;
	}
}

// Call the following regularly to keep the status up to date
void Duet3DFilamentSensor::Poll()
{
	static const uint32_t BitsPerSecond = 1000;							// the nominal bit rate that the data is transmitted at
	static const uint32_t NominalBitLength = DDA::stepClockRate/BitsPerSecond;
	static const uint32_t MinBitLength = (NominalBitLength * 10)/13;	// allow 30% clock speed tolerance
	static const uint32_t MaxBitLength = (NominalBitLength * 13)/10;	// allow 30% clock speed tolerance
	static const uint32_t ErrorRecoveryDelayBits = 12;					// after an error we wait for the line to be low for this long
	static const uint32_t ErrorRecoveryTime = NominalBitLength * ErrorRecoveryDelayBits;

	const size_t numEdgesCaptured = numberOfEdgesCaptured;				// capture volatile variable
	const uint32_t now = Platform::GetInterruptClocks();
	switch (state)
	{
	case RxdState::waitingForStartBit:
		if (numEdgesCaptured >= 2)
		{
			// Check for a valid start bit
			startBitLength = edgeCaptures[1] - edgeCaptures[0];
			if (startBitLength >= MinBitLength && startBitLength <= MaxBitLength)
			{
				bitChangeIndex = 2;
				valueBeingAssembled = 0;
				nibblesAssembled = 0;
				state = RxdState::waitingForNibble;
			}
			else
			{
				state = RxdState::errorRecovery;
			}
		}
		break;

	case RxdState::waitingForNibble:
		{
			const uint32_t nibbleStartTime = edgeCaptures[bitChangeIndex - 1];
			if (now - nibbleStartTime > (13 * startBitLength)/2)
			{
				// 6.5 bit times have passed since the start of the bit that preceded the current nibble, so we should have a complete nibble and the following stuffing bit
				uint32_t samplePoint = (startBitLength * 3)/2;		// sampling time after the end of the start bit for bit 7 (MSB)
				uint8_t currentNibble = 0;
				for (uint8_t numBits = 0; numBits < 5; ++numBits)
				{
					if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - nibbleStartTime < samplePoint)
					{
						++bitChangeIndex;
						if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - nibbleStartTime < samplePoint)
						{
							state = RxdState::errorRecovery;		// there should be at most 1 transition per bit
							return;
						}
					}
					currentNibble <<= 1;
					if ((bitChangeIndex & 1u) != 0)
					{
						currentNibble |= 1u;
					}
					samplePoint += startBitLength;
				}

				// The 5th bit we received should be the inverse of the 4th bit
				if ((((currentNibble >> 1u) ^ currentNibble) & 0x01u) == 0)
				{
					state = RxdState::errorRecovery;
					return;
				}

				currentNibble >>= 1;
				valueBeingAssembled = (valueBeingAssembled << 4) | currentNibble;
				++nibblesAssembled;
				if (nibblesAssembled == 4)
				{
					numberOfEdgesCaptured = 0;				// ready for a new byte
					if (dataReceived)
					{
						const uint16_t angleChange = (valueBeingAssembled - sensorValue) & AngleMask;		// angle change in range 0..1023
						const int32_t movement = (angleChange <= 512) ? (int32_t)angleChange : (int32_t)angleChange - 1024;
						accumulatedExtrusionMeasured += (float)movement * mmPerRev * (1.0/1024.0);
					}
					sensorValue = valueBeingAssembled;
					dataReceived = true;
					state = RxdState::waitingForStartBit;
				}
			}
		}
		break;

	case RxdState::errorRecovery:
		if (Platform::ReadPin(GetPin()) || numEdgesCaptured != 0)	// when we first enter this state, numEdgesCaptured is always nonzero
		{
			errorRecoveryStartTime = now;
			numberOfEdgesCaptured = 0;
		}
		else if (now - errorRecoveryStartTime >= ErrorRecoveryTime)
		{
			state = RxdState::waitingForStartBit;
		}
		break;
	}
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus Duet3DFilamentSensor::Check(float filamentConsumed)
{
	accumulatedExtrusionCommanded += filamentConsumed;
	Poll();

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if ((sensorValue & ErrorBit) != 0)
	{
		ret = FilamentSensorStatus::sensorError;
	}
	else if (withSwitch && (sensorValue & SwitchOpenBit) != 0)
	{
		ret = FilamentSensorStatus::noFilament;
	}
	else if (accumulatedExtrusionCommanded >= minimumExtrusionCheckLength)
	{
		const float minExtrusionExpected = ( (accumulatedExtrusionCommanded >= 0.0)
											 ? accumulatedExtrusionCommanded * (1.0 - tolerance)
												: accumulatedExtrusionCommanded * (1.0 + tolerance)
										   )
										   - absTolerance;
		if (accumulatedExtrusionMeasured < minExtrusionExpected)
		{
			ret = FilamentSensorStatus::tooLittleMovement;
		}
		else
		{
			const float maxExtrusionExpected = ( (accumulatedExtrusionCommanded >= 0.0)
												 ? accumulatedExtrusionCommanded * (1.0 + tolerance)
													: accumulatedExtrusionCommanded * (1.0 - tolerance)
											   )
											   + absTolerance;
			if (accumulatedExtrusionMeasured > maxExtrusionExpected)
			{
				ret = FilamentSensorStatus::tooMuchMovement;
			}
		}
		accumulatedExtrusionCommanded = accumulatedExtrusionMeasured = 0.0;
	}

	return ret;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus Duet3DFilamentSensor::Clear()
{
	accumulatedExtrusionCommanded = accumulatedExtrusionMeasured = 0.0;
	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if ((sensorValue & ErrorBit) != 0)
	{
		ret = FilamentSensorStatus::sensorError;
	}
	else if (withSwitch && (sensorValue & SwitchOpenBit) != 0)
	{
		ret = FilamentSensorStatus::noFilament;
	}
	return ret;
}

// Print diagnostic info for this sensor
void Duet3DFilamentSensor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	Poll();
	const char* const statusText = (!dataReceived) ? "no data received"
									: ((sensorValue & ErrorBit) != 0) ? "error"
										: (withSwitch && (sensorValue & SwitchOpenBit) != 0) ? "no filament"
											: "ok";
	const float sensorAngle = (sensorValue & AngleMask) * (360.0/1024.0);
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: angle %.1f, %s\n", extruder, sensorAngle, statusText);
}

// End
