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
	: FilamentSensor(type), mmPerRev(DefaultMmPerRev), tolerance(DefaultTolerance), withSwitch(type == 4), sensorValue(0),
	  numberOfEdgesCaptured(0), state(RxdState::waitingForStartBit), dataReceived(false)
{
}

// Configure this sensor returning true if error
bool Duet3DFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	gb.TryGetFValue('S', mmPerRev, seen);

	if (gb.Seen('R'))
	{
		seen = true;
		const float tol = gb.GetFValue();
		if (tolerance < 0.0 || tolerance >= 1.0)
		{
			reply.copy("Tolerance must be between 0 and 1");
			return true;
		}
		tolerance = tol;
	}

	if (seen)
	{
		dataReceived = false;
		numberOfEdgesCaptured = 0;
	}
	else
	{
		reply.printf("Duet3D filament sensor on endstop %u, %s microswitch, %.1fmm per rev, tolerance %.2f",
						GetEndstopNumber(), (withSwitch) ? "with" : "no", mmPerRev, tolerance);
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
	static const uint32_t ErrorRecoveryDelayBits = 25;					// after an error we wait for the line to be low for this long
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
					if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - edgeCaptures[1] < samplePoint)
					{
						++bitChangeIndex;
						if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - edgeCaptures[1] < samplePoint)
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
// Return nullptr if everything is OK, else an error reason to include in a message.
const char *Duet3DFilamentSensor::Check(float filamentConsumed)
{
	if ((sensorValue & ErrorBit) != 0)
	{
		return "sensor error";
	}
	if (withSwitch && (sensorValue & SwitchOpenBit) != 0)
	{
		return "no filament";
	}

	//TODO
	return nullptr;
}

// Print diagnostic info for this sensor
void Duet3DFilamentSensor::Diagnostics(MessageType mtype, unsigned int extruder)
{
	const char* const statusText = (!dataReceived) ? "no data received"
									: ((sensorValue & ErrorBit) != 0) ? "error"
										: (withSwitch && (sensorValue & SwitchOpenBit) != 0) ? "no filament"
											: "ok";
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: angle %u, %s\n", extruder, sensorValue, statusText);
}

// End
