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
	  tolerance(DefaultTolerance), minimumExtrusionCheckLength(DefaultMinimumExtrusionCheckLength), withSwitch(type == 4)
{
	Init();
}

void Duet3DFilamentSensor::Init()
{
	samplesReceived = 0;
	sensorValue = 0;
	calibrationStarted = comparisonStarted = dataReceived = false;
	edgeCaptureReadPointer = edgeCaptureWritePointer = 1;
	edgeCaptures[0] = Platform::GetInterruptClocks();					// assume we just had a high-to-low transition
	lastMeasurementTime = 0;
	accumulatedExtrusionCommanded = accumulatedRevsMeasured = extrusionCommandedAtLastMeasurement = tentativeExtrusionCommanded = 0.0;
	state = RxdState::waitingForStartBit;
}

// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
bool Duet3DFilamentSensor::Configure(GCodeBuffer& gb, StringRef& reply, bool& seen)
{
	if (ConfigurePin(gb, reply, seen))
	{
		return true;
	}

	gb.TryGetFValue('S', mmPerRev, seen);
	gb.TryGetFValue('E', minimumExtrusionCheckLength, seen);

	if (gb.Seen('R'))
	{
		seen = true;
		tolerance = gb.GetFValue() * 0.01;
	}

	if (seen)
	{
		Init();
	}
	else
	{
		reply.printf("Duet3D filament sensor on endstop %u, %s microswitch, %.1fmm per rev, check every %.1fmm, tolerance %.1f%%, ",
						GetEndstopNumber(), (withSwitch) ? "with" : "no", (double)mmPerRev, (double)minimumExtrusionCheckLength, (double)(tolerance * 100.0));

		if (!dataReceived)
		{
			reply.cat("no data received");
		}
		else if ((sensorValue & ErrorBit) != 0)
		{
			reply.cat("error");
		}
		else
		{
			reply.catf("current angle %.1f", (double)GetCurrentAngle());
		}
	}

	return false;
}

// ISR for when the pin state changes
void Duet3DFilamentSensor::Interrupt()
{
	uint32_t now = Platform::GetInterruptClocks();
	const size_t writePointer = edgeCaptureWritePointer;			// capture volatile variable
	if ((writePointer + 1) % EdgeCaptureBufferSize != edgeCaptureReadPointer)
	{
		if (IoPort::ReadPin(GetPin()))
		{
			if ((writePointer & 1) == 0)							// low-to-high transitions should occur on odd indices
			{
				return;
			}
		}
		else
		{
			if ((writePointer & 1) != 0)							// high-to-low transitions should occur on even indices
			{
				return;
			}
			now -= 40;												// partial correction for skew caused by debounce filter on Duet endstop inputs (measured skew = 74)
		}
	}

	edgeCaptures[writePointer] = now;								// record the time at which this edge was detected
	edgeCaptureWritePointer = (writePointer + 1) % EdgeCaptureBufferSize;
}

// Call the following regularly to keep the status up to date
void Duet3DFilamentSensor::Poll()
{
	static constexpr uint32_t BitsPerSecond = 1000;							// the nominal bit rate that the data is transmitted at
	static constexpr uint32_t NominalBitLength = DDA::stepClockRate/BitsPerSecond;
	static constexpr uint32_t MinBitLength = (NominalBitLength * 10)/13;	// allow 30% clock speed tolerance
	static constexpr uint32_t MaxBitLength = (NominalBitLength * 13)/10;	// allow 30% clock speed tolerance
	static constexpr uint32_t ErrorRecoveryDelayBits = 8;					// before a start bit we want the line to be low for this long
	static constexpr uint32_t ErrorRecoveryTime = NominalBitLength * ErrorRecoveryDelayBits;

	bool again;
	do
	{
		again = false;
		const size_t writePointer = edgeCaptureWritePointer;			// capture volatile variable
		const uint32_t now = Platform::GetInterruptClocks();
		switch (state)
		{
		case RxdState::waitingForStartBit:
			if (writePointer != edgeCaptureReadPointer)
			{
				if ((edgeCaptureReadPointer & 1u) == 0)						// if we are out of sync (this is normal when the last stuffing bit was a 1)
				{
					edgeCaptureReadPointer = (edgeCaptureReadPointer + 1u) % EdgeCaptureBufferSize;
					again = true;
				}
				else
				{
					if (edgeCaptures[edgeCaptureReadPointer] - edgeCaptures[(edgeCaptureReadPointer - 1) % EdgeCaptureBufferSize] < ErrorRecoveryTime)
					{
						// The input line has not been low for long enough before the start bit
						edgeCaptureReadPointer = (edgeCaptureReadPointer + 1u) % EdgeCaptureBufferSize;		// ignore this start bit
						state = RxdState::errorRecovery1;
						again = true;
					}
					else
					{
						tentativeExtrusionCommanded = accumulatedExtrusionCommanded;	// we have received what could be the beginning of a start bit
						state = RxdState::waitingForEndOfStartBit;
						again = true;
					}
				}
			}
			break;

		case RxdState::waitingForEndOfStartBit:
			// This state must be made to time out because while we are in it, comparison of filament extruded is suspended
			if ((writePointer - edgeCaptureReadPointer) % EdgeCaptureBufferSize >= 2)
			{
				// Check for a valid start bit
				lastBitChangeIndex = (edgeCaptureReadPointer + 1u) % EdgeCaptureBufferSize;
				startBitLength = edgeCaptures[lastBitChangeIndex] - edgeCaptures[edgeCaptureReadPointer];
				edgeCaptureReadPointer = lastBitChangeIndex;
				if (startBitLength >= MinBitLength && startBitLength <= MaxBitLength)
				{
					valueBeingAssembled = 0;
					nibblesAssembled = 0;
					state = RxdState::waitingForNibble;
					again = true;
					//debugPrintf("sb %u\n", startBitLength);
				}
				else
				{
					// Start bit too long or too short
					state = RxdState::errorRecovery2;
					again = true;
				}
			}
			else if (now - edgeCaptures[edgeCaptureReadPointer] > MaxBitLength)
			{
				edgeCaptureReadPointer = (edgeCaptureReadPointer + 1u) % EdgeCaptureBufferSize;
				state = RxdState::errorRecovery2;
				again = true;
			}
			break;

		case RxdState::waitingForNibble:
			// This state must time out because while we are in it, comparison of filament extruded is suspended
			{
				const uint32_t nibbleStartTime = edgeCaptures[lastBitChangeIndex];
				if (now - nibbleStartTime > (13 * startBitLength)/2)
				{
					// 6.5 bit times have passed since the start of the bit that preceded the current nibble, so we should have a complete nibble and the following stuffing bit
					uint32_t samplePoint = (startBitLength * 3)/2;		// sampling time after the end of the start bit for bit 7 (MSB)
					uint8_t currentNibble = 0;
					size_t nextBitChangeIndex = (lastBitChangeIndex + 1u) % EdgeCaptureBufferSize;
					for (uint8_t numBits = 0; numBits < 5; ++numBits)
					{
						if (nextBitChangeIndex != edgeCaptureWritePointer && edgeCaptures[nextBitChangeIndex] - nibbleStartTime < samplePoint)
						{
							lastBitChangeIndex = nextBitChangeIndex;
							nextBitChangeIndex = (lastBitChangeIndex + 1u) % EdgeCaptureBufferSize;
							if (nextBitChangeIndex != writePointer && edgeCaptures[nextBitChangeIndex] - nibbleStartTime < samplePoint)
							{
								edgeCaptureReadPointer = nextBitChangeIndex;
								state = RxdState::errorRecovery3;
								again = true;
								break;
							}
						}
						currentNibble <<= 1;
						currentNibble |= (uint8_t)(lastBitChangeIndex & 1u);
						samplePoint += startBitLength;
					}

					if (state != RxdState::waitingForNibble)
					{
						break;
					}

					// The 5th bit we received should be the inverse of the 4th bit
					if ((((currentNibble >> 1u) ^ currentNibble) & 0x01u) == 0)
					{
						edgeCaptureReadPointer = nextBitChangeIndex;
						state = RxdState::errorRecovery4;
						again = true;
						break;
					}

					currentNibble >>= 1;
					valueBeingAssembled = (valueBeingAssembled << 4) | currentNibble;
					++nibblesAssembled;
					if (nibblesAssembled == 4)
					{
						edgeCaptureReadPointer = nextBitChangeIndex;	// ready for a new word
						if (samplesReceived == 0)
						{
							dataReceived = true;
							accumulatedExtrusionCommanded -= tentativeExtrusionCommanded;
							accumulatedRevsMeasured = 0.0;				// use the first measurement sample as a baseline
						}
						else
						{
							const uint16_t angleChange = (valueBeingAssembled - sensorValue) & AngleMask;		// angle change in range 0..1023
							const int32_t movement = (angleChange <= 512) ? (int32_t)angleChange : (int32_t)angleChange - 1024;
							accumulatedRevsMeasured += (float)movement/1024;
						}

						lastMeasurementTime = millis();
						extrusionCommandedAtLastMeasurement = tentativeExtrusionCommanded;
						sensorValue = valueBeingAssembled;
						if (samplesReceived < 100)
						{
							++samplesReceived;
						}
						edgeCaptureReadPointer = nextBitChangeIndex;
						state = RxdState::waitingForStartBit;
					}
					again = true;
				}
			}
			break;

		default:	// error recovery states
			if (reprap.Debug(moduleFilamentSensors))
			{
				debugPrintf("Fil err %u\n", (unsigned int)state);
			}
			state = RxdState::waitingForStartBit;
			again = true;
			break;
		}
	} while (again);
}

// Return the current wheel angle
float Duet3DFilamentSensor::GetCurrentAngle() const
{
	return (sensorValue & AngleMask) * (360.0/1024.0);
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus Duet3DFilamentSensor::Check(bool full, float filamentConsumed)
{
	accumulatedExtrusionCommanded += filamentConsumed;
	Poll();

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (full)
	{
		if ((sensorValue & ErrorBit) != 0)
		{
			ret = FilamentSensorStatus::sensorError;
		}
		else if (withSwitch && (sensorValue & SwitchOpenBit) != 0)
		{
			ret = FilamentSensorStatus::noFilament;
		}
		else if (samplesReceived >= 10 && state != RxdState::waitingForEndOfStartBit && state != RxdState::waitingForNibble)
		{
			if (extrusionCommandedAtLastMeasurement >= minimumExtrusionCheckLength)
			{
				ret = CheckFilament(extrusionCommandedAtLastMeasurement, false);
			}
			else if (accumulatedExtrusionCommanded >= minimumExtrusionCheckLength * 1.1 && millis() - lastMeasurementTime > 110)
			{
				ret = CheckFilament(accumulatedExtrusionCommanded, true);
			}
		}
	}

	return ret;
}

// Compare the amount commanded with the amount of extrusion measured, and set up for the next comparison
FilamentSensorStatus Duet3DFilamentSensor::CheckFilament(float amountCommanded, bool overdue)
{
	const float extrusionMeasured = accumulatedRevsMeasured * mmPerRev;
	if (reprap.Debug(moduleFilamentSensors))
	{
		debugPrintf("Extr req %.3f meas %.3f rem %.3f %s\n", (double)amountCommanded, (double)extrusionMeasured, (double)(accumulatedExtrusionCommanded - amountCommanded),
			(overdue) ? " overdue" : "");
	}

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (!comparisonStarted)
	{
		// The first measurement after we start extruding is often a long way out, so discard it
		comparisonStarted = true;
		calibrationStarted = false;
	}
	else if (tolerance >= 0.0)
	{
		const float minExtrusionExpected = (amountCommanded >= 0.0)
											 ? amountCommanded * (1.0 - tolerance)
												: amountCommanded * (1.0 + tolerance);
		if (extrusionMeasured < minExtrusionExpected)
		{
			ret = FilamentSensorStatus::tooLittleMovement;
		}
		else
		{
			const float maxExtrusionExpected = (amountCommanded >= 0.0)
												 ? amountCommanded * (1.0 + tolerance)
													: amountCommanded * (1.0 - tolerance);
			if (extrusionMeasured > maxExtrusionExpected)
			{
				ret = FilamentSensorStatus::tooMuchMovement;
			}
		}
	}
	else
	{
		// Tolerance < 0.0 means do calibration
		const float ratio = accumulatedRevsMeasured/amountCommanded;
		if (calibrationStarted)
		{
			if (ratio < minMovementRatio)
			{
				minMovementRatio = ratio;
			}
			if (ratio > maxMovementRatio)
			{
				maxMovementRatio = ratio;
			}
			totalExtrusionCommanded += amountCommanded;
			totalRevsMeasured += accumulatedRevsMeasured;
		}
		else
		{
			minMovementRatio = maxMovementRatio = ratio;
			totalExtrusionCommanded = amountCommanded;
			totalRevsMeasured = accumulatedRevsMeasured;
			calibrationStarted = true;
		}
	}
	accumulatedExtrusionCommanded -= amountCommanded;
	extrusionCommandedAtLastMeasurement = accumulatedRevsMeasured = 0.0;

	return ret;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus Duet3DFilamentSensor::Clear(bool full)
{
	Poll();								// to keep the diagnostics up to date
	accumulatedExtrusionCommanded = accumulatedRevsMeasured = extrusionCommandedAtLastMeasurement = tentativeExtrusionCommanded = 0.0;
	samplesReceived = 0;
	comparisonStarted = false;

	FilamentSensorStatus ret = FilamentSensorStatus::ok;
	if (full)
	{
		if ((sensorValue & ErrorBit) != 0)
		{
			ret = FilamentSensorStatus::sensorError;
		}
		else if (withSwitch && (sensorValue & SwitchOpenBit) != 0)
		{
			ret = FilamentSensorStatus::noFilament;
		}
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
	reprap.GetPlatform().MessageF(mtype, "Extruder %u sensor: angle %.1f, %s, ", extruder, (double)GetCurrentAngle(), statusText);
	if (calibrationStarted && fabsf(totalRevsMeasured) > 1.0 && totalExtrusionCommanded > 20.0)
	{
		const float measuredMmPerRev = totalExtrusionCommanded/totalRevsMeasured;
		const float normalRatio = 1.0/measuredMmPerRev;
		const int measuredPosTolerance = lrintf(100.0 * (((normalRatio > 0.0) ? maxMovementRatio : minMovementRatio) - normalRatio)/normalRatio);
		const int measuredNegTolerance = lrintf(100.0 * (normalRatio - ((normalRatio > 0.0) ? minMovementRatio : maxMovementRatio))/normalRatio);
		reprap.GetPlatform().MessageF(mtype,"%.2fmm/rev, tolerance +%d%% -%d%%\n", (double)measuredMmPerRev, measuredPosTolerance, measuredNegTolerance);
	}
	else
	{
		reprap.GetPlatform().Message(mtype, "no calibration data\n");
	}
}

// End
