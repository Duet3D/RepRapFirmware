/*
 * Duet3DFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 *
 * This is the base class for the Duet3D laser and magnetic filament monitors. Both of these report filament position at intervals.
 * RepRapFirmware uses the difference between position reports to calculate the amount and direction of filament movement.
 * The position reports are sent as 10-bit values in a 16-bit word.
 *
 * This module is responsible for receiving the 16-bit word. It does this by using an interrupt to detect each edge of the input
 * from the filament monitor and recording the times of these edges in a ring buffer (see function Interrupt). If the state machine
 * indicates that we are waiting for the start bit of a new 16-bit word then Interrupt() returns true, causing the calling function
 * in FilamentMonitor.cpp to record the extrusion commanded.
 *
 * The RRF main polling loop calls FilamentMonitor::Spin, which calls the Check() function of each active filament monitor.
 *
 * The Check function in LaserFilamentMonitor or RotatingMagnetFilamenMonitor calls PollReceiveBuffer at intervals via, which decodes
 * the data in the ring buffer, and passes back the 16-bit word when it is complete. The Check function decodes the 16-bit word, and
 * if it is a measurement, retrieves the extrusion amount that was saved when the start bit was seen.
 */

#include "Duet3DFilamentMonitor.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/Platform.h>
#include <Movement/StepTimer.h>
#include <Platform/RepRap.h>

// Constructors
Duet3DFilamentMonitor::Duet3DFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept
	: FilamentMonitor(drv, monitorType, did), overrunErrorCount(0), polarityErrorCount(0)
{
	InitReceiveBuffer();
}

void Duet3DFilamentMonitor::InitReceiveBuffer() noexcept
{
	edgeCaptureReadPointer = edgeCaptureWritePointer = 1;
	edgeCaptures[0] = StepTimer::GetTimerTicks();				// pretend we just had a high-to-low transition
	state = RxdState::waitingForStartBit;
}

// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched and stored because we have seen a potential start bit.
bool Duet3DFilamentMonitor::Interrupt() noexcept
{
	uint32_t now = StepTimer::GetTimerTicks();
	bool wantReading = false;
	const size_t writePointer = edgeCaptureWritePointer;			// capture volatile variable
	if ((writePointer + 1) % EdgeCaptureBufferSize != edgeCaptureReadPointer)	// if buffer is not full
	{
		if (GetPort().ReadDigital())
		{
			if ((writePointer & 1) == 0)							// low-to-high transitions should occur on odd indices
			{
				++polarityErrorCount;
				return false;
			}
			if (state == RxdState::waitingForStartBit && writePointer == edgeCaptureReadPointer && !HaveIsrStepsCommanded())
			{
				wantReading = true;									// if this is a possible start bit, ask for the extrusion commanded
			}
		}
		else
		{
			if ((writePointer & 1) != 0)							// high-to-low transitions should occur on even indices
			{
				++polarityErrorCount;
				return false;
			}
			now -= 40;												// partial correction for skew caused by debounce filter on older Duet endstop inputs (measured skew = 74)
		}

		edgeCaptures[writePointer] = now;							// record the time at which this edge was detected
		edgeCaptureWritePointer = (writePointer + 1) % EdgeCaptureBufferSize;
	}
	else
	{
		++overrunErrorCount;
	}
	return wantReading;
}

// Call the following regularly to keep the status up to date
Duet3DFilamentMonitor::PollResult Duet3DFilamentMonitor::PollReceiveBuffer(uint16_t& measurement) noexcept
{
	// For the Duet3D sensors we need to decode the received data from the transition times recorded in the edgeCaptures array
	static constexpr uint32_t BitsPerSecond = 1000;							// the nominal bit rate that the data is transmitted at
	static constexpr uint32_t NominalBitLength = StepClockRate/BitsPerSecond;	// the nominal bit length in step clocks
	static constexpr uint32_t MinBitLength = (NominalBitLength * 10)/13;	// allow 30% clock speed tolerance
	static constexpr uint32_t MaxBitLength = (NominalBitLength * 13)/10;	// allow 30% clock speed tolerance
	static constexpr uint32_t ErrorRecoveryDelayBits = 8;					// before a start bit we want the line to be low for this long
	static constexpr uint32_t ErrorRecoveryTime = NominalBitLength * ErrorRecoveryDelayBits;

	bool again;
	do
	{
		again = false;
		const size_t writePointer = edgeCaptureWritePointer;				// capture volatile variable
		const uint32_t now = StepTimer::GetTimerTicks();
		switch (state)
		{
		case RxdState::waitingForStartBit:
			if (writePointer != edgeCaptureReadPointer)						// if we have recorded any new edges
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
						// The input line has not been idle for long enough before the start bit
						edgeCaptureReadPointer = (edgeCaptureReadPointer + 1u) % EdgeCaptureBufferSize;		// ignore this start bit
						state = RxdState::errorRecovery1;
						again = true;
					}
					else
					{
						state = RxdState::waitingForEndOfStartBit;
						again = true;
					}
				}
			}
			break;

		case RxdState::waitingForEndOfStartBit:
			// This state must time out because while we are in it, comparison of filament extruded is suspended
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
				}
				else
				{
					// Start bit too long or too short
					state = RxdState::errorRecovery2;
					again = true;
				}
			}
			else if (now - edgeCaptures[edgeCaptureReadPointer] > MaxBitLength)		// check for timeout
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
								// We recorded two edges within one sample period
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
					if (nibblesAssembled == 4)							// if we have a complete 16-bit word
					{
						edgeCaptureReadPointer = nextBitChangeIndex;	// ready for a new word
						measurement = valueBeingAssembled;
						state = RxdState::waitingForStartBit;
						return PollResult::complete;
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
			return PollResult::error;
		}
	} while (again);
	return PollResult::incomplete;
}

// Return true if we are on the process of receiving data from the filament monitor
bool Duet3DFilamentMonitor::IsReceiving() const noexcept
{
	return state == RxdState::waitingForEndOfStartBit || state == RxdState::waitingForNibble;
}

// Return true if we are waiting for a start bit
bool Duet3DFilamentMonitor::IsWaitingForStartBit() const noexcept
{
	return state == RxdState::waitingForStartBit;
}

// End
