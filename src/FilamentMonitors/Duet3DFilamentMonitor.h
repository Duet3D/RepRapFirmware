/*
 * Duet3DFilamentMonitor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 *
 *  This is the base class for filament monitors that use the Duet3D protocol for sending 16-bit words to the Duet.
 */

#ifndef SRC_FILAMENTSENSORS_DUET3DFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_DUET3DFILAMENTMONITOR_H_

#include "FilamentMonitor.h"

class Duet3DFilamentMonitor : public FilamentMonitor
{
protected:
	DECLARE_OBJECT_MODEL

	Duet3DFilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept;

	bool Interrupt() noexcept override;

#if SUPPORT_CAN_EXPANSION
	void UpdateLiveData(const FilamentMonitorDataNew2& data) noexcept override;
#endif
#if SUPPORT_REMOTE_COMMANDS
	void GetLiveData(FilamentMonitorDataNew2& data) const noexcept override;
#endif

	void InitReceiveBuffer() noexcept;

	enum class PollResult : uint8_t
	{
		incomplete,
		complete,
		error
	};
	PollResult PollReceiveBuffer(uint16_t& measurement) noexcept;
	bool IsReceiving() const noexcept;
	bool IsWaitingForStartBit() const noexcept;

	uint32_t overrunErrorCount;
	uint32_t polarityErrorCount;

	// Live data reported in the object model
	float totalExtrusionCommanded = 0.0;
	uint16_t lastKnownPosition = 0;
	int16_t avgPercentage;
	int16_t minPercentage;
	int16_t maxPercentage;
	int16_t lastPercentage;
	bool hasLiveData = false;

private:
	static constexpr size_t EdgeCaptureBufferSize = 64;				// must be a power of 2

	// Buffer used to capture received data, and associated info
	uint32_t edgeCaptures[EdgeCaptureBufferSize];
	size_t edgeCaptureReadPointer;
	volatile size_t edgeCaptureWritePointer;

	enum class RxdState : uint8_t
	{
		waitingForStartBit = 0,
		waitingForEndOfStartBit,
		waitingForNibble,
		errorRecovery1,
		errorRecovery2,
		errorRecovery3,
		errorRecovery4
	};

	uint32_t startBitLength;
	uint32_t errorRecoveryStartTime;
	size_t lastBitChangeIndex;
	uint16_t valueBeingAssembled;
	uint8_t nibblesAssembled;
	RxdState state;
};

#endif /* SRC_FILAMENTSENSORS_DUET3DFILAMENTMONITOR_H_ */
