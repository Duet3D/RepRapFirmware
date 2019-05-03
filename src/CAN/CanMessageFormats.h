/*
 * CanMessageFormats.h
 *
 *  Created on: 16 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEFORMATS_H_
#define SRC_CAN_CANMESSAGEFORMATS_H_

constexpr unsigned int DriversPerCanBoard = 3;

union MovementFlags
{
	uint32_t u32;
	struct
	{
		uint32_t	deltaDrives : 4,
					pressureAdvanceDrives : 4,
					endStopsToCheck : 4,
					stopAllDrivesOnEndstopHit : 1;
	};
};

struct CanMovementMessage
{
	// Timing information
	uint32_t timeNow;					// the time now, temporary until we have separate clock sync messages
	uint32_t moveStartTime;				// when this move should start
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;
	float initialSpeedFraction;
	float finalSpeedFraction;

	MovementFlags flags;				// miscellaneous flags and small stuff

	float initialX;						// needed only for delta movement
	float initialY;						// needed only for delta movement
	float finalX;						// needed only for delta movement
	float finalY;						// needed only for delta movement
	float zMovement;					// needed only for delta movement

	struct
	{
		int32_t steps;					// net steps moved
	} perDrive[DriversPerCanBoard];

	void DebugPrint();
};

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
