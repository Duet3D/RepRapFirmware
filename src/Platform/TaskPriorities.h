/*
 * TaskPriorities.h
 *
 *  Created on: 23 Oct 2019
 *      Author: David
 */

#ifndef SRC_TASKPRIORITIES_H_
#define SRC_TASKPRIORITIES_H_

// Task priorities. These must all be less than configMAX_PRIORITIES defined in FreeRTOSConfig.g.
namespace TaskPriority
{
	constexpr unsigned int IdlePriority = 0;
	constexpr unsigned int SpinPriority = 1;						// priority for tasks that rarely block
#if HAS_SBC_INTERFACE
	constexpr unsigned int SbcPriority = 2;							// priority for SBC task
#endif
    constexpr unsigned int HeatPriority = 3;
	constexpr unsigned int UsbPriority = 3;							// priority of USB task when using tinyusb
	constexpr unsigned int MovePriority = 4;
	constexpr unsigned int TmcPriority = 4;
	constexpr unsigned int AinPriority = 4;
	constexpr unsigned int HeightFollowingPriority = 4;
#ifdef DUET_NG
	constexpr unsigned int DueXPriority = 5;
#endif
	constexpr unsigned int LaserPriority = 5;
	constexpr unsigned int CanSenderPriority = 5;
	constexpr unsigned int EthernetPriority = 5;
	constexpr unsigned int CanReceiverPriority = 6;
	constexpr unsigned int Accelerometer = 6;
	constexpr unsigned int CanClockPriority = 7;

	// Assert that the highest priority one isn't too high
	static_assert(CanClockPriority < configMAX_PRIORITIES);
}

#endif /* SRC_TASKPRIORITIES_H_ */
