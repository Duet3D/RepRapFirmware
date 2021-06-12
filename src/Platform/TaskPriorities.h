/*
 * TaskPriorities.h
 *
 *  Created on: 23 Oct 2019
 *      Author: David
 */

#ifndef SRC_TASKPRIORITIES_H_
#define SRC_TASKPRIORITIES_H_

// Task priorities
namespace TaskPriority
{
	constexpr int IdlePriority = 0;
	constexpr int SpinPriority = 1;							// priority for tasks that rarely block
#if HAS_LINUX_INTERFACE
	constexpr int SbcPriority = 1;							// priority for SBC task. TODO increase this when we are certain that it never spins.
#endif
#if defined(LPC_NETWORKING)
    constexpr int TcpPriority  = 2;
    //EMAC priority = 3 defined in FreeRTOSIPConfig.h
#endif
    constexpr int HeatPriority = 3;
	constexpr int Move = 4;
	constexpr int SensorsPriority = 4;
	constexpr int TmcPriority = 4;
	constexpr int AinPriority = 4;
	constexpr int HeightFollowingPriority = 4;
#ifdef DUET_NG
	constexpr int DueXPriority = 5;
#endif
	constexpr int LaserPriority = 5;
	constexpr int CanSenderPriority = 5;
	constexpr int CanReceiverPriority = 5;
	constexpr int EthernetPriority = 5;
	constexpr int Accelerometer = 6;
	constexpr int CanClockPriority = 7;
}

#endif /* SRC_TASKPRIORITIES_H_ */
