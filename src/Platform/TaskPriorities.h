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
	constexpr unsigned int IdlePriority = 0;
	constexpr unsigned int SpinPriority = 1;						// priority for tasks that rarely block
#if HAS_LINUX_INTERFACE
	constexpr unsigned int SbcPriority = 2;							// priority for SBC task
#endif
#if defined(LPC_NETWORKING)
    constexpr int TcpPriority  = 2;
    //EMAC priority = 3 defined in FreeRTOSIPConfig.h
#endif
    constexpr unsigned int HeatPriority = 3;
	constexpr unsigned int Move = 4;
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
}

#endif /* SRC_TASKPRIORITIES_H_ */
