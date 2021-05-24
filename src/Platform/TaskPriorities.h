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
	constexpr int DueXPriority = 5;
	constexpr int LaserPriority = 5;
	constexpr int CanSenderPriority = 5;
	constexpr int CanReceiverPriority = 5;
	constexpr int EthernetPriority = 5;
	constexpr int Accelerometer = 6;
	constexpr int CanClockPriority = 7;
}

#endif /* SRC_TASKPRIORITIES_H_ */
