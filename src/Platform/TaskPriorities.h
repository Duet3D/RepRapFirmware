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
	static constexpr int IdlePriority = 0;
	static constexpr int SpinPriority = 1;							// priority for tasks that rarely block
#if defined(LPC_NETWORKING)
    static constexpr int TcpPriority  = 2;
    //EMAC priority = 3 defined in FreeRTOSIPConfig.h
    static constexpr int HeatPriority = 4;
    static constexpr int SensorsPriority = 4;
    static constexpr int AinPriority = 4;
    static constexpr int HeightFollowingPriority = 4;
    static constexpr int LaserPriority = 5;
#else
    static constexpr int HeatPriority = 2;
	static constexpr int SensorsPriority = 2;
	static constexpr int TmcPriority = 2;
	static constexpr int AinPriority = 2;
	static constexpr int HeightFollowingPriority = 2;
	static constexpr int DueXPriority = 3;
	static constexpr int LaserPriority = 3;
	static constexpr int CanSenderPriority = 3;
	static constexpr int CanReceiverPriority = 3;
	static constexpr int CanClockPriority = 3;
	static constexpr int EthernetPriority = 3;
#endif
}

#endif /* SRC_TASKPRIORITIES_H_ */
