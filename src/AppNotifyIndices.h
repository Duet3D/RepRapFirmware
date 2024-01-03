/*
 * AppNotifyIndices.h
 *
 *  Created on: 2 Jan 2024
 *      Author: David
 *
 *  Definitions of task notification indices used by the application layer
 */

#ifndef SRC_APPNOTIFYINDICES_H_
#define SRC_APPNOTIFYINDICES_H_

#if SUPPORT_CAN_EXPANSION
# include <CANlibNotifyIndices.h>
#else
# include <CoreNotifyIndices.h>
#endif

namespace NotifyIndices
{
#if SUPPORT_CAN_EXPANSION
	constexpr uint32_t FirstAvailableApp = NextAvailableAfterCANlib;
#else
	constexpr uint32_t FirstAvailableApp = NextAvailableAfterCore;
#endif
	constexpr uint32_t I2C = FirstAvailableApp;
	constexpr uint32_t Spi = FirstAvailableApp;
	constexpr uint32_t AccelerometerHardware = FirstAvailableApp + 1;
	constexpr uint32_t HeightController = FirstAvailableApp + 2;
	constexpr uint32_t AccelerometerDataCollector = FirstAvailableApp + 2;
	constexpr uint32_t Heat = FirstAvailableApp + 2;
	constexpr uint32_t Tmc = FirstAvailableApp + 2;
	constexpr uint32_t Move = FirstAvailableApp + 2;
	constexpr uint32_t DueX = FirstAvailableApp + 2;
	constexpr uint32_t CanMessageQueue = FirstAvailableApp + 3;
	constexpr uint32_t CanSender = FirstAvailableApp + 3;
	constexpr uint32_t SbcInterface = FirstAvailableApp + 3;
	constexpr uint32_t WiFi = FirstAvailableApp + 3;
	constexpr uint32_t EthernetHardware = FirstAvailableApp + 3;
	constexpr uint32_t Laser = FirstAvailableApp + 3;
	constexpr uint32_t TotalUsed = FirstAvailableApp + 4;
}

#ifdef RTOS
# include <FreeRTOSConfig.h>
static_assert(NotifyIndices::TotalUsed <= configTASK_NOTIFICATION_ARRAY_ENTRIES);
#endif

#endif /* SRC_APPNOTIFYINDICES_H_ */
