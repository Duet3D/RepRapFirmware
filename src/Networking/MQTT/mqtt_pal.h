/*
 * mqtt_pal.h
 *
 *  Created on: 01 Nov 2022
 *      Author: rechrtb
 */
#ifndef SRC_NETWORKING_MQTT_MQTT_PAL_H_
#define SRC_NETWORKING_MQTT_MQTT_PAL_H

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include <stdarg.h>

/*
 * MQTT-C PAL layer declarations and definitions
 */

// Network socket - pointer to Socket object, given to MqttClient on acceptance.
typedef void* mqtt_pal_socket_handle;

// Timekeeping - must give time in seconds. Define time units in terms of time_t,
// even though it is not referenced to epoch.
typedef time_t mqtt_pal_time_t;
extern uint32_t millis() noexcept;
#define MQTT_PAL_TIME() (millis() / 1000)

typedef struct
{
	struct Mutex *m;
} mqtt_pal_mutex_t;

#define MQTT_PAL_MUTEX_INIT(m)          mqtt_pal_mutex_init(m)
#define MQTT_PAL_MUTEX_LOCK(m)          mqtt_pal_mutex_lock(m)
#define MQTT_PAL_MUTEX_UNLOCK(m)        mqtt_pal_mutex_unlock(m)

// Byte order functions
#define MQTT_PAL_HTONS(s) __builtin_bswap16(s)
#define MQTT_PAL_NTOHS(s) __builtin_bswap16(s)

#ifdef __cplusplus
extern "C" {
#endif

// Socket read/write
/**
 * @brief Sends all the bytes in a buffer.
 * @ingroup pal
 *
 * @param[in] fd The file-descriptor (or handle) of the socket.
 * @param[in] buf A pointer to the first byte in the buffer to send.
 * @param[in] len The number of bytes to send (starting at \p buf).
 * @param[in] flags Flags which are passed to the underlying socket.
 *
 * @returns The number of bytes sent if successful, an \ref MQTTErrors otherwise.
 *
 * Note about the error handling:
 * - On an error, if some bytes have been processed already,
 *   this function should return the number of bytes successfully
 *   processed. (partial success)
 * - Otherwise, if the error is an equivalent of EAGAIN, return 0.
 * - Otherwise, return MQTT_ERROR_SOCKET_ERROR.
 */
ssize_t mqtt_pal_sendall(mqtt_pal_socket_handle fd, const void* buf, size_t len, int flags);

/**
 * @brief Non-blocking receive all the byte available.
 * @ingroup pal
 *
 * @param[in] fd The file-descriptor (or handle) of the socket.
 * @param[in] buf A pointer to the receive buffer.
 * @param[in] bufsz The max number of bytes that can be put into \p buf.
 * @param[in] flags Flags which are passed to the underlying socket.
 *
 * @returns The number of bytes received if successful, an \ref MQTTErrors otherwise.
 *
 * Note about the error handling:
 * - On an error, if some bytes have been processed already,
 *   this function should return the number of bytes successfully
 *   processed. (partial success)
 * - Otherwise, if the error is an equivalent of EAGAIN, return 0.
 * - Otherwise, return MQTT_ERROR_SOCKET_ERROR.
 */
ssize_t mqtt_pal_recvall(mqtt_pal_socket_handle fd, void* buf, size_t bufsz, int flags);

void mqtt_pal_mutex_init(mqtt_pal_mutex_t *mutex);
void mqtt_pal_mutex_lock(mqtt_pal_mutex_t *mutex);
void mqtt_pal_mutex_unlock(mqtt_pal_mutex_t *mutex);

#ifdef __cplusplus
}
#endif

#endif