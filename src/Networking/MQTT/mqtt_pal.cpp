/*
 * mqtt_pal.cpp
 *
 *  Created on: 01 Nov 2022
 *      Author: rechrtb
 */
#include <Platform/Platform.h>
#include <RTOSIface/RTOSIface.h>
#include <Socket.h>

#include "mqtt.h"

/*
 * MQTT-C PAL layer implementation
 */

ssize_t mqtt_pal_sendall(mqtt_pal_socket_handle fd, const void* buf, size_t len, int flags)
{
	Socket* skt = static_cast<Socket*>(fd);

	ssize_t res = 0;

	if (skt->CanSend())
	{
		res = skt->Send((const uint8_t*)buf, len);
	}
	else
	{
		res = MQTT_ERROR_SOCKET_ERROR;
	}

	return res;
}

ssize_t mqtt_pal_recvall(mqtt_pal_socket_handle fd, void* buf, size_t sz, int flags)
{
	Socket* skt = static_cast<Socket*>(fd);

	ssize_t res = 0;
	char c = 0;

	if (skt->CanRead())
	{
		char *cbuf = static_cast<char*>(buf);
		while (res < static_cast<ssize_t>(sz) && skt->ReadChar(c))
		{
			cbuf[res] = c;
			res++;
		}
	}
	else
	{
		res = MQTT_ERROR_SOCKET_ERROR;
	}

	return res;
}

void mqtt_pal_mutex_init(mqtt_pal_mutex_t *mutex)
{
	static const char name[] = "MQTTClient";
	mutex->m = new Mutex();
	mutex->m->Create(name);
}

void mqtt_pal_mutex_lock(mqtt_pal_mutex_t *mutex)
{
	mutex->m->Take();
}

void mqtt_pal_mutex_unlock(mqtt_pal_mutex_t *mutex)
{
	mutex->m->Release();
}
