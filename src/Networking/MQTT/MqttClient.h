/*
 * MqttClient.h
 *
 *  Created on: 01 Nov 2022
 *      Author: rechrtb
 */
#ifndef SRC_NETWORKING_MQTT_MQTTCLIENT_H_
#define SRC_NETWORKING_MQTT_MQTTCLIENT_H_

#include <RepRapFirmware.h>

#if SUPPORT_MQTT

#include "mqtt.h"
#include "NetworkClient.h"
#include "General/StringFunctions.h"

class MqttClient : public NetworkClient
{
public:
	MqttClient(NetworkResponder *n, NetworkClient *c) noexcept;

	bool Spin() noexcept override;
	bool Accept(Socket *s) noexcept override;
	void Terminate() noexcept override;
	void Diagnostics(MessageType mtype) const noexcept override;

	bool HandlesProtocol(NetworkProtocol p) noexcept override;

	static GCodeResult Configure(GCodeBuffer &gb, const StringRef& reply) THROWS(GCodeException);
	static void Disable() noexcept;
	static void Publish(const char *msg) noexcept;

private:
	static const int SendBufferSize = 2048;
	static const int ReceiveBufferSize = 1024;
	static const size_t DefaultKeepAlive = 400;
	static const size_t MessageTimeout = 5000;
	static const size_t ReconnectCooldown = 1000;

	struct Subscription
	{
		Subscription(size_t sz)
		{
			topic = new char[sz];
		}

		char *topic;
		uint8_t qos;
		struct Subscription *next;
	};

	bool Start() noexcept override;
	void Stop() noexcept override;
	void ConnectionLost() noexcept override;
	static void PublishCallback(void** state, struct mqtt_response_publish *published);

	mqtt_client client;
	uint8_t sendBuf[SendBufferSize];
	uint8_t recvBuf[ReceiveBufferSize];

	Subscription *prevSub, *currSub; // Used for subscribing to topics
	OutputBuffer *currBuf; // Current message being published

	uint32_t messageTimer;	// General purpose variable for keeping track of queued messages timeout
	Mutex publishMutex;

	MqttClient *next;

	static MqttClient *instance;

	// MQTT configuration, shared by all MqttClient's
	static char *username;
	static char *password;
	static char *id;
	static char *willTopic;
	static char *willMessage;
	static size_t keepAlive;
	static char *publishTopic;
	static uint8_t publishQos;
	static bool duplicate;
	static bool retain;
	static Subscription *subs;

	static MqttClient *clients; // List of all MQTT clients
};

#endif	// SUPPORT_MQTT

#endif /* SRC_NETWORKING_MQTT_MQTTCLIENT_H_ */
