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
	static MqttClient *Init(NetworkResponder *n, NetworkClient *c) noexcept;

	bool Spin() noexcept override;
	bool Accept(Socket *s) noexcept override;
	void Terminate() noexcept override;
	void Diagnostics(MessageType mtype) const noexcept override;

	bool HandlesProtocol(NetworkProtocol p) noexcept override;

	static GCodeResult Configure(GCodeBuffer &gb, const StringRef& reply) THROWS(GCodeException);
	static void Disable() noexcept;
	static void Publish(const char *msg, const char *topic, int qos, bool retain, bool dup) noexcept;

private:
	MqttClient(NetworkResponder *n, NetworkClient *c) noexcept;

	static constexpr int SendBufferSize = 1024;
	static constexpr int ReceiveBufferSize = 1024;

	static constexpr size_t DefaultKeepAlive = 400;
	static constexpr size_t MessageTimeout = 5000;
	static constexpr size_t ReconnectCooldown = 1000;

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

	uint8_t *sendBuf, *recvBuf;
	mqtt_client client;

	Subscription *prevSub, *currSub; // Used for subscribing to topics

	uint32_t messageTimer;	// General purpose variable for keeping track of queued messages timeout

	MqttClient *next;

	// MQTT configuration, shared by all MqttClient
	char *username;
	char *password;
	char *id;
	char *willTopic;
	char *willMessage;
	Subscription *subs;
	size_t keepAlive;
	uint8_t connectFlags;


	static MqttClient *clients; // List of all MQTT clients
};

#endif	// SUPPORT_MQTT

#endif /* SRC_NETWORKING_MQTT_MQTTCLIENT_H_ */
