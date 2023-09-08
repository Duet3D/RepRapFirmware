/*
 * MqttClient.cpp
 *
 *  Created on: 01 Nov 2022
 *      Author: rechrtb
 */
#include "MqttClient.h"

#if SUPPORT_MQTT

#include <Platform/Platform.h>
#include <Networking/Network.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Socket.h>

#include "mqtt.h"

MqttClient::MqttClient(NetworkResponder *n, NetworkClient *c) noexcept
	: NetworkClient(n, c), prevSub(nullptr), currSub(nullptr), messageTimer(0), next(clients)
{
	clients = this;
	sendBuf = recvBuf = nullptr;

	username = nullptr;
	password = nullptr;
	id = nullptr;
	willTopic = nullptr;
	willMessage = nullptr;
	keepAlive = MqttClient::DefaultKeepAlive;
	connectFlags = MQTT_CONNECT_CLEAN_SESSION;
	subs = nullptr;
}

bool MqttClient::Spin() noexcept
{
	bool res = false;

	if (responderState != ResponderState::free)
	{
		mqtt_sync(&client);

		enum MQTTErrors err = client.error;

		if (err != MQTT_OK &&
			err != MQTT_ERROR_SUBSCRIBE_FAILED && // Handled in ResponderState::subscribing
			err != MQTT_ERROR_SEND_BUFFER_IS_FULL) // Buffer gets drained as socket send
		{
			if (reprap.Debug(Module::Webserver))
			{
				debugPrintf("MQTT encountered an error '%s', state = %d, resetting connection\n",
							mqtt_error_str(err), static_cast<int>(responderState));
			}
			ConnectionLost();
			res = true;
		}
		else
		{
			switch (responderState)
			{
			case ResponderState::connecting:
				{
					// Check if there is a queued CONNECT message
					struct mqtt_queued_message* msg = mqtt_mq_find(&client.mq, MQTT_CONTROL_CONNECT, NULL);
					bool connecting = msg && msg->state != MQTT_QUEUED_COMPLETE;

					if (connecting)
					{
						if (millis() - messageTimer >= MqttClient::MessageTimeout)
						{
							ConnectionLost();
							if (reprap.Debug(Module::Webserver))
							{
								debugPrintf("MQTT connect timed out\n");
							}
							res = true;
						}
					}
					else
					{
						currSub = subs;
						prevSub = nullptr;
						responderState = ResponderState::subscribing;
						if (reprap.Debug(Module::Webserver))
						{
							debugPrintf("MQTT client connected\n");
						}
						res = true;
					}
				}
				break;

			case ResponderState::subscribing:
				{
					// Check if there is a queued SUBSCRIBE message
					struct mqtt_queued_message* msg = mqtt_mq_find(&client.mq, MQTT_CONTROL_SUBSCRIBE, NULL);
					bool subscribing = (msg && msg->state != MQTT_QUEUED_COMPLETE);

					if (client.error == MQTT_ERROR_SUBSCRIBE_FAILED)
					{
						subscribing = false; // Skip the topic
						prevSub = nullptr;
						if (reprap.Debug(Module::Webserver))
						{
							debugPrintf("MQTT subscribe to topic %s failed, skipped\n", prevSub->topic);
						}
						res = true;
					}

					if (subscribing)
					{
						if (millis() - messageTimer >= MqttClient::MessageTimeout)
						{
							ConnectionLost();
							if (reprap.Debug(Module::Webserver))
							{
								debugPrintf("MQTT subscribe timed out\n");
							}
							res = true;
						}
					}
					else
					{
						if (prevSub && reprap.Debug(Module::Webserver))
						{
							debugPrintf("MQTT subscribed to topic %s\n", prevSub->topic);
						}

						if (currSub)
						{
							mqtt_subscribe(&client, currSub->topic, currSub->qos);
							prevSub = currSub;
							currSub = currSub->next;
							messageTimer = millis();
							res = true;
						}
						else
						{
							// No more topics, prepare to publish messages
							responderState = ResponderState::active;
						}
					}
				}
				break;

			case ResponderState::active:
				{
					struct mqtt_queued_message* msg = mqtt_mq_find(&client.mq, MQTT_CONTROL_PUBLISH, NULL);
					bool publishing = (msg && msg->state != MQTT_QUEUED_COMPLETE);

				 	if (publishing)
					{
						res = true;
					}
				}
				break;

			case ResponderState::disconnecting:
				{
					const mqtt_queued_message *const msg = mqtt_mq_find(&client.mq, MQTT_CONTROL_DISCONNECT, NULL);
					const bool disconnecting = (msg != nullptr && msg->state != MQTT_QUEUED_COMPLETE);

					// If received ACK for DISCONNECT regardless of result, or the time has expired.
					if (!disconnecting || millis() - messageTimer >= MqttClient::MessageTimeout)
					{
						ConnectionLost();
						if (reprap.Debug(Module::Webserver))
						{
							debugPrintf("MQTT disconnected\n");
						}
						res = true;
					}
				}
				break;

			default:
				break;
			}
		}
	}

	return res;
}

bool MqttClient::Accept(Socket *s) noexcept
{
	if (responderState == ResponderState::free)
	{
		skt = s;

		if (sendBuf && recvBuf)
		{
			mqtt_reinit(&client, skt, sendBuf, SendBufferSize, recvBuf, ReceiveBufferSize);
		}
		else
		{
			sendBuf = new uint8_t[SendBufferSize];
			recvBuf = new uint8_t[ReceiveBufferSize];
			mqtt_init(&client, skt, sendBuf, SendBufferSize, recvBuf, ReceiveBufferSize, PublishCallback);
		}

		mqtt_connect(&client, id, willTopic, willMessage, strlen(willMessage), username, password, connectFlags , keepAlive);
		responderState = ResponderState::connecting;
		messageTimer = millis();
		return true;
	}

	return false;
}

void MqttClient::Terminate() noexcept
{
	if (responderState != ResponderState::free)
	{
		ConnectionLost();
	}
}

void MqttClient::Diagnostics(MessageType mt) const noexcept
{
	GetPlatform().MessageF(mt, " MQTT(%d)", (int)responderState);
}

bool MqttClient::HandlesProtocol(NetworkProtocol protocol) noexcept
{
	return protocol == MqttProtocol;
}

bool MqttClient::Start() noexcept
{
	// Implement a simple reconnect cooldown
	if (millis() - messageTimer < ReconnectCooldown)
	{
		return false;
	}
	return true;
}

void MqttClient::Stop() noexcept
{
	if (responderState != ResponderState::free && responderState != ResponderState::disconnecting)
	{
		mqtt_disconnect(&client);
		responderState = ResponderState::disconnecting;
		messageTimer = millis();
	}
}

void MqttClient::ConnectionLost() noexcept
{
	NetworkClient::ConnectionLost();
	messageTimer = millis();
}

/* static */ GCodeResult MqttClient::Configure(GCodeBuffer &gb, const StringRef& reply) THROWS(GCodeException)
{
	MqttClient *client = clients;

	if (client->responderState != ResponderState::free)
	{
		reply.copy("Unable to configure MQTT when active on an interface");
		return GCodeResult::error;
	}

	String<MaxGCodeLength> param;

	auto clearMemb = [](char *&field)
	{
		if (field)
		{
			delete field;
			field = nullptr;
		}
	};

	auto setMemb = [clearMemb, reply, &param](char *&field) -> bool
	{
		clearMemb(field);
		size_t sz = param.strlen() + 1;
		field = new char[sz]();
		if (field == nullptr)
		{
			reply.copy("Unable to allocate memory");
			return false;
		}
		SafeStrncpy(field, param.c_str(), sz);
		return true;
	};

	if (gb.Seen('U'))
	{
		// Set username
		gb.GetQuotedString(param.GetRef());

		if (!setMemb(client->username))
		{
			return GCodeResult::error;
		}

		// Set password. Setting the password without the username shouldn't be
		// possible, so it's processed only if a username is also specified.
		if (gb.Seen('K'))
		{
			gb.GetQuotedString(param.GetRef());
			if (!setMemb(client->password))
			{
				clearMemb(client->username);
				return GCodeResult::error;
			}
		}
		else
		{
			clearMemb(client->password);
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Username set to '%s'", client->username);
			if (client->password)
			{
				debugPrintf("with password '%s'", client->password);
			}
			debugPrintf("\n");
		}
	}

	if (gb.Seen('C')) // Client ID
	{
		gb.GetQuotedString(param.GetRef());

		if (!setMemb(client->id))
		{
			return GCodeResult::error;
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Client ID set to '%s'\n", client->id);
		}
	}

	if (gb.Seen('W')) //  LWT message, topic, retain and qos
	{
		// Set the will message
		gb.GetQuotedString(param.GetRef());

		int qos = 0;
		int retain = 0;

		// Check qos
		if (gb.Seen('Q'))
		{
			qos = gb.GetIValue();
			if (qos < 0 || qos > 2)
			{
				reply.copy("Invalid will QOS");
				return GCodeResult::badOrMissingParameter;
			}
		}

		// Check retain flag
		if (gb.Seen('R'))
		{
			retain = gb.GetIValue();
			if (retain < 0 || retain > 1)
			{
				reply.copy("Invalid will retain flag");
				return GCodeResult::badOrMissingParameter;
			}
		}

		if (!setMemb(client->willMessage))
		{
			return GCodeResult::error;
		}

		// Set will topic. Setting the will topic without the will message shouldn't be
		// possible, so it's processed only if a will message is also specified.
		gb.MustSee('T');
		{
			gb.GetQuotedString(param.GetRef());
			if (!setMemb(client->willTopic))
			{
				clearMemb(client->willMessage);
				return GCodeResult::error;
			}
		}

		switch (qos)
		{
		case 1:
			client->connectFlags |= MQTT_CONNECT_WILL_QOS_1;
			break;

		case 2:
			client->connectFlags |= MQTT_CONNECT_WILL_QOS_2;
			break;

		case 0:
		default:
			client->connectFlags |= MQTT_CONNECT_WILL_QOS_0;
			break;
		}

		if (retain)
		{
			client->connectFlags |= MQTT_CONNECT_WILL_RETAIN;
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Set will message '%s' with topic '%s', QOS=%d, retain = %s\n",
						client->willMessage, client->willTopic, qos, retain ? "true": "false");
		}
	}

	if (gb.Seen('S')) // Subscribe topic
	{
		gb.GetQuotedString(param.GetRef());

		// Check the max QOS first
		int qos = 0;
		if (gb.Seen('M'))
		{
			qos = gb.GetIValue();
			if (qos < 0 || qos > 2)
			{
				reply.copy("Invalid max QOS");
				return GCodeResult::badOrMissingParameter;
			}
		}

		// Then check if the topic is already in the subscriptions,
		Subscription *sub;
		for (sub = client->subs; sub != nullptr; sub = sub->next)
		{
			if (strcmp(sub->topic, param.c_str()) == 0)
			{
				break;
			}
		}

		if (sub)
		{
			// Just overwrite the existing QOS
			sub->qos = qos;
		}
		else
		{
			// If parameters are valid, allocate mem for it
			size_t sz = param.strlen() + 1;
			sub = new Subscription(sz);

			if (!sub)
			{
				reply.copy("Unable to allocate memory");
				return GCodeResult::error;
			}

			SafeStrncpy(sub->topic, param.c_str(), sz);
			sub->qos = qos;

			// Append to list of subscriptions
			sub->next = client->subs;
			client->subs = sub;

			if (reprap.Debug(Module::Webserver))
			{
				debugPrintf("Topic '%s' added with max QOS=%d to subscriptions\n", param.c_str(), qos);
			}
		}
	}

	return GCodeResult::ok;
}

/* static */void MqttClient::Disable() noexcept
{
	// Nothing needed here
}

/* static */void MqttClient::Publish(const char *msg, const char *topic, int qos, bool retain, bool dup) noexcept
{
	MqttClient *client = clients;
	if (client->responderState == ResponderState::active)
	{
		uint8_t flags = 0;

		switch (qos)
		{
		case 1:
			flags |= MQTT_PUBLISH_QOS_1;
			break;

		case 2:
			flags |= MQTT_PUBLISH_QOS_2;
			break;

		case 0:
		default:
			flags |= MQTT_PUBLISH_QOS_0;
			break;
		}

		if (retain)
		{
			flags |= MQTT_PUBLISH_RETAIN;
		}

		if (dup)
		{
			flags |= MQTT_PUBLISH_DUP;
		}

		const MQTTErrors mqttErr = mqtt_publish(&client->client, topic, msg, strlen(msg), flags);

		if (mqttErr != MQTT_OK)
		{
			GetPlatform().MessageF(UsbMessage, "Failed to publish MQTT message with error '%" PRId16 "'\n", mqttErr);
		}
	}
	else
	{
		GetPlatform().MessageF(UsbMessage, "Failed to publish MQTT message, client not active\n");
	}
}

/*static*/ void MqttClient::PublishCallback(void** state, struct mqtt_response_publish *msg)
{
	// Null terminate the received message details. Since this is just for display, cut them off if they don't fit the buffer.
	char topic[32];
	SafeStrncpy(topic, static_cast<const char*>(msg->topic_name), std::min(static_cast<size_t>(msg->topic_name_size + 1), sizeof(topic)));
	char message[64];
	SafeStrncpy(message, static_cast<const char*>(msg->application_message), std::min(static_cast<size_t>(msg->application_message_size + 1), sizeof(message)));
	GetPlatform().MessageF(UsbMessage, "Received message from topic '%s': '%s'\n", topic, message);
}

MqttClient *MqttClient::Init(NetworkResponder *n, NetworkClient *c) noexcept
{
	return new MqttClient(n, c);
}

/* Static members */

MqttClient *MqttClient::clients = nullptr;

#endif
