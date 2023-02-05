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
	: NetworkClient(n, c),
	  prevSub(nullptr), currSub(nullptr), currBuf(nullptr), messageTimer(0), next(clients)
{
	clients = this;

	publishMutex.Create("MqttPublishBuffer");

	memset(sendBuf, 0, sizeof(sendBuf));
	memset(recvBuf, 0, sizeof(recvBuf));

	mqtt_init(&client, skt, sendBuf, sizeof(sendBuf), recvBuf, sizeof(recvBuf), PublishCallback);
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
							currBuf = nullptr;
							responderState = ResponderState::active;
						}
					}
				}
				break;

			case ResponderState::active:
				{
					MutexLocker lock(publishMutex);

					if (currBuf)
					{
						outBuf = OutputBuffer::Release(currBuf);
						res = true;
					}

					currBuf = outBuf;

					if (currBuf)
					{
						uint8_t flags = 0;

						// If not specified, publish under the hostname
						const char *const topic = publishTopic ? publishTopic : reprap.GetNetwork().GetHostname();

						flags |= (publishQos == 0) ? MQTT_PUBLISH_QOS_0 :
									(publishQos == 1 ? MQTT_PUBLISH_QOS_1 : MQTT_PUBLISH_QOS_2);
						flags |= (retain) ? MQTT_PUBLISH_RETAIN : 0;
						flags |= (duplicate) ? MQTT_PUBLISH_DUP : 0;

						const MQTTErrors mqttErr = mqtt_publish(&client, topic, currBuf->Data(), currBuf->DataLength(), flags);
						if (mqttErr == MQTT_ERROR_SEND_BUFFER_IS_FULL)
						{
							currBuf = nullptr; // retry to publish the same buffer on the next loop
						}
						else
						{
							res = true;
						}
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
		mqtt_reinit(&client, skt, sendBuf, SendBufferSize, recvBuf, ReceiveBufferSize);
		mqtt_connect(&client, id, willTopic, willMessage, strlen(willMessage), username, password, MQTT_CONNECT_CLEAN_SESSION, keepAlive);
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
	// Since the config is shared, make sure the protocol is not active on any interface.
	for (MqttClient *c = clients; c != nullptr; c = c->next)
	{
		if (c->responderState != ResponderState::free)
		{
			reply.copy("Unable to configure MQTT when active on an interface");
			return GCodeResult::error;
		}
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

		if (!setMemb(username))
		{
			return GCodeResult::error;
		}

		// Set password. Setting the password without the username shouldn't be
		// possible, so it's processed only if a username is also specified.
		if (gb.Seen('K'))
		{
			gb.GetQuotedString(param.GetRef());
			if (!setMemb(password))
			{
				clearMemb(username);
				return GCodeResult::error;
			}
		}
		else
		{
			clearMemb(password);
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Username set to '%s'", username);
			if (password)
			{
				debugPrintf("with password '%s'", password);
			}
			debugPrintf("\n");
		}
	}

	if (gb.Seen('C'))
	{
		// Set the client ID
		gb.GetQuotedString(param.GetRef());

		if (!setMemb(id))
		{
			return GCodeResult::error;
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Client ID set to '%s'\n", id);
		}
	}

	if (gb.Seen('W')) //  Will message and topic
	{
		// Set the will message
		gb.GetQuotedString(param.GetRef());

		if (!setMemb(willMessage))
		{
			return GCodeResult::error;
		}

		// Set will topic. Setting the will topic without the will message shouldn't be
		// possible, so it's processed only if a will message is also specified.
		gb.MustSee('T');
		{
			gb.GetQuotedString(param.GetRef());
			if (!setMemb(willTopic))
			{
				clearMemb(willMessage);
				return GCodeResult::error;
			}
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Will message set to '%s'", willMessage);
			if (willTopic)
			{
				debugPrintf("with topic '%s'", willTopic);
			}
			debugPrintf("\n");
		}
	}

	if (gb.Seen('S')) // Subscription
	{
		gb.GetQuotedString(param.GetRef());

		// Check the QOS first
		int qos = 0;
		if (gb.Seen('Q'))
		{
			qos = gb.GetIValue();
			if (qos < 0 || qos > 2)
			{
				reply.copy("Invalid subscription QOS");
				return GCodeResult::badOrMissingParameter;
			}
		}

		// Then check if the topic is already in the subscriptions,
		Subscription *sub;
		for (sub = subs; sub != nullptr; sub = sub->next)
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
				reply.copy("Unable to allocate mem");
				return GCodeResult::error;
			}

			SafeStrncpy(sub->topic, param.c_str(), sz);
			sub->qos = qos;

			// Append to list of subscriptions
			sub->next = subs;
			subs = sub;

			if (reprap.Debug(Module::Webserver))
			{
				debugPrintf("MQTT added topic %s with QOS %d to subscriptions\n", param.c_str(), qos);
			}
		}
	}

	if (gb.Seen('P')) // Publish
	{
		gb.GetQuotedString(param.GetRef());

		if (!setMemb(publishTopic))
		{
			return GCodeResult::error;
		}

		retain = duplicate = 0;
		publishQos = 0;

		if (gb.Seen('R'))
		{
			retain = gb.GetIValue();
		}

		if (gb.Seen('D'))
		{
			duplicate = gb.GetIValue();
		}

		if (gb.Seen('Q'))
		{
			int qos = gb.GetIValue();

			if (qos < 0 || qos > 2)
			{
				reply.copy("Invalid publish QOS");
				return GCodeResult::badOrMissingParameter;
			}
			else
			{
				publishQos = qos;
				if (reprap.Debug(Module::Webserver))
				{
					debugPrintf("MQTT publish QOS set to %d\n", publishQos);
				}
			}
		}

		if (reprap.Debug(Module::Webserver))
		{
			debugPrintf("Publish topic '%s', with settings duplicate = %d retain = %d qos = %d\n", publishTopic, retain, duplicate, publishQos);
		}
	}

	return GCodeResult::ok;
}

/* static */void MqttClient::Disable() noexcept
{
	// Nothing needed here
}

/* static */void MqttClient::Publish(const char *msg) noexcept
{
	for (MqttClient *c = clients; c != nullptr; c = c->next)
	{
		if (c->responderState == ResponderState::active)
		{
			if (strlen(msg) < OUTPUT_BUFFER_SIZE - 1)
			{
				MutexLocker lock(c->publishMutex);
				OutputBuffer *buf = nullptr;
				if (OutputBuffer::Allocate(buf))
				{
					buf->copy(msg);

					if (c->outBuf)
					{
						c->outBuf->Append(buf);
					}
					else
					{
						c->outBuf = buf;
					}
				}
			}
		};
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

/* Static members */
char *MqttClient::username = nullptr;
char *MqttClient::password = nullptr;
char *MqttClient::id = nullptr;
char *MqttClient::willTopic = nullptr;
char *MqttClient::willMessage = nullptr;
size_t MqttClient::keepAlive = MqttClient::DefaultKeepAlive;

char *MqttClient::publishTopic = nullptr;
uint8_t MqttClient:: publishQos = 0;
bool MqttClient::duplicate = false;
bool MqttClient::retain = false;

MqttClient::Subscription *MqttClient::subs = nullptr;
MqttClient *MqttClient::clients = nullptr;

#endif
