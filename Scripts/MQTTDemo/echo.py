# Runs the MQTT broker and echo client. See README.md for more information
# about the demonstration this Python script is a part of.

import paho.mqtt.client as mqtt
import subprocess
import threading
import time

from colorama import Fore
from colorama import Style

HOST = "localhost"
PORT = 1884
USERNAME = "test-echo"
PASSWORD = "test-echo-pswd"
CLIENT_ID = "echo"
SUBSCRIBE_TOPIC = "topic-duet"
PUBLISH_TOPIC = "topic-echo"

def broker():
    # Start the Mosquitto broker in verbose mode, using the config file.
    subprocess.run(["mosquitto", "-v", "-c", "mosquitto.conf"])

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"{Fore.YELLOW}echo: connect succeeded, subscribing to topic '{SUBSCRIBE_TOPIC}'...{Style.RESET_ALL}")
        client.subscribe(SUBSCRIBE_TOPIC)
    else:
        print(f"{Fore.YELLOW}echo: connect failed, result code: {rc}{Style.RESET_ALL}")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(f"{Fore.YELLOW}echo: received message with topic '{msg.topic}': '{msg.payload}', echoing... {Style.RESET_ALL}")
    res = client.publish(PUBLISH_TOPIC, msg.payload, msg.qos, msg.retain)

    if res[0] == 0:
        print(f"{Fore.YELLOW}echo: echo succeeded {Style.RESET_ALL}")
    else:
        print(f"{Fore.YELLOW}echo: echo failed, result code: ${res[0]}{Style.RESET_ALL}")

def on_subscribe(client, userdata, mid, granted_qos):
    print(f"{Fore.YELLOW}echo: subscribe succeeded, waiting for messages...{Style.RESET_ALL}")

def echo():
    client = mqtt.Client(CLIENT_ID)
    client.username_pw_set(USERNAME, PASSWORD)

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe

    print(f"{Fore.YELLOW}echo: connecting to '{HOST}' on port '{PORT}' as '{CLIENT_ID}'{Style.RESET_ALL}...")
    client.connect(HOST, PORT)
    client.loop_forever()

def main():
    broker_thread = threading.Thread(target=broker)
    echo_thread = threading.Thread(target=echo)

    broker_thread.start()
    time.sleep(1) # make sure the broker is up
    echo_thread.start()

if __name__ == '__main__':
    main()