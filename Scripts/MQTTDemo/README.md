
# Requirements

- [Python](https://www.python.org/downloads/) - runs the demo script, `echo.py`, which runs broker and `echo` client
    - [Paho](https://www.eclipse.org/paho/index.php?page=clients/python/index.php) - used to implement the `echo` MQTT client
    - [Colorama](https://pypi.org/project/colorama/) - used to colorize log output of `echo.py`

- [Mosquitto](https://mosquitto.org/download/) - MQTT broker used in the demo

# Overview

## Setup

The demo has three components: the MQTT broker, the `echo` MQTT client and the RRF MQTT client.

The RRF MQTT client publishes message sent via `M118` under a topic `topic-duet`.
The `echo` MQTT client is subscribed to this topic, which retransmits the message under the topic `topic-echo`. Since the RRF MQTT client in turn is subscribed to this topic, it receives and displays the retransmitted message.

## Broker Configuration

Broker configuration can be found in [mosquitto.conf](./mosquitto.conf). This configuration:
- disallows anonymous clients, allowing only clients with authentication credentials to connect
- specifies the password file, [passwords.txt](./passwords.txt), whose contents are the allowed client usernames and the corresponding password hashes
- runs the MQTT broker on a different port, 1884 instead of the typical 1883

## Password File

The clear text contents of the password file [passwords.txt](./passwords.txt) are as follows:

```
test-echo:test-echo-pswd
test-duet:test-duet-pswd
```

Running the command `mosquitto_passwd -U passwords.txt` on the clear text contents will replace the password part (the text after the colon on each row) with its hash.

The first row are the credentials for the `echo` client; the second are the credentials for the RRF MQTT client.


# Running the Demo

## Host

Open a command line/terminal and `cd` into this directory, then run the command below.

```
python echo.py
```

Running the [GCode commands on RepRapFirmware](#reprapfirmware), a log similar to the following one should be seen.

```
1671016326: mosquitto version 2.0.15 starting
1671016326: Config loaded from mosquitto.conf.
1671016326: Opening ipv4 listen socket on port 1884.
1671016326: Opening ipv6 listen socket on port 1884.
1671016326: mosquitto version 2.0.15 running
echo: connecting to 'localhost' on port '1884' as 'echo'...
1671016327: New connection from ::1:39785 on port 1884.
1671016327: New client connected from ::1:39785 as echo (p2, c1, k60, u'test-echo').
1671016327: No will message specified.
1671016327: Sending CONNACK to echo (0, 0)
echo: connect succeeded, subscribing to topic 'topic-duet'...
1671016327: Received SUBSCRIBE from echo
1671016327:     topic-duet (QoS 0)
1671016327: echo 0 topic-duet
1671016327: Sending SUBACK to echo
echo: subscribe succeeded, waiting for messages...
1671016359: New connection from 192.168.10.125:60423 on port 1884.
1671016359: New client connected from 192.168.10.125:60423 as duet (p2, c1, k400, u'test-duet').
1671016359: No will message specified.
1671016359: Sending CONNACK to duet (0, 0)
1671016359: Received SUBSCRIBE from duet
1671016359:     topic-echo (QoS 0)
1671016359: duet 0 topic-echo
1671016359: Sending SUBACK to duet
1671016365: Received PUBLISH from duet (d0, q0, r0, m0, 'topic-duet', ... (12 bytes))
1671016365: Sending PUBLISH to echo (d0, q0, r0, m0, 'topic-duet', ... (12 bytes))
echo: received message with topic 'topic-duet': 'b'duet-message\n'', echoing...
echo: echo succeeded
1671016365: Received PUBLISH from echo (d0, q0, r0, m0, 'topic-echo', ... (12 bytes))
1671016365: Sending PUBLISH to duet (d0, q0, r0, m0, 'topic-echo', ... (12 bytes))
1671016372: Received DISCONNECT from duet
1671016372: Client duet disconnected.

```


## RepRapFirmware

The WiFi interface must be configured and enabled.

### Enable debugging messages (optional)

```
M111 P2 S1
```

### Configure the MQTT client

```
M586.4 C"duet"
M586.4 U"test-duet" K"test-duet-pswd"
M586.4 P"topic-duet" D0 R0 Q0
M586.4 S"topic-echo" Q0
```

- `C` - Client ID
- `U`, `K` - Username and password ()
- `S`, `Q` - Subcription topic and corresponding QOS
- `P`, `D`, `R`, `Q` - Publish settings: topic, duplicate flag, retain flag, QOS
### Enable the MQTT protocol

```
M586 P4 R1884 H192.168.10.244 S1
```

### Publish a message via M118

```
M118 P6 S"duet-message"
```

This message will be echoed back by the `echo` client, under topic `topic-echo`.
Since the RRF MQTT client is subscribed to this topic, it should receive that message:

```
Received message from topic 'topic-echo': 'duet-message'
```

### Disable the MQTT Protocol

```
M586 P4 S0
```
# Scenarios


1. MQTT protocol is enabled before `echo.py` can be started. This means the broker is not yet started when the RRF MQTT client attempts to connect.

    - This should be ok, as the MQTT client will attempt reconnection automatically.

2. The running `echo.py` is terminated while RRF MQTT client is connected.

    - This should be ok, as the MQTT client will attempt reconnection automatically.

3. MQTT protocol is configured and enabled before starting the network interface via `M552 S1`.

    - This should be ok, the MQTT client will only attempt connection once the network interface is active.

4. MQTT client is configured while connecting/connected.

    - This is not ok, and should results in an error GCode result. Configuration is
        only possible while the protocol is disabled.
