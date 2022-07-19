# Summary

The protocol uses multicast IP address 239.255.2.3 and default port number 10002. As with other network protocols supported by RRF, the port number can be changed in the M586 command.

# Availability

The protocol is implemented only on the Ethernet interface of Duet 3 MB6HC and MB6XD boards. It is implemented in RRF 3.4.2 and later [to be confirmed].

# Enabling and disabling the protocol

The protocol is disabled in RRF by default. To enable the protocol, use this command:

```M586 P3 S1```

This command can be included in config.g if support for the protocol is always required. When this command is run, RAM will be allocated for the multicast protocol handler.

To disable the protocol, use:

```M586 P3 S0```

Running this command does not release the RAM occupied by the protocol handler.

# Implementing the DNETINF command

The DNETINF command allows the IP address, netmask and gateway UP address to be changed as from the next reboot. If support for this command is required then the configuration files on the SD card must adhere to the following convention:

- File sys/config.g must include the following lines:
```
M98 P"network-override.g"
M552 S1
```
- After these commands there must not be any M550, M552, M553 or M554 commands
- Optionally, create an initial file sys/network-override.g containing M550, M552, M553 and M554 commands to set default device name, IP address etc. In the absence of file network-override.g, DHCP will be used.

When the DNETINF multicast command is received, file sys/network-override.g will be re-created with M550, M552, M553 and M554 commands to set the requested device name, IP address, netmask and gateway IP address.

Note, if the DHCP flag is set in the DNETINF command then the received IP address, netmask and gateway address will be ignored and the IP address etc. will be set to zero instead. This is because RRF uses a zero IP address to indicate that DHCP should be used.
