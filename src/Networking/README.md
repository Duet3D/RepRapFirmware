# Networking

`src/Networking/` is RRF's standalone network stack and responder layer. It owns HTTP, FTP, Telnet, MQTT, multicast discovery, and the interface-specific drivers that expose those services on Ethernet or WiFi when the firmware is operating without DSF as the front end.

## Key Files And Areas

| Area | Purpose |
|---|---|
| [Network.cpp](Network.cpp) / [Network.h](Network.h) | Top-level network subsystem. |
| [HttpResponder.cpp](HttpResponder.cpp) | Standalone HTTP server and `rr_*` API implementation. |
| [FtpResponder.cpp](FtpResponder.cpp), [TelnetResponder.cpp](TelnetResponder.cpp) | Standalone file-transfer and interactive console responders. |
| [MQTT/](MQTT) | MQTT support. |
| [LwipEthernet/](LwipEthernet), [W5500Ethernet/](W5500Ethernet), [ESP8266WiFi/](ESP8266WiFi) | Interface-specific transport implementations. |
| [MulticastDiscovery/](MulticastDiscovery) | Discovery and multicast support. |

## How It Works

In standalone mode the networking subsystem is the user-facing front end for the firmware. It accepts HTTP requests, file transfers, Telnet sessions, and related network traffic, then routes the resulting control actions into the same internal machine-control modules that every other front end uses.

The directory is therefore a front-end layer, not an ownership layer for motion or thermal behavior. Its job is to translate network-facing protocols into firmware actions and data.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) receives most user-issued codes coming in over the network.
- [../ObjectModel/README.md](../ObjectModel/README.md) supplies status and model data to the HTTP responders.
- [../Storage/README.md](../Storage/README.md) handles file operations initiated over the network.
- [../Platform/README.md](../Platform/README.md) provides interface setup, buffers, and low-level support.

## DSF And Duet3Expansion Interfaces

- **DSF**: this is one of the clearest mode splits in the whole firmware. In SBC mode DSF's `DuetWebServer` replaces this directory as the main network front end, while still mirroring the `rr_*` behavior for compatibility.
- **Duet3Expansion**: no direct interface.

## Standalone Vs SBC

This module is standalone-oriented. In SBC mode its responsibilities are largely displaced by DSF even though some compatibility behaviors still depend on matching semantics.

## Related Docs

- [../../docs/devel/NETWORKING.md](../../docs/devel/NETWORKING.md)
- [../../docs/devel/STANDALONE_VS_SBC.md](../../docs/devel/STANDALONE_VS_SBC.md)
- [DuetSoftwareFramework/src/DuetWebServer/README.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/src/DuetWebServer/README.md)
