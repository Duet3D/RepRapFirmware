# SBC

`src/SBC/` is the firmware-side implementation of the RepRapFirmware-to-DSF link. It owns the SPI or USB transport framing, packet handling, object-model exchange, file-operation proxying, and the dedicated firmware channel that lets DSF act as the front end for an attached Linux SBC.

## Key Files

| File | Purpose |
|---|---|
| [SbcInterface.cpp](SbcInterface.cpp) / [SbcInterface.h](SbcInterface.h) | Main SBC-link state machine and task-facing logic. |
| [DataTransfer.cpp](DataTransfer.cpp) / [DataTransfer.h](DataTransfer.h) | Transfer-buffer handling and packet plumbing. |
| [SbcMessageFormats.h](SbcMessageFormats.h) | Wire-format definitions and protocol constants. |

## How It Works

When an SBC is present, this module becomes the front-door transport between RRF and DSF. It receives firmware-facing work from DCS, exposes the dedicated `SBC` G-code channel, proxies filesystem requests to DSF, and returns replies and object-model state back across the link.

The runtime mode split in RRF is most visible here. The same firmware image can run without an SBC, but when this link comes up it changes who owns the network, file-system, and browser-facing control surface.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) consumes the `SBC` channel.
- [../Storage/README.md](../Storage/README.md) uses the file-operation proxy path in SBC mode.
- [../ObjectModel/README.md](../ObjectModel/README.md) supplies the replicated machine state exported to DSF.
- [../Platform/README.md](../Platform/README.md) provides the low-level hardware support for the transport.

## DSF And Duet3Expansion Interfaces

- **DSF**: this is the direct peer of DSF's `DuetControlServer` link and protocol code. It is the main board's half of the DSF integration.
- **Duet3Expansion**: no direct interface. Expansion boards remain behind RRF on the CAN bus.

## Standalone Vs SBC

This module is the firmware-side definition of SBC mode. In standalone mode it stays dormant; in SBC mode it is one of the most important modules in the entire firmware.

## Related Docs

- [../../docs/devel/SBC_INTERFACE.md](../../docs/devel/SBC_INTERFACE.md)
- [../../docs/devel/STANDALONE_VS_SBC.md](../../docs/devel/STANDALONE_VS_SBC.md)
- [DuetSoftwareFramework/src/DuetControlServer/README.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/src/DuetControlServer/README.md)
