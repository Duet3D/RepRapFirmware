# Comms

`src/Comms/` contains the serial- and device-oriented communication helpers that are not part of the standalone network stack. This includes AUX/PanelDue-adjacent paths, USB CDC support, and firmware-update utilities.

## Key Files

| File | Purpose |
|---|---|
| [AuxDevice.cpp](AuxDevice.cpp) | AUX serial-device support. |
| [UsbDeviceRrf.cpp](UsbDeviceRrf.cpp) | Firmware-side USB CDC behavior. |
| [FirmwareUpdater.cpp](FirmwareUpdater.cpp) | Firmware-update support helpers. |
| [PanelDueUpdater.cpp](PanelDueUpdater.cpp) | PanelDue update support. |

## How It Works

This module packages the lower-level comms paths that sit beside the main G-code inputs instead of replacing them. AUX and USB can both feed or receive firmware messages, and the updater helpers manage specific device-update flows that do not belong in the core motion or network modules.

The important architectural distinction is that `Comms` is about attached serial or device channels, whereas [../Networking/README.md](../Networking/README.md) is about standalone TCP/IP services and [../SBC/README.md](../SBC/README.md) is about the SBC link.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) consumes text input arriving through these channels.
- [../Platform/README.md](../Platform/README.md) owns the lower-level routing, buffering, and hardware setup these helpers rely on.
- [../Display/README.md](../Display/README.md) and PanelDue-related workflows are adjacent consumers of the AUX path.

## DSF And Duet3Expansion Interfaces

- **DSF**: in SBC deployments, DSF becomes the main external control surface, so the importance of local serial and device-facing comms is reduced. The module itself still has no direct DSF protocol role.
- **Duet3Expansion**: no direct interface.

## Standalone Vs SBC

This module is mode-dependent. USB/AUX-style local interactions matter primarily in standalone-style deployments or board-local maintenance paths, while SBC mode shifts the main user-facing workflow onto DSF.

## Related Docs

- [../../docs/devel/GCODE_PROCESSING.md](../../docs/devel/GCODE_PROCESSING.md)
- [../Networking/README.md](../Networking/README.md)
- [../SBC/README.md](../SBC/README.md)
