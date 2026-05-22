# CAN

`src/CAN/` is the main-board CAN-FD subsystem. It turns RepRapFirmware into the bus master for Duet3Expansion boards, handling board discovery, time synchronisation, remote motion, remote heaters and sensors, and the command/reply traffic that makes expansion boards look like extensions of the main firmware.

## Key Files

| File | Purpose |
|---|---|
| [CanInterface.cpp](CanInterface.cpp) | Core CAN-FD transport and queue ownership on the main board. |
| [ExpansionManager.cpp](ExpansionManager.cpp) | Tracks known remote boards and their reported state. |
| [CanMotion.cpp](CanMotion.cpp) | Packs motion fragments for remote execution. |
| [CommandProcessor.h](CommandProcessor.h) | Declares the handling surface for higher-level CAN messages. |

## How It Works

RRF owns the only full machine view. When a configuration or motion decision involves a remote board, this module turns that decision into CANlib-defined messages and sends them to the right address. The reverse direction is equally important: expansion boards stream back replies, sensor data, input changes, board announcements, and status updates that `ExpansionManager` folds into the main board's state.

The other crucial job is time alignment. Remote moves only work because the main board distributes the timing information needed for expansion boards to schedule their own step output in lockstep with local drivers.

## Interfaces Within RepRapFirmware

- [../Movement/README.md](../Movement/README.md) uses this module to split motion between local and remote drivers.
- [../Heating/README.md](../Heating/README.md), [../Fans/README.md](../Fans/README.md), and [../InputMonitors/README.md](../InputMonitors/README.md) use it to reach remote devices.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes remote-board state via the `boards[]` and related subtrees.
- [../GCodes/README.md](../GCodes/README.md) is the user-facing entry path for configuration that ultimately creates remote resources.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct CAN ownership. DSF only sees CAN-derived machine state after RRF merges it into the object model or returns it through diagnostics.
- **Duet3Expansion**: this module is the master-side peer of Duet3Expansion's `src/CAN/` and `src/CommandProcessing/` modules. The shared on-wire contract lives in CANlib.

## Standalone Vs SBC

CAN expansion is orthogonal to standalone vs SBC mode. The same CAN subsystem runs in both; the only difference is whether the control surface above RRF is local networking or DSF over SPI.

## Related Docs

- [../../docs/devel/CAN_BUS.md](../../docs/devel/CAN_BUS.md)
- [../../docs/devel/ARCHITECTURE.md](../../docs/devel/ARCHITECTURE.md)
- [Duet3Expansion/docs/devel/CAN_PROTOCOL.md](https://github.com/Duet3D/Duet3Expansion/blob/3.7-docker/docs/devel/CAN_PROTOCOL.md)
