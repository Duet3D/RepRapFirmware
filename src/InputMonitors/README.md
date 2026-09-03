# InputMonitors

`src/InputMonitors/` provides the abstraction used to observe local and remote inputs in a uniform way. It is the bridge that lets endstops, probes, triggers, and similar signal-driven behaviors work even when the physical input lives on a CAN-attached board.

## Key Files

| File | Purpose |
|---|---|
| [InputMonitor.cpp](InputMonitor.cpp) / [InputMonitor.h](InputMonitor.h) | Main input-monitor abstraction on the RRF side. |

## How It Works

Instead of making every consumer understand where a signal physically lives, the firmware creates input-monitor objects that represent the inputs it cares about. Local hardware and remote CAN-backed signals are then surfaced through a consistent higher-level interface.

This is a good example of how RRF keeps the user-facing machine model stable even when the hardware is distributed across multiple boards.

## Interfaces Within RepRapFirmware

- [../Endstops/README.md](../Endstops/README.md) is the most direct consumer.
- [../GCodes/README.md](../GCodes/README.md) configures or triggers behaviors tied to monitored inputs.
- [../CAN/README.md](../CAN/README.md) transports remote input changes from expansion boards.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes resulting state where appropriate.

## DSF And Duet3Expansion Interfaces

- **DSF**: DSF does not own input monitoring, but it does consume the resulting machine state and diagnostics.
- **Duet3Expansion**: this module is the main-board peer of Duet3Expansion's own input-monitoring and command-processing support.

## Standalone Vs SBC

This module is common to both modes.

## Related Docs

- [../Endstops/README.md](../Endstops/README.md)
- [../CAN/README.md](../CAN/README.md)
- [../../docs/devel/CAN_BUS.md](../../docs/devel/CAN_BUS.md)
