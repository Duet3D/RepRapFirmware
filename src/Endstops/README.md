# Endstops

`src/Endstops/` owns endstop and probe abstraction on the main firmware side. It lets higher-level motion and homing logic reason about a unified set of stop and probe inputs regardless of whether the signal is local to the main board or proxied in from a CAN-attached board.

## What This Module Owns

- endstop and Z-probe objects and their lifetime;
- the mapping between user configuration and the runtime stop/probe state seen by motion and homing logic;
- the firmware-side abstraction that hides whether an input is local or remote.

## How It Works

Configuration creates or updates endstop and probe objects with the semantics needed by the active machine setup. During homing, probing, and trigger handling, the rest of the firmware asks this module for the current state instead of reading hardware pins directly.

That separation matters because the input may come from the main board, a CAN expansion board, or a higher-level probe abstraction that has additional behavior layered on top of a simple digital input.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) is the control/configuration entry path.
- [../Movement/README.md](../Movement/README.md) depends on this module during homing and probing workflows.
- [../InputMonitors/README.md](../InputMonitors/README.md) provides related infrastructure for local and remote inputs.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes the resulting state.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct protocol ownership. DSF influences the module only by sending or storing the configuration G-codes and observing the resulting state.
- **Duet3Expansion**: remote endstops and probes reach the main board through the CAN/input-monitor path and are consumed here as part of the unified abstraction.

## Standalone Vs SBC

This module is common to both modes. Only the configuration/control surface differs.

## Related Docs

- [../../docs/devel/GCODE_PROCESSING.md](../../docs/devel/GCODE_PROCESSING.md)
- [../InputMonitors/README.md](../InputMonitors/README.md)
- [../Movement/README.md](../Movement/README.md)
