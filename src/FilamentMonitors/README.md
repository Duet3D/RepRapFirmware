# FilamentMonitors

`src/FilamentMonitors/` owns filament-presence and filament-motion monitoring. It links the configured monitor hardware to extrusion state so the firmware can detect runout, slips, and other feed-related faults.

## What This Module Owns

- filament monitor object types and their runtime state;
- the correlation between extrusion movement and monitor feedback;
- error detection and reporting related to missing or inconsistent filament movement.

## How It Works

Configured monitors are sampled as part of the firmware's regular control loop. The raw monitor signal is not enough on its own; the module compares it against what the motion subsystem expects the extruder to be doing, then decides whether the current print state is healthy.

That is why this directory sits at the intersection of machine inputs, motion, and print control rather than being a pure hardware-driver layer.

## Interfaces Within RepRapFirmware

- [../Movement/README.md](../Movement/README.md) supplies the extrusion-side reference behavior.
- [../GCodes/README.md](../GCodes/README.md) provides the configuration path.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes filament-monitor state and faults.

## DSF And Duet3Expansion Interfaces

- **DSF**: observes the resulting state and fault behavior through RRF's replies and object model.
- **Duet3Expansion**: supported monitor hardware can live on an expansion board and report back through the CAN path owned by RRF.

## Standalone Vs SBC

This module is common to both modes.

## Related Docs

- [../Movement/README.md](../Movement/README.md)
- [../CAN/README.md](../CAN/README.md)
- [../../docs/devel/GCODE_PROCESSING.md](../../docs/devel/GCODE_PROCESSING.md)
