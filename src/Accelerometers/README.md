# Accelerometers

`src/Accelerometers/` contains the firmware-side support for accelerometer devices used in resonance measurement and motion-tuning workflows. In practice this is mostly about collecting samples from supported sensors and making them available to higher-level tuning flows rather than participating in the normal print-control loop.

## Key Files

| File | Purpose |
|---|---|
| [Accelerometers.cpp](Accelerometers.cpp) / [Accelerometers.h](Accelerometers.h) | High-level accelerometer management entry points. |
| [LISAccelerometer.cpp](LISAccelerometer.cpp) / [LISAccelerometer.h](LISAccelerometer.h) | LIS3DH-specific implementation details. |

## How It Works

The module abstracts supported accelerometer hardware behind a small management layer. RRF can configure the sensor, collect bursts of samples, and hand the results to the parts of the system that need motion-characterisation data.

The interesting architectural point is not the sensor read itself, but the fact that accelerometer capture sits at the boundary between motion tuning, diagnostics, and cross-board telemetry. Local boards can sample directly; expansion boards can stream sample data back over CAN for the main board to interpret.

## Interfaces Within RepRapFirmware

- [../Movement/README.md](../Movement/README.md) uses the resulting data for input-shaping and resonance-related workflows.
- [../GCodes/README.md](../GCodes/README.md) provides the user-facing control path for configuring or triggering measurement.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes the resulting state and measurements where needed.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct transport or control ownership. DSF observes the resulting state through the object model and any code path that invokes the measurement workflow.
- **Duet3Expansion**: boards that carry an accelerometer can capture locally and report results over the CAN path owned by RRF's [../CAN/README.md](../CAN/README.md).

## Standalone Vs SBC

This module is common to both standalone and SBC deployments. The control surface changes, but the accelerometer-support logic on the firmware side does not.

## Related Docs

- [../../docs/devel/MOTION_PIPELINE.md](../../docs/devel/MOTION_PIPELINE.md)
- [../Movement/README.md](../Movement/README.md)
- [../CAN/README.md](../CAN/README.md)
