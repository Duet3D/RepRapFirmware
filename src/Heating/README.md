# Heating

`src/Heating/` is the heater and temperature-control subsystem. It owns temperature sensors, heater models, PID control, autotuning, tool temperature state, and the safety logic around thermal faults and cold extrusion.

## Key Files

| File or area | Purpose |
|---|---|
| [Heat.cpp](Heat.cpp) / [Heat.h](Heat.h) | Top-level heating subsystem. |
| [Heater.cpp](Heater.cpp) / [Heater.h](Heater.h) | Per-heater runtime behavior. |
| [Sensors/](Sensors) | Sensor implementations and abstraction. |
| [FOPDT.cpp](FOPDT.cpp) | Model-fitting support used for tuning. |

## How It Works

The heating subsystem creates and owns the machine's sensor and heater objects, periodically samples the relevant inputs, and computes the output behavior needed to track the requested temperatures safely. It is intentionally separated from `Tools`: a tool can depend on one or more heaters, but the generic control problem lives here.

This module is also where local and remote thermal hardware are unified. From the machine's point of view a heater or sensor is a heater or sensor; the transport to a CAN board is handled below this layer.

## Interfaces Within RepRapFirmware

- [../Tools/README.md](../Tools/README.md) consumes heater resources when defining tools or spindles.
- [../Fans/README.md](../Fans/README.md) uses sensor state for thermostatic behavior.
- [../GCodes/README.md](../GCodes/README.md) is the configuration and control entry path.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes thermal state to the outside world.
- [../CAN/README.md](../CAN/README.md) carries remote thermal resources.

## DSF And Duet3Expansion Interfaces

- **DSF**: DSF does not own thermal control, but DCS and DWS expose and drive it by feeding RRF the relevant G-codes and reading the object model.
- **Duet3Expansion**: remote heaters and sensors live on expansion boards and are represented here through the main-board CAN integration.

## Standalone Vs SBC

This module is common to both modes. The only change is where the incoming control commands originate.

## Related Docs

- [../../docs/devel/HEATING.md](../../docs/devel/HEATING.md)
- [../Tools/README.md](../Tools/README.md)
- [../Fans/README.md](../Fans/README.md)
- [../CAN/README.md](../CAN/README.md)
