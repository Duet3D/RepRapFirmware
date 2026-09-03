# Fans

`src/Fans/` contains the fan-control subsystem. It owns fan objects, thermostatic behavior, tachometer processing, and the distinction between local fan outputs and remote fan resources that live on CAN expansion boards.

## Key Files

| File | Purpose |
|---|---|
| [FansManager.cpp](FansManager.cpp) / [FansManager.h](FansManager.h) | Global fan manager and periodic control logic. |
| [Fan.cpp](Fan.cpp) / [Fan.h](Fan.h) | Per-fan runtime state and behavior. |

## How It Works

Fans are configured once and then managed as part of the firmware's regular housekeeping. Each fan can be treated as a simple controllable output or as a thermostatic resource that reacts to heater and sensor state.

The manager periodically reevaluates what each fan should be doing, applies any startup or safety logic, and routes the result either to local hardware or over CAN to the board that actually owns the fan output.

## Interfaces Within RepRapFirmware

- [../Heating/README.md](../Heating/README.md) provides sensor and heater state for thermostatic logic.
- [../GCodes/README.md](../GCodes/README.md) is the configuration and direct-control entry path.
- [../Platform/README.md](../Platform/README.md) provides the local PWM and input resources used for fans.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes fan state to external consumers.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct transport ownership. DSF sends the normal fan-related codes and reads fan state through the object model.
- **Duet3Expansion**: remote fans are hosted on expansion boards and controlled over the main-board [../CAN/README.md](../CAN/README.md) path.

## Standalone Vs SBC

This module is common to both modes. The fan-control logic is local firmware behavior either way.

## Related Docs

- [../../docs/devel/HEATING.md](../../docs/devel/HEATING.md)
- [../Heating/README.md](../Heating/README.md)
- [../CAN/README.md](../CAN/README.md)
