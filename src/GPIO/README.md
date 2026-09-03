# GPIO

`src/GPIO/` provides the general-purpose input and output abstractions used when a user configures pins as machine-facing I/O instead of as one of the more specialised resources. It is the smallest of the top-level modules, but it matters because it decouples user-configurable I/O from board-specific register handling.

## Key Files

| File | Purpose |
|---|---|
| [GpInPort.cpp](GpInPort.cpp) / [GpInPort.h](GpInPort.h) | General-purpose input-port abstraction. |
| [GpOutPort.cpp](GpOutPort.cpp) / [GpOutPort.h](GpOutPort.h) | General-purpose output-port abstraction. |

## How It Works

The module wraps configured input and output pins in objects with behavior that higher-level code can reason about consistently. That makes it possible to configure and use GPIO without forcing the G-code layer to know about processor-family register details or pin-table internals.

GPIO is also one of the recurring boundaries between local and remote hardware: a configured GPIO might be consumed locally or represented on a CAN board through a higher-level abstraction.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) is the entry path for configuring or using general-purpose ports.
- [../Platform/README.md](../Platform/README.md) supplies the underlying pin and timer resources.
- [../InputMonitors/README.md](../InputMonitors/README.md) is adjacent on the input side for machine-observed signals.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct ownership. DSF reaches GPIO only by sending the same commands a local front end would send.
- **Duet3Expansion**: remote GPIO functionality is ultimately carried over the [../CAN/README.md](../CAN/README.md) path when the resource lives on an expansion board.

## Standalone Vs SBC

This module is common to both modes.

## Related Docs

- [../Platform/README.md](../Platform/README.md)
- [../InputMonitors/README.md](../InputMonitors/README.md)
- [../CAN/README.md](../CAN/README.md)
