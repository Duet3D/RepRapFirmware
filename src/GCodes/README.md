# GCodes

`src/GCodes/` is the command-processing hub of RepRapFirmware. It receives codes from all active channels, parses them, maintains per-channel execution state, dispatches them to the owning subsystem, and routes replies back to the originating channel.

## Key Files

| File | Purpose |
|---|---|
| [GCodes.cpp](GCodes.cpp) / [GCodes.h](GCodes.h) | Main scheduler and control flow for G-code processing. |
| [GCodes2.cpp](GCodes2.cpp) through [GCodes7.cpp](GCodes7.cpp) | Large dispatch tables and command handlers. |
| [GCodeBuffer/](GCodeBuffer) | Per-channel parser and execution context. |
| [GCodeMachineState.h](GCodeMachineState.h) | Per-channel stack frame state for nested execution. |
| [GCodeChannel.h](GCodeChannel.h) | Channel definitions shared with the rest of the system. |

## How It Works

Each input source gets its own `GCodeBuffer`, which means macros, interactive commands, file execution, trigger handling, and SBC-fed codes can all progress independently without corrupting each other's parser or machine state. `GCodes::Spin()` walks the active channels, parses what is ready, and dispatches the resulting command to the module that owns the underlying behavior.

Long-running or stateful operations stay cooperative by returning a not-finished result and being resumed later. That makes `GCodes` the orchestration layer for a large amount of firmware behavior without turning it into a blocking controller.

## Interfaces Within RepRapFirmware

Everything user-visible eventually passes through this module. It dispatches into [../Movement/README.md](../Movement/README.md), [../Heating/README.md](../Heating/README.md), [../Networking/README.md](../Networking/README.md), [../Storage/README.md](../Storage/README.md), [../CAN/README.md](../CAN/README.md), [../SBC/README.md](../SBC/README.md), [../Tools/README.md](../Tools/README.md), and [../Platform/README.md](../Platform/README.md).

It also relies heavily on [../ObjectModel/README.md](../ObjectModel/README.md) for expressions, variables, and state-dependent logic.

## DSF And Duet3Expansion Interfaces

- **DSF**: the SBC channel is the key DSF integration point. DCS feeds pre-tokenised binary codes into RRF through this module, and many DSF tools or services ultimately exist to drive or observe that path.
- **Duet3Expansion**: expansion boards do not parse text G-code. Instead, this module configures and drives RRF-side behavior that later becomes CAN messages headed toward Duet3Expansion.

## Standalone Vs SBC

`GCodes` is common to both modes, but the active channel sources differ substantially. In standalone mode HTTP, Telnet, and local file execution feed directly into RRF channels. In SBC mode those front ends are owned by DSF and arrive through the dedicated `SBC` channel.

## Related Docs

- [../../docs/devel/GCODE_PROCESSING.md](../../docs/devel/GCODE_PROCESSING.md)
- [../SBC/README.md](../SBC/README.md)
- [../Movement/README.md](../Movement/README.md)
- [../ObjectModel/README.md](../ObjectModel/README.md)
