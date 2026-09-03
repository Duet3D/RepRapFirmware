# Tools

`src/Tools/` contains the higher-level abstractions for tools, spindles, and filament ownership. It is where multiple lower-level resources such as heaters, fans, and extruders are grouped into the machine concepts that users actually select or activate.

## Key Files

| File | Purpose |
|---|---|
| [Tool.cpp](Tool.cpp) / [Tool.h](Tool.h) | Main tool abstraction. |
| [Spindle.cpp](Spindle.cpp) / [Spindle.h](Spindle.h) | Spindle-specific support. |
| [Filament.cpp](Filament.cpp) / [Filament.h](Filament.h) | Filament ownership and related tool-level state. |

## How It Works

This module sits above the raw heater, fan, and movement layers. It lets the firmware reason about a tool as a coordinated collection of resources rather than as unrelated low-level devices.

That is why tool changes and tool activation can remain high-level operations even though the actual implementation work is spread across several other modules.

## Interfaces Within RepRapFirmware

- [../Heating/README.md](../Heating/README.md) supplies heaters and temperature behavior.
- [../Movement/README.md](../Movement/README.md) supplies extruder and motion-side coordination.
- [../Fans/README.md](../Fans/README.md) supplies part-cooling or tool-adjacent fan behavior.
- [../GCodes/README.md](../GCodes/README.md) is the user-facing entry path.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes tool state.

## DSF And Duet3Expansion Interfaces

- **DSF**: DSF interacts through normal tool-related G-codes and object-model state; it does not own tool behavior itself.
- **Duet3Expansion**: tools often depend on resources hosted on expansion boards, so this module is one of the main higher-level consumers of the CAN-backed device model.

## Standalone Vs SBC

This module is common to both modes.

## Related Docs

- [../Heating/README.md](../Heating/README.md)
- [../Movement/README.md](../Movement/README.md)
- [../CAN/README.md](../CAN/README.md)
