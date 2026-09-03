# DuetNG

`src/DuetNG/` contains legacy support for the Duet 2-era expansion hardware, particularly the DueXn and SX1509-backed I/O expansion path. It is not central to Duet 3 development, but it remains part of the firmware tree for the builds that still support those boards.

## Key Files

| File | Purpose |
|---|---|
| [DueXn.cpp](DueXn.cpp) / [DueXn.h](DueXn.h) | Support for DueX-style expansion hardware. |
| [SX1509.cpp](SX1509.cpp) / [SX1509.h](SX1509.h) | SX1509 GPIO-expander support. |
| [SX1509Registers.h](SX1509Registers.h) | Register-level definitions for the expander device. |

## How It Works

This module adapts legacy Duet expansion hardware into the same broader firmware model used elsewhere. The firmware still owns the machine-level behavior; `DuetNG` simply provides the hardware-specific bridge needed for those older boards.

Because it represents legacy hardware rather than the current Duet 3 CAN architecture, its main documentation value is preserving how those builds are wired into the rest of the system.

## Interfaces Within RepRapFirmware

- [../Platform/README.md](../Platform/README.md) uses the hardware support offered here.
- [../Config/README.md](../Config/README.md) selects whether this support is relevant for the build.
- machine-facing modules consume the resulting I/O capacity through the normal platform abstractions rather than through direct `DuetNG` awareness.

## DSF And Duet3Expansion Interfaces

- **DSF**: none directly.
- **Duet3Expansion**: none directly; `DuetNG` is the legacy predecessor to the Duet 3 CAN-expansion architecture, not a peer of it.

## Standalone Vs SBC

This module is independent of the standalone-vs-SBC split. Its presence depends on the target hardware family rather than the front-end mode.

## Related Docs

- [../../docs/devel/BUILD_VARIANTS.md](../../docs/devel/BUILD_VARIANTS.md)
- [../Config/README.md](../Config/README.md)
- [../Platform/README.md](../Platform/README.md)
