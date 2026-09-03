# Config

`src/Config/` is the build-time board configuration layer. It tells the rest of RepRapFirmware what hardware it is being compiled for, which features are available, and how board-specific pins and capabilities should be described to the higher-level modules.

## Key Files

| File | Purpose |
|---|---|
| [Configuration.h](Configuration.h) | Shared configuration definitions and compile-time glue. |
| [Pins.h](Pins.h) / [Pins.cpp](Pins.cpp) | Common pin-description plumbing. |
| [Pins_Duet3_MB6HC.h](Pins_Duet3_MB6HC.h), [Pins_Duet3_MB6XD.h](Pins_Duet3_MB6XD.h), [Pins_Duet3Mini.h](Pins_Duet3Mini.h), [Pins_DuetNG.h](Pins_DuetNG.h), [Pins_FMDC.h](Pins_FMDC.h), [Pins_Pccb.h](Pins_Pccb.h), [Pins_Duet3_INDX.h](Pins_Duet3_INDX.h) | Per-board configuration tables and feature flags. |

## How It Works

The build system selects a target board, and that choice brings in one of the `Pins_*` headers. Those files define the processor family, driver counts, available peripherals, and the feature macros that determine whether modules such as SBC support, CAN expansion, networking backends, or a direct LCD are compiled in.

This directory is what keeps most board variation out of the main firmware logic. Higher-level modules query capabilities through the configured tables instead of scattering board-specific `#ifdef` blocks throughout the codebase.

## Interfaces Within RepRapFirmware

- [../Platform/README.md](../Platform/README.md) is the primary consumer.
- [../Hardware/README.md](../Hardware/README.md) depends on the processor-family and linker-side choices established here.
- build-variant-sensitive modules such as [../SBC/README.md](../SBC/README.md), [../CAN/README.md](../CAN/README.md), [../Networking/README.md](../Networking/README.md), and [../Display/README.md](../Display/README.md) are enabled or disabled by the feature flags defined here.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct runtime interface, but flags such as `HAS_SBC_INTERFACE` determine whether DSF can attach at all.
- **Duet3Expansion**: analogous role only. Both repos use per-board configuration headers to decide feature availability, but there is no direct runtime coupling.

## Standalone Vs SBC

This directory controls whether standalone-only and SBC-capable features are compiled in. The runtime mode still depends on the actual SBC handshake, but the capability exists only if the selected board configuration enables it.

## Related Docs

- [../../docs/devel/BUILD_VARIANTS.md](../../docs/devel/BUILD_VARIANTS.md)
- [../Platform/README.md](../Platform/README.md)
- [../Hardware/README.md](../Hardware/README.md)
