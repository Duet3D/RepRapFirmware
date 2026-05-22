# Platform

`src/Platform/` is the main hardware-abstraction and runtime-orchestration layer above the low-level processor-family code. It owns major housekeeping loops, task setup, watchdog integration, message routing, power and voltage monitoring, and many of the services that other modules treat as generic board capabilities.

## What This Module Owns

- the main cooperative runtime scaffolding around `RepRap::Spin()`;
- task priorities and the relationship between the cooperative loop and dedicated RTOS tasks;
- low-level machine services that do not belong in motion, heating, or networking directly;
- the abstraction boundary between module logic and board-specific hardware details.

## How It Works

Platform is where the firmware becomes a running system rather than a collection of modules. It provides the recurring service calls that the rest of the firmware expects, coordinates shared hardware resources, and maintains the book-keeping needed to keep a real-time machine stable.

It is intentionally broad, but not because it owns every behavior directly. Many capabilities live elsewhere; `Platform` exists to give them consistent access to the underlying board.

## Interfaces Within RepRapFirmware

Almost every module depends on `Platform`. The most important links are:

- [../Config/README.md](../Config/README.md) for board capabilities and pins;
- [../Hardware/README.md](../Hardware/README.md) for low-level startup and device-family support;
- [../Movement/README.md](../Movement/README.md), [../Heating/README.md](../Heating/README.md), [../Networking/README.md](../Networking/README.md), [../Storage/README.md](../Storage/README.md), and [../SBC/README.md](../SBC/README.md) for the recurring services they need.

## DSF And Duet3Expansion Interfaces

- **DSF**: the SBC handshake and related timing or hardware support depend on platform services, but DSF does not interact with this module directly as a protocol peer.
- **Duet3Expansion**: analogous role only. The expansion firmware has its own platform layer rather than calling into RRF's.

## Standalone Vs SBC

This module is common to both modes. Some of the services it hosts become more or less important depending on who owns the front end, but the platform layer itself remains central either way.

## Related Docs

- [../../docs/devel/PLATFORM_AND_TASKS.md](../../docs/devel/PLATFORM_AND_TASKS.md)
- [../../docs/devel/ARCHITECTURE.md](../../docs/devel/ARCHITECTURE.md)
- [../Hardware/README.md](../Hardware/README.md)
- [../Config/README.md](../Config/README.md)
