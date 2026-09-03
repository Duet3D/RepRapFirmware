# Hardware

`src/Hardware/` contains the low-level processor-family support that sits below the higher-level `Platform` abstraction. It covers startup, device-family specialisation, low-level peripheral definitions, linker layouts, reset handling, non-volatile memory support, and exception/fault plumbing.

## Key Files And Areas

| Area | Purpose |
|---|---|
| [ExceptionHandlers.cpp](ExceptionHandlers.cpp) | Fault and exception handling support. |
| [I2C.cpp](I2C.cpp), [IoPorts.cpp](IoPorts.cpp), [NonVolatileMemory.cpp](NonVolatileMemory.cpp), [SoftwareReset.cpp](SoftwareReset.cpp) | Shared low-level hardware helpers. |
| [SAM4E/](SAM4E), [SAM4S/](SAM4S), [SAME5x/](SAME5x), [SAME70/](SAME70) | Processor-family startup code, device wiring, and linker scripts. |

## How It Works

This directory is where firmware startup and processor-family differences become concrete. The code here sets up the hardware context in which the rest of RRF can run, then exposes the low-level facilities that `Platform` and other modules depend on.

Keeping these details here means the higher-level firmware can mostly talk in terms of board capabilities and platform services instead of hard-coding processor-family logic all over the tree.

## Interfaces Within RepRapFirmware

- [../Platform/README.md](../Platform/README.md) is the main consumer.
- [../Config/README.md](../Config/README.md) selects which processor-family path is active.
- low-level services such as storage, I2C-attached devices, resets, and exception reporting all depend on the code here.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct runtime interface, though some DSF-facing features such as the SBC link depend on the hardware support enabled here.
- **Duet3Expansion**: analogous role only. Duet3Expansion has its own lower-level processor and startup code but not a shared runtime path.

## Standalone Vs SBC

This module is common to both modes. The hardware platform is the same regardless of who owns the front end.

## Related Docs

- [../../docs/devel/PLATFORM_AND_TASKS.md](../../docs/devel/PLATFORM_AND_TASKS.md)
- [../Platform/README.md](../Platform/README.md)
- [../Config/README.md](../Config/README.md)
