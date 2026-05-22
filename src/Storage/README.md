# Storage

`src/Storage/` owns file and storage behavior as seen by the rest of the firmware. It is responsible for SD-card-facing lifecycle and file operations in standalone mode, and it is equally responsible for making DSF-backed virtual storage look like normal firmware storage in SBC mode.

## Key Files

| File | Purpose |
|---|---|
| [MassStorage.cpp](MassStorage.cpp) / [MassStorage.h](MassStorage.h) | Overall storage/card lifecycle management. |
| [FileStore.cpp](FileStore.cpp) / [FileStore.h](FileStore.h) | File-facing abstraction used by the rest of the firmware. |

## How It Works

The storage module hides whether files are coming from a physical SD card or from DSF's virtual SD. Other modules ask for file-level behavior and this directory provides it through a stable abstraction.

That makes it one of the clearest examples of mode-sensitive back-end swapping in the firmware: the visible behavior is similar, but the implementation path is very different between standalone and SBC deployments.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) depends on file operations for macros, jobs, and configuration.
- [../PrintMonitor/README.md](../PrintMonitor/README.md) depends on file-backed job context.
- [../Platform/README.md](../Platform/README.md) participates in card lifecycle and recurring housekeeping.
- [../SBC/README.md](../SBC/README.md) supplies the remote file-operation path in SBC mode.

## DSF And Duet3Expansion Interfaces

- **DSF**: in SBC mode this module becomes a client of DSF's virtual-SD backing store, reached over the SBC link.
- **Duet3Expansion**: no direct interface.

## Standalone Vs SBC

This is one of the most mode-sensitive modules in the firmware. In standalone mode it owns real SD-backed storage; in SBC mode it proxies file operations to DSF while keeping the higher-level firmware behavior stable.

## Related Docs

- [../../docs/devel/SBC_INTERFACE.md](../../docs/devel/SBC_INTERFACE.md)
- [../../docs/devel/STANDALONE_VS_SBC.md](../../docs/devel/STANDALONE_VS_SBC.md)
- [../PrintMonitor/README.md](../PrintMonitor/README.md)
