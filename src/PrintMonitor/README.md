# PrintMonitor

`src/PrintMonitor/` tracks print progress, metadata, and ETA-related state. It is the firmware-side job-observation layer that turns file-level and print-state information into the user-visible job-progress fields exposed in the machine model.

## Key Files

| File | Purpose |
|---|---|
| [PrintMonitor.cpp](PrintMonitor.cpp) / [PrintMonitor.h](PrintMonitor.h) | Print-monitoring logic and derived state. |

## How It Works

The print monitor watches the active job context and extracts or consumes the metadata needed to describe progress meaningfully. It tracks concepts such as layer-related progress and time estimates, then publishes the resulting state outward through the object model.

One of the important mode differences shows up here: the overall print-monitoring feature exists in both modes, but the source of file and metadata information changes when DSF owns the file system.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) and [../Storage/README.md](../Storage/README.md) supply the active file and execution context.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes the resulting job state.
- [../SBC/README.md](../SBC/README.md) matters in SBC mode because the active print data is mediated by DSF.

## DSF And Duet3Expansion Interfaces

- **DSF**: DCS owns more of the file and job-management surface in SBC mode, which makes this module a clear consumer of DSF-mediated state.
- **Duet3Expansion**: no direct interface.

## Standalone Vs SBC

This module is common to both modes but strongly mode-dependent in where it gets its backing information from.

## Related Docs

- [../../docs/devel/STANDALONE_VS_SBC.md](../../docs/devel/STANDALONE_VS_SBC.md)
- [../Storage/README.md](../Storage/README.md)
- [../ObjectModel/README.md](../ObjectModel/README.md)
