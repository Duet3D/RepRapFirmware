# Libraries

`src/Libraries/` is the small collection of support libraries that are kept directly inside the RRF source tree rather than being consumed as top-level git submodules. They are not machine-control modules in their own right, but they provide the building blocks used by higher-level code such as storage and hashing.

## Key Areas

| Area | Purpose |
|---|---|
| [Fatfs/](Fatfs) | FAT filesystem support used by the storage layer. |
| [sd_mmc/](sd_mmc) | SD/MMC access helpers. |
| [sha1/](sha1) | SHA-1 support used where hashing is required. |

## How It Fits Into The Firmware

These sources sit below the real firmware modules. They provide reused implementation pieces but do not define the machine-level architecture or external control surface themselves.

The clearest example is storage: [../Storage/README.md](../Storage/README.md) owns the file and card behavior that the rest of the firmware sees, while `src/Libraries/` supplies the underlying filesystem and card-access code.

## Interfaces Within RepRapFirmware

- [../Storage/README.md](../Storage/README.md) is the primary consumer.
- lower-level platform and utility paths may also depend on the helpers here.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct interface.
- **Duet3Expansion**: no direct interface in this repo; expansion-board firmware has its own support stack.

## Standalone Vs SBC

This layer is mode-dependent only through its consumers. The library code itself has no standalone-vs-SBC policy.

## Related Docs

- [../Storage/README.md](../Storage/README.md)
- [../../docs/devel/BUILD_VARIANTS.md](../../docs/devel/BUILD_VARIANTS.md)
