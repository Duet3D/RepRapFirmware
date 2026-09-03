# Display

`src/Display/` implements the direct LCD user interface that is available on supported standalone-oriented builds. It turns local rotary-encoder and LCD hardware into a small firmware-hosted control surface for status, menus, and manual actions.

## What This Module Owns

- LCD and menu rendering logic for supported board/display combinations;
- rotary-encoder input handling;
- menu actions that feed back into the normal G-code and machine-control paths.

## How It Works

The display module is a UI wrapper around existing firmware capabilities, not an alternative control plane. It polls local input hardware, redraws the menu state, and dispatches actions through the same lower-level machine-control logic that other front ends use.

That makes it a good example of RRF's architectural layering: the display itself owns interaction and presentation, while motion, heating, tool changes, or file actions are still carried out by the core modules.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) receives many menu-triggered operations.
- [../ObjectModel/README.md](../ObjectModel/README.md) supplies live state for the UI.
- [../Platform/README.md](../Platform/README.md) provides the underlying pin and timing access.

## DSF And Duet3Expansion Interfaces

- **DSF**: none directly. DSF replaces the main UI surface in SBC systems, so this module is not part of the DSF-owned control path.
- **Duet3Expansion**: none.

## Standalone Vs SBC

This module is effectively standalone-oriented. In SBC mode the primary UI is browser-based and served by DSF, so the direct LCD path is not the main control surface.

## Related Docs

- [../../docs/devel/ARCHITECTURE.md](../../docs/devel/ARCHITECTURE.md)
- [../GCodes/README.md](../GCodes/README.md)
- [../ObjectModel/README.md](../ObjectModel/README.md)
