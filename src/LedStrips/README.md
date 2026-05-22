# LedStrips

`src/LedStrips/` contains support for addressable LED strips and related lighting effects. It packages the per-strip behavior and the logic needed to integrate lighting into the machine-control and user-feedback model.

## What This Module Owns

- LED-strip object lifetime and state;
- strip-level color and animation behavior;
- the bridge between user configuration and local or remote lighting hardware.

## How It Works

The firmware configures LED resources, then updates strip state as requested by user commands or firmware events. The module itself handles the strip-facing behavior while relying on lower layers to deal with whether the physical output is local or remote.

This keeps machine-facing lighting logic separate from the general-purpose GPIO and timer details that sit below it.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) provides the user-facing configuration and control path.
- [../Platform/README.md](../Platform/README.md) provides local hardware access.
- [../CAN/README.md](../CAN/README.md) carries remote strip updates when the hardware lives on an expansion board.
- [../ObjectModel/README.md](../ObjectModel/README.md) can surface lighting state where relevant.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct ownership; DSF just drives the usual commands and reads the resulting state.
- **Duet3Expansion**: remote LED resources are hosted and executed on expansion boards through the CAN path.

## Standalone Vs SBC

This module is common to both modes.

## Related Docs

- [../CAN/README.md](../CAN/README.md)
- [../Platform/README.md](../Platform/README.md)
- [../GCodes/README.md](../GCodes/README.md)
