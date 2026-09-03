# ClosedLoop

`src/ClosedLoop/` contains the hooks and shared firmware support for the closed-loop driver variants used on selected Duet 3 hardware. In the RepRapFirmware tree this is primarily the main-board side of configuration, feature enablement, and coordination with boards that execute closed-loop motion locally.

## What This Module Owns

- the closed-loop feature integration points that are compiled in on supported builds;
- the high-level firmware-side state and configuration touch points for closed-loop hardware;
- the bridge between G-code configuration and the lower-level motion or expansion-board implementation.

## How It Works

RepRapFirmware remains the machine-level coordinator. It accepts closed-loop configuration through normal G-code processing, stores the resulting state, and routes motion or tuning requests toward the appropriate driver location. On Duet 3 systems that usually means the detailed current-loop execution happens on an expansion board, while RRF owns the higher-level orchestration.

This directory therefore sits at the boundary between motion configuration and CAN-distributed execution rather than replacing the open-loop motion pipeline wholesale inside the main-board firmware.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) supplies the control/configuration path.
- [../Movement/README.md](../Movement/README.md) remains the owner of the motion-planning side.
- [../CAN/README.md](../CAN/README.md) carries the remote execution path on systems where the closed-loop driver is not local to the main board.

## DSF And Duet3Expansion Interfaces

- **DSF**: no direct transport. DSF participates by feeding the relevant G-codes and observing the resulting object-model state.
- **Duet3Expansion**: the heavy closed-loop execution path lives on the expansion side for supported boards, making this module a direct architectural peer of Duet3Expansion's `src/ClosedLoop/`.

## Standalone Vs SBC

Closed-loop support is mode-independent from RRF's point of view. The path into the feature differs between standalone and SBC, but the firmware responsibilities are the same.

## Related Docs

- [../../docs/devel/CAN_BUS.md](../../docs/devel/CAN_BUS.md)
- [Duet3Expansion docs/devel/CLOSED_LOOP.md](https://github.com/Duet3D/Duet3Expansion/blob/3.7-docker/docs/devel/CLOSED_LOOP.md)
- [../Movement/README.md](../Movement/README.md)
