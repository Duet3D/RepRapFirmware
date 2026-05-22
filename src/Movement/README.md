# Movement

`src/Movement/` is the motion engine of RepRapFirmware. It turns parsed moves into timed step output, handling kinematics, look-ahead, shaping, pressure advance, bed compensation, and the split between local drivers and CAN-attached remote drivers.

## Key Files And Areas

| Area | Purpose |
|---|---|
| [Move.cpp](Move.cpp), [Move2.cpp](Move2.cpp), [Move3.cpp](Move3.cpp) | Top-level motion control and queue management. |
| [DDA.cpp](DDA.cpp), [DDARing.cpp](DDARing.cpp) | Digital differential analyzer objects and queue structures. |
| [DriveMovement.cpp](DriveMovement.cpp), [MoveSegment.cpp](MoveSegment.cpp) | Per-drive and per-segment execution details. |
| [StepTimer.cpp](StepTimer.cpp) | Step-timing and ISR-facing timing support. |
| [Kinematics/](Kinematics) | Machine-type-specific kinematics implementations. |
| [BedProbing/](BedProbing), [HeightControl/](HeightControl) | Probe-driven movement support and related motion behaviors. |
| [StepperDrivers/](StepperDrivers) | Smart-driver integration. |

## How It Works

The motion subsystem accepts high-level move intent from `GCodes`, transforms it into motor-space behavior, and prepares the time-coherent data structures that the step-execution path consumes. It owns the most timing-sensitive logic in the firmware and therefore embodies many of the major performance constraints in the system.

For Duet 3 systems it also owns the split between local and remote execution. The move planner produces a single coherent machine motion, then separates the local-driver portion from the pieces that must be sent over CAN to expansion boards.

## Interfaces Within RepRapFirmware

- [../GCodes/README.md](../GCodes/README.md) is the main entry path.
- [../Endstops/README.md](../Endstops/README.md) and [../InputMonitors/README.md](../InputMonitors/README.md) participate in homing and probing.
- [../CAN/README.md](../CAN/README.md) carries remote motion execution.
- [../Platform/README.md](../Platform/README.md) provides the low-level driver and timing resources.
- [../ObjectModel/README.md](../ObjectModel/README.md) exposes the resulting motion state.

## DSF And Duet3Expansion Interfaces

- **DSF**: DSF does not plan motion, but DCS feeds motion-producing codes into RRF and observes the resulting state and replies. In SBC mode, the motion subsystem is one of the most important consumers of the SBC channel.
- **Duet3Expansion**: this module is the main-board peer of Duet3Expansion's own motion-execution code. RRF plans the machine motion; expansion boards execute their assigned slices.

## Standalone Vs SBC

The motion subsystem is common to both modes. The entry path differs, not the motion math or timing model.

## Related Docs

- [../../docs/devel/MOTION_PIPELINE.md](../../docs/devel/MOTION_PIPELINE.md)
- [../CAN/README.md](../CAN/README.md)
- [../GCodes/README.md](../GCodes/README.md)
- [Duet3Expansion/docs/devel/MOTION.md](https://github.com/Duet3D/Duet3Expansion/blob/3.7-docker/docs/devel/MOTION.md)
