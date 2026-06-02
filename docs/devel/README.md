# RepRapFirmware Developer Documentation

This directory contains developer documentation for **RepRapFirmware (RRF)** — the firmware that runs on Duet 3D printer control boards. It is intended as a working reference for anyone modifying, extending, or porting the firmware.

For build/setup instructions, see [../DEVELOPER.md](../DEVELOPER.md).

## How to read these docs

Start with [ARCHITECTURE.md](ARCHITECTURE.md) — the system-level diagram and component summary it contains is the map for every other document in this folder. From there, go deep on whichever subsystem you are working on.

| Document | What it covers |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Top-level firmware architecture, the `RepRap` container, the `Spin()` cooperative loop, FreeRTOS tasks, build-time variants. |
| [GCODE_PROCESSING.md](GCODE_PROCESSING.md) | G/M/T-code lifecycle: input channels, the `GCodeBuffer`, parser, state machine, command dispatch tables (`GCodes2.cpp` … `GCodes7.cpp`). |
| [MOTION_PIPELINE.md](MOTION_PIPELINE.md) | Move planning: `RawMove` → `DDA` → `MoveSegment` → `DriveMovement` → step ISR. Look-ahead, jerk, input shaping, `StepTimer`. |
| [KINEMATICS.md](KINEMATICS.md) | Kinematics architecture and conversion contracts, including generalized `K13` (`robot5axis`) mixed rotary/linear chain implementation and M669 configuration. |
| [HEATING.md](HEATING.md) | Heater / sensor / fan / tool architecture. PID, autotune, virtual heaters, remote sensors over CAN. |
| [OBJECT_MODEL.md](OBJECT_MODEL.md) | The reflected machine state — definition macros, JSON serialisation, sequence numbers, M409 query, replication to DSF. |
| [CAN_BUS.md](CAN_BUS.md) | CAN-FD bus to expansion / tool boards: addressing, time sync, motion fragments, generic command messages, `ExpansionManager`. |
| [SBC_INTERFACE.md](SBC_INTERFACE.md) | SPI link to a Linux SBC running DSF: framing, packet types, transfer state machine, file proxy. |
| [NETWORKING.md](NETWORKING.md) | Network stacks (LWIP / W5500 / WiFi), responders (HTTP, FTP, Telnet, MQTT), the `rr_*` legacy API. |
| [PLATFORM_AND_TASKS.md](PLATFORM_AND_TASKS.md) | The `Platform` HAL, task priorities, ADC pipeline, watchdogs, software reset, diagnostics. |
| [BUILD_VARIANTS.md](BUILD_VARIANTS.md) | The `make` matrix, conditional compilation flags (`SUPPORT_CAN_EXPANSION`, `HAS_SBC_INTERFACE`, …), board configuration. |

## Companion repositories

RepRapFirmware does not run alone. Two sibling repositories complete the system:

- **[Duet3Expansion](https://github.com/Duet3D/Duet3Expansion)** — slim firmware for CAN-attached tool / expansion boards. RRF is the CAN master.
- **[DuetSoftwareFramework](https://github.com/Duet3D/DuetSoftwareFramework)** (DSF) — .NET services that run on a Linux SBC paired with the Duet over SPI. RRF is the SPI slave.

For the cross-repo picture (how G-codes flow from a browser through DSF, RRF and out to a stepper driver on an expansion board), see the integration documentation under `DuetSoftwareFramework/docs/architecture/`.
