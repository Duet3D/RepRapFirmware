# RepRapFirmware Source Modules

RepRapFirmware is a single firmware image rather than a multi-project solution, so the most useful documentation boundary inside `src/` is the top-level module directory. Each first-party source module now has its own `README.md` describing what it owns, how it works, and how it interacts with DuetSoftwareFramework and Duet3Expansion.

## Core control-path modules

| Module | Focus |
|---|---|
| [GCodes](GCodes/README.md) | G/M/T-code parsing, channel scheduling, dispatch, conditional G-code, replies. |
| [Movement](Movement/README.md) | Motion planning, DDA queue, kinematics, step timing, shaping, CAN motion splitting. |
| [Heating](Heating/README.md) | Heaters, sensors, PID, autotune, tool temperature state. |
| [ObjectModel](ObjectModel/README.md) | Reflected machine state, M409, expression backing, DSF schema contract. |
| [Platform](Platform/README.md) | Main hardware abstraction, cooperative housekeeping, task ownership, watchdog integration. |
| [Storage](Storage/README.md) | SD card access, file lifecycle, SBC virtual-SD proxying. |
| [SBC](SBC/README.md) | SPI/USB link to DSF. |
| [CAN](CAN/README.md) | Main-board CAN-FD master for expansion/tool boards. |

## Machine I/O and device modules

| Module | Focus |
|---|---|
| [Endstops](Endstops/README.md) | Endstops, probes, homing-state inputs. |
| [InputMonitors](InputMonitors/README.md) | Local and CAN-remote input monitoring abstraction. |
| [Fans](Fans/README.md) | PWM, thermostatic, tacho, local and remote fan control. |
| [FilamentMonitors](FilamentMonitors/README.md) | Runout and slip detection tied to extrusion state. |
| [LedStrips](LedStrips/README.md) | Addressable LED control and animation. |
| [GPIO](GPIO/README.md) | General-purpose inputs, outputs, PWM, servo-style ports. |
| [Accelerometers](Accelerometers/README.md) | Onboard and remote accelerometer support for resonance and shaping workflows. |
| [Tools](Tools/README.md) | Tool, spindle, and filament ownership above the lower-level modules. |

## Front-end and board-variant modules

| Module | Focus |
|---|---|
| [Networking](Networking/README.md) | Standalone HTTP, FTP, Telnet, MQTT, and interface drivers. |
| [Comms](Comms/README.md) | USB CDC, AUX/PanelDue-adjacent comms, update helpers. |
| [Display](Display/README.md) | Direct LCD menu systems on supported boards. |
| [PrintMonitor](PrintMonitor/README.md) | Print metadata, progress, and ETA estimation. |
| [Config](Config/README.md) | Build-time board configuration and feature flags. |
| [Hardware](Hardware/README.md) | Processor-family startup, low-level devices, fault/reset support, linker layout. |
| [DuetNG](DuetNG/README.md) | Legacy Duet 2 expansion support. |
| [ClosedLoop](ClosedLoop/README.md) | Closed-loop support hooks on boards that compile it in. |
| [Libraries](Libraries/README.md) | Local support libraries vendored into the firmware tree. |

## Excluded vendor trees

These directories are intentionally not covered by the per-module README pass because they are imported or tool-specific support code rather than first-party RRF modules:

- `bossa`
- `libc`
- `libcpp`

For the system-level map, start with [../docs/devel/ARCHITECTURE.md](../docs/devel/ARCHITECTURE.md). For the standalone-vs-SBC split, see [../docs/devel/STANDALONE_VS_SBC.md](../docs/devel/STANDALONE_VS_SBC.md).
