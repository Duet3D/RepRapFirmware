# Standalone Vs SBC Mode In RepRapFirmware

This document describes what functionality RepRapFirmware owns directly in **standalone mode**, what shifts to [DuetSoftwareFramework](https://github.com/Duet3D/DuetSoftwareFramework/tree/v3.7-andy) in **SBC mode**, and which firmware modules stay central in both deployments.

It complements the system-level view in [ARCHITECTURE.md](ARCHITECTURE.md) and the cross-repo view in [DuetSoftwareFramework/docs/architecture/DEPLOYMENT_MODES.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/docs/architecture/DEPLOYMENT_MODES.md).

## 1. The mode boundary

RRF is always the real-time controller. The mode split changes who owns the **front end** and the **filesystem/network surface**, not who owns motion, heaters, or the CAN master role.

| Concern | Standalone owner | SBC owner |
|---|---|---|
| HTTP / browser API | RRF [Networking](../../src/Networking/README.md) | DSF `DuetWebServer` |
| File storage surface | RRF [Storage](../../src/Storage/README.md) on local SD | DSF virtual SD via RRF [SBC](../../src/SBC/README.md) |
| G-code front end | RRF local channels | DSF feeds RRF through the `SBC` channel |
| Real-time machine control | RRF | RRF |
| CAN expansion master | RRF [CAN](../../src/CAN/README.md) | RRF [CAN](../../src/CAN/README.md) |

## 2. Standalone-only functionality in RRF

These functions are owned directly by RRF only when there is no active DSF front end.

| Functionality | RRF modules | Why it is standalone-only |
|---|---|---|
| HTTP server and `rr_*` API | [Networking](../../src/Networking/README.md), especially `HttpResponder.cpp` | In SBC mode, DWS becomes the network front end and mirrors compatibility behavior. |
| FTP and Telnet services | [Networking](../../src/Networking/README.md) | DSF replaces the machine-facing network services with Linux-side equivalents or browser-driven flows. |
| On-firmware MQTT front end | [Networking](../../src/Networking/README.md) | In SBC deployments, DSF can own MQTT-side integration instead. |
| Local SD-card ownership | [Storage](../../src/Storage/README.md), [Libraries](../../src/Libraries/README.md) | In SBC mode, RRF no longer owns the user-visible filesystem. |
| Direct file-backed uploads, downloads, and directory listings | [Networking](../../src/Networking/README.md), [Storage](../../src/Storage/README.md) | These become DSF web or file operations. |
| Direct LCD as primary local UI | [Display](../../src/Display/README.md) | SBC systems normally use browser-based control through DWS/DWC instead. |

## 3. SBC-only functionality in RRF

These functions only make sense when DSF is attached and the SBC transport is active.

| Functionality | RRF modules | DSF peer |
|---|---|---|
| SPI or USB control-plane transport to the SBC | [SBC](../../src/SBC/README.md) | `DuetControlServer` link/protocol code |
| Dedicated `SBC` G-code channel fed by DSF | [GCodes](../../src/GCodes/README.md), [SBC](../../src/SBC/README.md) | DCS pipeline and channel management |
| Virtual-SD file proxying | [Storage](../../src/Storage/README.md), [SBC](../../src/SBC/README.md) | DCS file/path resolver and virtual SD |
| DSF-driven macro and file execution path | [GCodes](../../src/GCodes/README.md), [Storage](../../src/Storage/README.md), [SBC](../../src/SBC/README.md) | DCS macro/file orchestration |
| DSF-side object-model replication requests and replies | [ObjectModel](../../src/ObjectModel/README.md), [SBC](../../src/SBC/README.md) | DCS object-model sync and DWS distribution |
| SBC-coordinated firmware update path | [SBC](../../src/SBC/README.md), [Comms](../../src/Comms/README.md) | `DuetControlServer --update` |

## 4. Shared firmware functionality, with different back ends

The modules below stay central in both deployments. What changes is where their commands come from or how supporting resources are provided.

| Functionality | RRF modules | Standalone path | SBC path |
|---|---|---|---|
| G/M/T-code parsing and execution | [GCodes](../../src/GCodes/README.md) | HTTP/Telnet/USB/AUX/File channels enter RRF directly | DSF-owned front ends feed the `SBC` channel |
| Motion planning and step execution | [Movement](../../src/Movement/README.md) | Same motion engine | Same motion engine |
| Heater, sensor, and tool control | [Heating](../../src/Heating/README.md), [Tools](../../src/Tools/README.md), [Fans](../../src/Fans/README.md) | Configured and driven locally | Configured and driven through DSF-fed codes |
| Endstops, probes, and input state | [Endstops](../../src/Endstops/README.md), [InputMonitors](../../src/InputMonitors/README.md) | Same runtime logic | Same runtime logic |
| Print-progress and ETA logic | [PrintMonitor](../../src/PrintMonitor/README.md) | Reads local file context | Consumes DSF-mediated file/job context |
| Object model | [ObjectModel](../../src/ObjectModel/README.md) | Queried directly by standalone clients | Replicated into and extended by DSF |
| CAN expansion master role | [CAN](../../src/CAN/README.md) | Same CAN ownership | Same CAN ownership |

## 5. Shared functionality that still depends on mode-specific semantics

Some modules are present in both modes but their practical role shifts noticeably.

| Module | Mode-dependent detail |
|---|---|
| [Storage](../../src/Storage/README.md) | The module stays in place, but the backing store flips from local SD ownership to SBC proxying. |
| [PrintMonitor](../../src/PrintMonitor/README.md) | The progress model remains, but the file/metadata source differs. |
| [ObjectModel](../../src/ObjectModel/README.md) | The schema is common, but standalone clients query RRF directly while SBC deployments consume a DSF-merged view. |
| [Networking](../../src/Networking/README.md) | Present as a major subsystem only in standalone mode; mirrored by DSF semantics in SBC deployments. |

## 6. Typical workflow examples

### Send a G-code from a UI

- **Standalone**: browser or Telnet client talks to RRF [Networking](../../src/Networking/README.md) which feeds [GCodes](../../src/GCodes/README.md) directly.
- **SBC**: browser talks to DWS, DWS talks to DCS, DCS feeds [SBC](../../src/SBC/README.md), and RRF [GCodes](../../src/GCodes/README.md) receives the code on the `SBC` channel.

### Start a print from a file

- **Standalone**: [Storage](../../src/Storage/README.md) and [PrintMonitor](../../src/PrintMonitor/README.md) work from the local SD-backed file.
- **SBC**: DCS owns the visible file store and RRF's [Storage](../../src/Storage/README.md) becomes a client of the SBC file proxy.

### Read machine state

- **Standalone**: clients query [ObjectModel](../../src/ObjectModel/README.md) through RRF's own network front end.
- **SBC**: RRF exports the same core state through [SBC](../../src/SBC/README.md), DCS mirrors it into `DuetAPI`, and DWS serves it onward.

## 7. What does not change with the mode

- RRF remains the owner of motion timing.
- RRF remains the owner of heater control and most low-level machine safety.
- RRF remains the CAN master for Duet3Expansion boards.
- Duet3Expansion boards never become direct DSF peers; they always sit behind RRF.

## 8. Related docs

- [ARCHITECTURE.md](ARCHITECTURE.md)
- [SBC_INTERFACE.md](SBC_INTERFACE.md)
- [NETWORKING.md](NETWORKING.md)
- [OBJECT_MODEL.md](OBJECT_MODEL.md)
- [DuetSoftwareFramework/docs/architecture/DEPLOYMENT_MODES.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/docs/architecture/DEPLOYMENT_MODES.md)
- [DuetSoftwareFramework/docs/architecture/COMPONENT_INTERACTION_MATRIX.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/docs/architecture/COMPONENT_INTERACTION_MATRIX.md)
