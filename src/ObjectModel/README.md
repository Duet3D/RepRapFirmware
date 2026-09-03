# ObjectModel

`src/ObjectModel/` is the reflected machine-state layer that ties together RepRapFirmware, DSF, DWC, and other clients. It turns live C++ state into a queryable schema used by `M409`, by conditional expressions, and by the DSF object-model replication path.

## Key Files

| File | Purpose |
|---|---|
| [ObjectModel.cpp](ObjectModel.cpp) / [ObjectModel.h](ObjectModel.h) | Core object-model support and reflection plumbing. |
| [Variable.cpp](Variable.cpp) / [Variable.h](Variable.h) | Variable support used by expressions and meta G-code. |
| [GlobalVariables.cpp](GlobalVariables.cpp) / [GlobalVariables.h](GlobalVariables.h) | Global variable storage. |
| [TypeCode.h](TypeCode.h) | Type metadata used by the reflection layer. |

## How It Works

The object model is built from descriptors that individual modules contribute. Those descriptors expose live machine state in a uniform way, which allows a single query and expression system to work across motion, heating, networking, tools, jobs, CAN expansion, and more.

This is one of the key contracts that spans repositories. RRF owns the firmware-side schema and serialization behavior, while DSF has to mirror the same concepts into `DuetAPI` and its object-model update logic.

## Interfaces Within RepRapFirmware

Every major module contributes state to the object model or depends on it for queries and expressions. The most important direct consumers are:

- [../GCodes/README.md](../GCodes/README.md) for expressions, variables, and `M409`;
- [../Networking/README.md](../Networking/README.md) for standalone API responses;
- [../SBC/README.md](../SBC/README.md) for DSF replication.

## DSF And Duet3Expansion Interfaces

- **DSF**: this is the deepest direct cross-repo contract between RRF and DSF. DCS mirrors the firmware object model into `DuetAPI`, merges in DSF-owned state, and serves it onward to clients.
- **Duet3Expansion**: expansion-board state reaches the object model through RRF's CAN path and is exposed as part of the main board's model rather than through a separate expansion-board object model.

## Standalone Vs SBC

The schema is common to both modes, but the consumers differ. Standalone clients query RRF directly; SBC deployments replicate and extend the model through DSF.

## Related Docs

- [../../docs/devel/OBJECT_MODEL.md](../../docs/devel/OBJECT_MODEL.md)
- [DuetSoftwareFramework/src/DuetAPI/README.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/src/DuetAPI/README.md)
- [DuetSoftwareFramework/docs/architecture/OBJECT_MODEL_END_TO_END.md](https://github.com/Duet3D/DuetSoftwareFramework/blob/v3.7-andy/docs/architecture/OBJECT_MODEL_END_TO_END.md)
