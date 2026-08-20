# Host-Native Tests

This directory contains Linux-hosted tests for code that can be validated without running on a Duet board.

The initial scope is intentionally narrow:

- pure math and algorithm code that does not require MCU peripherals
- selected firmware-side model code compiled against narrow test-only shims
- deterministic tests that run with the system C++ compiler
- no dependency on the ARM cross-toolchain

Run the tests from the RepRapFirmware root:

```sh
make test-host
```

Current contents:

- `support/` - minimal in-repo test runner
- `support/shims/` - narrow host-only shims for firmware types needed by selected model code
- `unit/` - host-native unit tests
- `make test-host` from the repository root builds and runs the suite with `g++`

Current coverage:

- RRFLibraries helpers: bitmap, deviation, integer square root, fast square root
- RepRapFirmware heater model: `FOPDT`

Planned next steps:

- add deterministic time and captured-output test support
- add more RepRapFirmware-side tests for kinematics and control logic
## Why the motion system is not testable here

`Move.cpp` needs 98 RepRapFirmware headers across 18 subsystems just to parse,
and its body calls `reprap.GetGCodes()` 91 times, plus `GetPlatform()`,
`GetExpansion()` and `GetPortControl()`. `Move` and `DDA` are not separable from
the `reprap` singleton, so reaching the planner from here would mean either a
large fake Platform/GCodes layer or refactoring `Move` onto interfaces.

Until that changes, host tests are limited to leaf code. Validating motion
behaviour - junction blending, speed ceilings, deceleration on loss of input -
needs a peripheral-level emulator such as Renode.

Measure it again before believing it:

```sh
make Duet3_MB6HC ...                       # produces the .d files
tr ' ' '\n' < Duet3_MB6HC/src/Movement/Move.d | grep -E '^src/.*\.h$' | sort -u | wc -l
```
