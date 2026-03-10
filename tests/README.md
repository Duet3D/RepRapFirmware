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