# Velocity jogging — M700

Normal motion commands say *where* to go. `M700` says *how fast to go, and in which direction*, per axis,
so an analogue input such as a joystick can drive the machine directly.

## Command

```
M700 X<speed> Y<speed> Z<speed> ... [S0] [P<ms>] [R<ms>] [D<n>]
```

| Parameter | Meaning |
|---|---|
| axis letters | Signed speed for that axis in **mm/sec** (degrees/sec for rotational axes). `G20` does *not* rescale these. |
| `S0` | Stop jogging now. |
| `P` | Chunk time in ms, 10..200, default 50. See *Latency* below. |
| `R` | Watchdog timeout in ms, default 250. |
| `D` | How many moves to keep queued, 2..8, default 3. |
| none | Report status. |

**The axis letters present define the whole velocity vector.** Any axis you do not mention is set to zero.
A truncated or partially-parsed command therefore cannot leave an axis running.

Send a fresh `M700` whenever the stick moves, and at least every `R` milliseconds while it is off centre.

```gcode
M700 X25 Y-12     ; X at +25 mm/s, Y at -12 mm/s, everything else stopped
M700 X25          ; Y now stops, X carries on
M700 S0           ; stop
```

## How it works

Jogging synthesises a stream of short constant-velocity moves and feeds them to movement system 0 through
exactly the same path a `G1` takes: `MovementState::raw` → `GCodes::ReadMove` → `DDARing::AddStandardMove`.

That reuse is the whole point of the design:

* **Lookahead blends the chunks**, so a steady stick gives steady motion, and changing the stick direction
  produces a normal cornered junction rather than a stop-start.
* **The last move in the ring is always planned to end at zero speed.** If the command stream dies — cable
  pulled, host crashed, task starved — the machine decelerates to a stop under its normal acceleration
  limits instead of stopping dead and losing steps.
* Per-axis speed and acceleration limits, kinematics, bed compensation and tool offsets all apply
  unchanged.

`JogController::Spin()` is called from `GCodes::Spin()` and tops the queue up whenever it holds fewer than
`D` moves.

## Latency

Response to a stick movement is roughly `D × P` plus the ~50 ms that `Move` prepares ahead
(`MoveTiming::UsualMinimumPreparedTime`) — about 150–200 ms at the defaults. Reducing `D` or `P` sharpens
the response but lowers the speed ceiling (see below) and leaves less slack for a busy main loop.

## Safety

* **Speed clamp.** Each axis is clamped to its `M203` maximum *and* to `2·a·P`. That second limit is not
  arbitrary: `DDA::InitStandardMove` caps the entry speed of every move at `sqrt(2·a·d)` for that move
  alone, so that any move can be the last one in the ring and still stop at its end. With `d = v·P` that
  solves to `v ≤ 2·a·P`. Commanding more would not go faster, it would silently not be obeyed, so `M700`
  clamps to it. At the defaults with a = 1000 mm/s² the ceiling is 100 mm/s (6000 mm/min); **to jog faster,
  raise `P`** — and accept the extra latency.
* **Watchdog.** If no `M700` arrives within `R` ms, the velocity is zeroed and the machine decelerates.
* **Axis limits.** Every chunk is passed through `Kinematics::LimitPosition` with `initialCoords` set, so
  the whole line is checked, not just its end point. An axis that reaches its limit simply stops; the
  others keep their commanded speed, because the chunk still takes one chunk time to execute.
* **Homing.** Starting a jog requires the same axes to be homed that a `G1` would, subject to `M564`.
* **Interlocks.** Jogging refuses to start while a print is running, stops if one starts, and is cancelled
  by anything that waits for standstill on movement system 0 (`G28`, `G30`, most `M` codes that move) and
  by `M112`/`M999`.

For an immediate halt, use `M112` — `M700 S0` decelerates.

## Limitations

* Movement system 0 only.
* Mentioning linear and rotational axes in the same command works, but RepRapFirmware treats the two
  groups' feedrates separately, so the resulting speeds are only approximately as commanded.
* RepRapFirmware has no USB host stack: the joystick has to be read by an SBC, Pi or other host that then
  sends `M700` over USB or the network, ideally on its own input channel.

## Where the code is

| | |
|---|---|
| `src/Movement/JogController.{h,cpp}` | all of the logic |
| `src/GCodes/GCodes.cpp` | `Spin()` calls `jogController.Spin()`; `Reset()` and `LockMovementSystemAndWaitForStandstill()` stop it |
| `src/GCodes/GCodes2.cpp` | `M700` dispatch |
