# Axis following — M604

Makes one axis track another **as part of the same coordinated move**, rather than reacting to it.

The motivating case is a Z-independent dust shoe on a U axis: the shoe is carried on the Z carriage,
so U has to move the opposite way as Z plunges to keep the bristles on the material.

## Command

```
M604 A"<follower>" B"<leader>" [S<scale>] [O<offset>] [E<0|1>]
```

| Parameter | Meaning |
|---|---|
| `A` | Follower axis letter, quoted. The axis that is driven. |
| `B` | Leader axis letter, quoted. The axis that is tracked. |
| `S` | Scale, default **-1**. A follower carried on the leader must move the opposite way to stay put. |
| `O` | Offset. Normally omit it — see below. |
| `E` | 1 engages, 0 disengages. |
| *(none)* | Report state. |

The rule is `follower = S × leader + O`, in **machine coordinates**, clamped to the follower's `M208`
limits.

```gcode
M604 A"U" B"Z" E1     ; U now tracks Z
M604                  ; U follows Z as -1.000 * Z + 70.000, engaged
M604 E0               ; U becomes an ordinary axis again
```

## Engaging captures the relationship

**Omit `O`.** When you engage, the offset is computed from where the two axes are *right now*, so
engaging means "hold the current separation from here on". That is what the G-code implementation
used `global.dustShoePrevZ` for. Give `O` explicitly only if you want an absolute relationship.

So the sequence is: move the follower where you want it, then engage.

```gcode
G1 U30 F8000          ; put the shoe where the bristles touch
M400
M604 A"U" B"Z" E1     ; hold that relationship
```

## What you get for free

* **Zero latency.** The follower is derived where a move's target is computed, so it is part of the
  same move as the leader. Measured skew between the Z and U step trains is 0.0000 ms at the start,
  middle and end of a move — the planner never sees them as separate axes. This holds for straight
  moves, arc segments (including helical) and `M700` velocity jogging.
* **Axis limits.** The follower is clamped to its own `M208` range, which *is* the useful behaviour:
  the shoe tracks the tool down until it reaches its lower limit, then rests there while Z carries on
  into the work.
* **Acceleration and jerk limits.** The follower is a real axis in a real move, so its own `M201`,
  `M203` and `M566` apply, and lookahead plans it like anything else.

## Homing

Engaging is refused unless the follower axis is homed, matching the check the G-code daemon made.
Disengaging is always allowed.

## Replacing the G-code implementation

The `daemon.g` tracking loop becomes unnecessary:

```gcode
; daemon.g - delete the dust shoe section entirely
var deltaZ = ...
if {abs(var.deltaZ) > 0.1}
    G53 G1 U{var.targetU} F8000        ; all of this goes away
```

`dustShoeEngage.g` keeps its positioning move and gains one line:

```gcode
G1 U{var.targetU} F8000
M400
M604 A"U" B"Z" E1                      ; instead of set global.dustShoeEngaged = true
```

`dustShoeRetract.g` likewise:

```gcode
M604 E0                                ; instead of set global.dustShoeEngaged = false
G53 G1 U{move.axes[3].max} F8000
```

Keep the engage/retract macros: the engaged height is a machine property that belongs in
configuration, not in firmware.

## Tool changes

Because the rule is applied in **machine** coordinates, after tool offsets, tool length is handled
automatically. A longer tool puts the carriage higher for the same work Z, the leader's machine
coordinate rises, and the follower moves down by the same amount — which is exactly what the shoe
needs.

That makes the `U{-var.newOffset}` half of

```gcode
G10 L1 Z{var.newOffset} U{-var.newOffset}
```

redundant while following is engaged. It is harmless — the follower's coordinate is derived, so a U
tool offset is simply ignored — but it is misleading to leave in, because it looks like it is doing
something.

## Limitations

* Movement system 0 only.
* One follower relationship at a time.
* The follower must not be the leader.
* `M604` is a provisional command number. It was free in this firmware; check it against anything else
  you rely on before adopting it widely.
