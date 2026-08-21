# Dynamic IDEX bed-plane control

## Scope of the foundation commit

The initial commit adds an isolated mathematical transform and safety validation. It deliberately does not modify runtime behavior. This separation allows the transform to be unit-tested before motion-state, segmentation, CAN, and recovery concerns are introduced.

## Machine model

For two active nozzles at the same Y coordinate, hold dynamic Y slope at zero and solve:

```text
p(x) = a + b*x
b = (rightCorrection - leftCorrection) / (rightX - leftX)
a = leftCorrection - b*leftX
```

Evaluate `p(x)` at each configured `M671` leadscrew X coordinate. The result is an ordered correction vector corresponding to the physical drivers declared by `M584 Z`.

## Planned integration points

1. `Move::ComputeHeightCorrection`
   - expose physical per-nozzle mesh samples instead of immediately reducing them to one scalar;
   - retain existing behavior unless the new mode is explicitly enabled.
2. `ZLeadscrewKinematics`
   - provide read-only access to ordered `M671` coordinates through `GetNumLeadscrews()` and `GetLeadscrewCoordinates()`;
   - validate that the number of coordinates matches the number of Z drivers.
3. `DDA`
   - represent per-Z-driver endpoints during ordinary queued moves;
   - include those motor deltas in speed, acceleration, and jerk limiting;
   - segment moves at mesh-cell boundaries or an equivalent error-bounded interval.
4. `CanMotion`
   - send distinct endpoints to each remote Z driver during normal moves;
   - preserve synchronized execution timestamps.
5. Position and recovery state
   - track logical Z separately from physical leadscrew offsets;
   - serialize all offsets for pause, resume, power-fail recovery, and tool changes.
6. Object model and configuration
   - report disabled/enabled/dry-run state, plane coefficients, motor targets, maxima, and rejection reason;
   - do not allocate a public G-code until the interface has been discussed upstream.

## First runtime milestone

The first planner-integrated version should be telemetry-only. It calculates the three motor endpoints and safety results for every move but continues sending the stock equal-Z movement. This makes trajectory comparison possible without changing hardware motion.

## Non-negotiable guards

- disabled by default;
- exact supported tool and mapped axes required;
- finite mesh samples inside the measured map;
- minimum nozzle separation;
- maximum X slope;
- maximum absolute motor correction and motor-to-motor spread;
- later: per-motor velocity, acceleration, jerk, and transform-discontinuity limits;
- disable/reject during homing, probing, leadscrew calibration, recovery, and unqualified tool changes.
