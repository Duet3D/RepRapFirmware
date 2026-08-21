# Dynamic IDEX bed-plane control

## Current implementation scope

The branch contains an isolated mathematical transform and safety validation, read-only access to configured `M671` leadscrew geometry, and enumeration of the individual mesh samples that stock firmware previously reduced immediately to one average. It deliberately does not modify runtime motion behavior.

`DynamicBedPlane::CalculateTelemetry()` now runs the complete two-nozzle-to-leadscrew calculation in one side-effect-free call. It reports the fitted plane, ordered leadscrew corrections, extrema, and correction spread. Rejected calculations leave the caller's last known-good telemetry unchanged.

`Move::CalculateDynamicBedPlaneTelemetry()` is the first firmware-facing bridge. Given a point that has already passed through `AxisTransform`, it:

- requires an active mesh and exactly two physical nozzle samples at the same Y;
- rejects either nozzle outside the measured grid instead of accepting `HeightMap`'s normal edge clamping;
- applies `zShift` and the configured taper to each physical nozzle sample;
- reads ordered `M671` geometry through the base kinematics interface; and
- returns the side-effect-free telemetry snapshot.

No planner path calls this method yet. It cannot change a DDA, a CAN message, `specialMoveCoords`, or a physical motor endpoint.

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
   - enumerate mapped-axis mesh samples through `ForEachHeightCorrection()` before reducing them to one scalar;
   - retain both transformed carriage coordinates and the legacy offset-adjusted lookup coordinates so duplicate-tool offsets are not irreversibly lost;
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

The pure telemetry calculation is implemented. The next planner-integrated version should call it for eligible duplicate/mirror moves and retain the results for diagnostics while continuing to send the stock equal-Z movement. This makes trajectory comparison possible without changing hardware motion.

## Non-negotiable guards

- disabled by default;
- exact supported tool and mapped axes required;
- finite mesh samples inside the measured map;
- minimum nozzle separation;
- maximum X slope;
- maximum absolute motor correction and motor-to-motor spread;
- later: per-motor velocity, acceleration, jerk, and transform-discontinuity limits;
- disable/reject during homing, probing, leadscrew calibration, recovery, and unqualified tool changes.
