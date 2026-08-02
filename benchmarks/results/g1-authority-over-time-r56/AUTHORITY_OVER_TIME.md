# Bonesaw continuous authority over time · r56

## Result

The same 2,317 immutable G1 oracle states are reinterpreted as independent continuous capability signals. No policy, state integration, simulator, or physics is introduced. Hard feasibility remains instantaneous; soft tracking, compute pressure, and legal Preference/Style clipping additionally receive episode-duration, dwell, and 0.25/1/5 second leaky-exposure evidence.

> There is deliberately no aggregate health score. “Dominant signal” only orders rows for display at one tick; it cannot hide a hard failure or convert unrelated units.

## Signal summary

| signal | max pressure | warning ticks / longest | critical ticks / longest | 1 s exposure p99 | meaning |
|---|---|---|---|---|---|
| hard_constraints | 0.171 | 40 / 20 ms | 0 / 0 ms | 0.032 | maximum rigid-body/contact/original-row residual divided by 1e-8 |
| finite_support | 1.000 | 1425 / 1325 ms | 424 / 30 ms | 0.793 | consumption of the first 5 mm reserve beyond the mandatory 5 mm erosion |
| invariant_root | 0.002 | 0 / 0 ms | 0 / 0 ms | 0.000 | maximum root attitude/height task RMS divided by its 1-unit contract |
| viability_transfer | 0.811 | 1 / 5 ms | 0 / 0 ms | 0.178 | maximum horizontal-root/CoM task RMS divided by its 1 m/s² contract |
| viability_effectors | 0.014 | 0 / 0 ms | 0 / 0 ms | 0.003 | maximum linear/angular effector residual divided by its task contract |
| joint_position | 0.000 | 0 / 0 ms | 0 / 0 ms | 0.000 | consumption of a 15 degree evaluation headroom band; zero margin is critical |
| actuator_effort | 0.531 | 0 / 0 ms | 0 / 0 ms | 0.354 | maximum authored actuator-limit utilization |
| solver_5ms | 1.566 | 297 / 155 ms | 76 / 40 ms | 0.773 | tick time divided by the nominal 5 ms servo period; observational, not r54 admission |
| solver_20ms | 0.392 | 0 / 0 ms | 0 / 0 ms | 0.193 | tick time divided by the declared r54 admission deadline |
| preference_clipping | 1.000 | 1394 / 990 ms | 0 / 0 ms | 0.844 | binary legal clipping of the joint-posture Preference task |
| style_clipping | 1.000 | 1267 / 575 ms | 0 / 0 ms | 0.766 | binary legal clipping of force or torque Style tasks |

## Persistence curves

Each row counts warning episodes that survive at least the stated continuous dwell. This lets an adapter distinguish a one-tick soft excursion from sustained loss of authority. Hard constraint criticality still has zero permitted dwell.

| signal | ≥5 ms | ≥10 ms | ≥20 ms | ≥100 ms | ≥500 ms |
|---|---|---|---|---|---|
| hard_constraints | 37 | 1 | 1 | 0 | 0 |
| finite_support | 130 | 89 | 57 | 16 | 3 |
| invariant_root | 0 | 0 | 0 | 0 | 0 |
| viability_transfer | 1 | 0 | 0 | 0 | 0 |
| viability_effectors | 0 | 0 | 0 | 0 | 0 |
| joint_position | 0 | 0 | 0 | 0 | 0 |
| actuator_effort | 0 | 0 | 0 | 0 | 0 |
| solver_5ms | 159 | 50 | 12 | 1 | 0 |
| solver_20ms | 0 | 0 | 0 | 0 | 0 |
| preference_clipping | 45 | 40 | 31 | 16 | 2 |
| style_clipping | 107 | 59 | 46 | 21 | 1 |

## Interpretation

- Hard constraints never reach their critical boundary; no waiting policy is legal or needed there.
- The finite-support row stays at its critical display boundary because the solver deliberately consumes the complete optional reserve while preserving the mandatory 5 mm erosion. This is low geometric reserve, not a CoP violation.
- Root Invariant and effector tracking remain far below their contracts. Viability transfer approaches but does not cross its contract, which makes it the meaningful continuous tracking pressure.
- The nominal 5 ms servo period has 76 overruns, while the declared 20 ms admission deadline has none. Any 5 ms deployment needs further CPU-tail work or a slower supervisory cadence; the evidence must not be relabeled.
- Preference and Style clipping persist for long runs. They are legal nullspace relaxation and must remain visible even though physical admission is green.

## Artifacts

`authority-over-time.csv` retains one row per tick and per signal. `authority-over-time-metrics.json` retains thresholds, distributions, warning/critical episodes, dwell curves, and leaky exposures. Source NPZ and JSON hashes are embedded.
