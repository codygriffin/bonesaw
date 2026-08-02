# G1 feedback-authority experiment · r36

## Question

Revision r35 proved that measured phase time alone cannot select a safe task
authority transfer. Revision r36 adds the missing feedback observations and
asks whether continuous envelope authority driven by capture, attitude, and
landing health can preserve the transfer.

All cases use the pinned Unitree G1 model, CMU 37/01 motion, 600 × 5 ms
Rust-owned ticks, backward DCM preview, 1 cm support erosion, centroidal
angular-momentum damping, and unchanged touchdown/hard-feasibility contracts.

## CPU implementation

`bonesaw-core` now computes the minimum signed inward half-space distance from
the measured DCM to the exact eroded support polygon used by virtual-ZMP
clipping. Positive margin is inside; negative margin is outside. No secondary
polygon approximation or heap storage is used.

The new feedback law combines three cubic-smooth factors:

- capture need: full envelope authority at nonpositive DCM margin, zero at
  `+3 cm`;
- attitude health: full through `5°`, zero at `30°` root error;
- landing age: full through 150 precontact ticks, zero at 250 ticks.

Their product is a target scale. Engagement is immediate and release remains
bounded by the selected Rust tick horizon. Measured phase, signed margin,
target/applied scale, and active-coordinate count are fixed-shape outputs.
Python only selects the predeclared case and reports the arrays.

## Results

| policy | first contingency | root RMS | stance RMS | swing RMS | DCM RMS | max attitude | p99 | rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| r33 DCM, no envelope | 427 | **30.35 cm** | 51.84 cm | 47.78 cm | 43.36 cm | **64.22°** | **48.25 ms** | 0 |
| r34 envelope .10, always | 467 | 61.76 cm | 32.34 cm | **33.39 cm** | **17.24 cm** | 179.90° | 102.16 ms | 0 |
| r35 phase-only, release 200 | 455 | 68.18 cm | 40.84 cm | 33.78 cm | 10.83 cm | 179.53° | 112.26 ms | 0 |
| feedback, release 150 | **472** | 56.72 cm | 38.09 cm | 53.61 cm | 33.52 cm | 179.60° | 195.07 ms | 0 |
| feedback, release 200 | 467 | 60.68 cm | **28.44 cm** | 53.17 cm | 27.47 cm | 179.87° | 106.90 ms | 0 |
| early feedback prototype, release 100 | 449 | 45.13 cm | 33.05 cm | 47.48 cm | 26.18 cm | 145.86° | 216.62 ms | 104 |

`rejected` is primal-infeasible plus numerical-failure ticks. The early
100-tick prototype exposed an unintended single-support authority gap; the
current feedback policy retains full authority between liftoff and Precontact.
It remains as a negative control rather than an accepted result.

## What the new margin proves

The 150-tick case postpones the first contingency five ticks beyond the best
always-on control without rejected solves, while the 200-tick case produces the
best stance RMS in the family. Neither restores capturability. In the 150-tick
case the signed DCM margin reaches `-85.855 cm`, its fifth percentile is
`-82.451 cm`, and measured DCM is inside support on only `48.33%` of ticks.
The 200-tick case reaches `-110.224 cm` and is inside on `47.50%` of ticks.

This is stronger evidence than tracking error alone: by late Precontact the
state is far outside the current single-support capture region. Reweighting
tasks cannot place the support polygon under the DCM because the landing target
and phase trajectory remain fixed.

## Architectural conclusion

Feedback authority is implemented, deterministic, and useful for diagnosis,
but it is not the missing planner. The next controller slice must use signed
DCM margin and landing reachability to change the swing landing target and/or
retime the phase while it is still reachable. Authority scheduling should then
remain the smooth execution layer beneath that planner. Touchdown thresholds,
the outgoing-support retention rule, and the `1e-8` hard-row gate stay frozen.

## Regression gates

| profile | ticks | root RMS | stance RMS | swing RMS | max attitude | p99 |
|---|---:|---:|---:|---:|---:|---:|
| deterministic 1 cm toe step | 160 | 3.567 cm | 0.002 cm | 0.639 cm | 0.139° | 4.052 ms |
| moving liftoff | 260 | 0.748 cm | 0.000 cm | 0.173 cm | 2.256° | 3.906 ms |

Both remain combined green. The workspace has 89 passing Rust tests (87 core,
2 tools), strict stable Clippy with warnings denied, checked Rust formatting,
and Python bytecode validation.
