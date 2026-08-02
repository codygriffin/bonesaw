# G1 measured phase-authority experiment · r35

## Question

Revision r34 showed that an always-on joint-velocity envelope improves DCM and
swing behavior but damages the late root state. Revision r35 asks whether that
early benefit can be retained by scheduling soft authority from measured
contact state and continuously handing it back during swing.

All cases use the same pinned Unitree G1 URDF, CMU 37/01 trace, 600 × 5 ms
Rust-owned ticks, backward DCM preview, 1 cm support erosion, centroidal
angular-momentum damping at Intent, and 200-tick precontact preview. Only the
named authority policy changes.

## New CPU contract

`bonesaw-core` now provides:

- a typed measured phase: Unsupported, SingleSupport, Precontact, or
  MultiSupport;
- a validated per-phase soft-authority schedule;
- a bounded, non-overshooting scalar authority slew;
- the existing cubic-smooth joint-velocity-envelope acceleration law.

`bonesaw-py` owns persistent scalar state and preallocated active-coordinate
vectors. It emits fixed arrays for measured authority phase, applied envelope
scale, and active coordinate count. Python chooses predeclared experiment
parameters and computes reports; it does not execute per-tick policy callbacks.

Engagement is immediate. Optional release is bounded by exactly
`1 / transition_ticks` per Rust tick. Defaults remain unchanged: the envelope
weight is zero, phase policy is `always`, and root task weights retain their
previous values.

## Matrix

| policy | first contingency | root RMS | stance RMS | swing RMS | DCM RMS | max attitude | p99 | rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| r33 DCM, no envelope | 427 | **30.35 cm** | 51.84 cm | 47.78 cm | 43.36 cm | **64.22°** | **48.25 ms** | 0 |
| r34 envelope .10, always | **467** | 61.76 cm | 32.34 cm | **33.39 cm** | 17.24 cm | 179.90° | 102.16 ms | 0 |
| multi-support, hard release | 400 | 138.01 cm | 133.53 cm | 151.54 cm | 163.31 cm | 154.23° | 188.90 ms | 0 |
| symmetric 100-tick ramp | 433 | 63.16 cm | 40.15 cm | 83.55 cm | 28.62 cm | 179.11° | 269.68 ms | 89 |
| immediate engage, 100-tick release | 453 | 57.32 cm | **31.31 cm** | 78.26 cm | 40.47 cm | 179.45° | 207.59 ms | 0 |
| immediate engage, 200-tick release | 455 | 68.18 cm | 40.84 cm | 33.78 cm | **10.83 cm** | 179.53° | 112.26 ms | 0 |
| release 200, angular weight 10 | 448 | 61.05 cm | 36.73 cm | 43.77 cm | 21.97 cm | 179.35° | 196.52 ms | 0 |
| release 200, angular/height 10/1 | 444 | 73.09 cm | 39.16 cm | 46.49 cm | 37.51 cm | 162.08° | 114.80 ms | 0 |

`rejected` is primal-infeasible plus numerical-failure ticks. Typed contact
contingencies remain accepted-but-red and are not hidden in that column.

## Findings

1. A hard task edge is unsafe. Removing Viability authority at the measured
   liftoff makes contingency arrive 67 ticks earlier than the always-on case.
2. Symmetric ramping is also wrong: it weakens the known-useful double-support
   build-up and creates 89 rejected ticks.
3. Immediate engagement plus bounded release is the correct transition shape,
   but phase alone cannot select its horizon. A 100-tick release improves root
   and stance RMS but loses swing capture; 200 ticks produces the best full-run
   DCM RMS (`10.831 cm`) and preserves swing RMS, yet still flips the root.
4. Static root weighting is not the missing policy. Root height and attitude
   share Invariant; changing their weight ratio trades errors but does not
   restore the contact transfer.

The fixed task trace localizes the next input signal. Under the 200-tick release,
root-angular task residual is zero through tick 220, reaches `8.337 rad/s²` at
tick 300 and `66.750 rad/s²` at tick 400 while the controller is still in typed
Precontact, and root angle first exceeds 30° at tick 420. Increasing angular
weight delays that 30° crossing only to tick 429. Attitude is already becoming
unrecoverable before fallback.

## Architectural conclusion

Measured phase is necessary policy state but not sufficient feedback. The next
schedule must consume at least:

- signed DCM distance/margin to the measured support polygon;
- root-attitude error or root-angular task residual;
- precontact age and landing reach/velocity error;
- joint velocity-envelope utilization.

Those signals should select a continuous authority vector for capture,
attitude, height, envelope, and swing landing. A fixed phase duration or static
weight cannot represent the observed trade. Touchdown thresholds and hard-row
tolerances remain unchanged.

## Regression gates

The default paths remain combined green:

| profile | ticks | root RMS | stance RMS | swing RMS | max attitude | p99 |
|---|---:|---:|---:|---:|---:|---:|
| deterministic 1 cm toe step | 160 | 3.567 cm | 0.002 cm | 0.639 cm | 0.139° | 4.100 ms |
| moving liftoff | 260 | 0.748 cm | 0.000 cm | 0.173 cm | 2.256° | 3.973 ms |

The workspace has 88 passing Rust tests (86 core, 2 tools), strict stable
Clippy with warnings denied, checked Rust formatting, and Python bytecode
validation.

