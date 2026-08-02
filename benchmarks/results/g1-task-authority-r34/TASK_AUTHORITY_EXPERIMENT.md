# G1 task-authority and liftoff experiment · r34

## Question

The r33 DCM controller is accurate through the first swing but loses the full
600-tick transfer. This experiment asks whether the loss is caused by an early
liftoff, missing joint-velocity viability, or the WBC task hierarchy.

All rows use the same pinned Unitree G1 URDF, reconstructed CMU 37/01 motion,
600 × 5 ms Rust-owned ticks, DCM backward preview, 1 cm support erosion,
centroidal angular-momentum damping, and 200-tick precontact preview. Only the
named control changes. Every full report, JSON summary, and NPZ trace is retained
beside this report under `benchmarks/results/`.

## Full-horizon results

| control | first contingency | root RMS | stance RMS | swing RMS | DCM RMS | max attitude | p99 | rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| r33 baseline | 427 | 30.35 cm | 51.84 cm | 47.78 cm | 43.36 cm | 64.22° | 48.25 ms | 0 |
| whole posture at Intent | 483 | 58.99 cm | 40.80 cm | **26.55 cm** | 20.95 cm | 128.10° | 128.14 ms | 0 |
| upper posture, Viability, weight .01 | **490** | 55.54 cm | 35.04 cm | 53.34 cm | **12.22 cm** | 178.94° | 255.67 ms | 30 |
| smooth velocity envelope, weight .10 | 467 | 61.76 cm | **32.34 cm** | 33.39 cm | 17.24 cm | 179.90° | 102.16 ms | 0 |
| smooth velocity envelope, weight .01 | 472 | 70.31 cm | 40.40 cm | 56.07 cm | 26.51 cm | 153.47° | 113.97 ms | 0 |
| Intent posture, no momentum task | 475 | 63.82 cm | 42.37 cm | 44.00 cm | 38.73 cm | 179.87° | 123.45 ms | 0 |
| Intent posture + hard joint braking | 366 | 39.62 cm | 40.01 cm | 40.25 cm | 19.47 cm | 106.47° | 186.80 ms | 68 |

`rejected` is `primal_infeasible + numerical_failure`. Contact contingencies
remain accepted-but-red states and therefore are not hidden in that column.

## Liftoff-delay falsification

The authored first liftoff is tick 199. At that tick the measured DCM is inside
the eroded outgoing sole and the DCM and swing tasks are still closely tracked,
but shoulder/elbow velocities have already consumed the 8 rad/s controller
envelope. A phase-aligned whole-reference hold was inserted at the first falling
stance edge as an eval-only falsification control.

| added hold | first contingency | root RMS | stance RMS | swing RMS | max attitude | p99 |
|---:|---:|---:|---:|---:|---:|---:|
| 0 ticks | 427 | 30.35 cm | 51.84 cm | 47.78 cm | 64.22° | 48.25 ms |
| 5 ticks | 325 | 105.35 cm | 83.89 cm | 106.12 cm | 179.22° | 186.51 ms |
| 10 ticks | 348 | 145.43 cm | 125.22 cm | 159.25 cm | 179.85° | 170.21 ms |
| 20 ticks | 393 | 124.94 cm | 110.33 cm | 145.63 cm | 179.87° | 101.28 ms |

The smallest delay makes contingency arrive 102 ticks earlier. A liftoff gate is
therefore rejected: it treats a consequence as the cause and does not recover
the joint authority already spent before tick 199.

## New CPU mechanism

`bonesaw-core` now provides an allocation-free joint-velocity-envelope law. It
uses a cubic smoothstep from zero at an authored utilization fraction to a
bounded inward acceleration at the controller velocity limit. `bonesaw-py`
builds the active coordinate subset in preallocated Rust vectors and exposes its
weight, strict priority, activation fraction, and response frequency to Python
eval land. It is disabled by default and shares the existing fixed subset-joint
task slot with protected posture; the constructor rejects simultaneous use.

The mechanism is technically sound and materially changes the failure mode: a
0.10 weight cuts DCM RMS 60%, cuts swing RMS 30%, and postpones contingency by
40 ticks without rejected solves. It does not solve the transfer because
late-horizon root attitude and tracking are worse. It therefore remains an
opt-in diagnostic/ablation, not a promoted default.

## Architectural conclusion

The failure is a coupled task-authority problem. Moving posture upward or
protecting velocity authority improves capture and contact timing, while root
tracking and attitude then lose authority. Disabling centroidal momentum or
changing root-horizontal weight in isolation is worse. The next controller
revision should make attitude/capture/joint-envelope authority phase-dependent
and jointly planned with swing landing, rather than adding another independent
always-on task or relaxing touchdown admission.

The evidence also preserves two important non-results:

- centroidal angular-momentum damping is stabilizing and stays at Intent;
- root-horizontal weight `1.0` is a sharp local optimum among tested `0`, `1`,
  and `10` values, so the rooted-motion analogy is not fixed by suppressing or
  overpowering that task.

## Regression gates

Defaults are unchanged and both dormant-path regressions remain combined green:

| profile | ticks | root RMS | stance RMS | swing RMS | max attitude | p99 |
|---|---:|---:|---:|---:|---:|---:|
| deterministic 1 cm toe step | 160 | 3.567 cm | 0.002 cm | 0.639 cm | 0.139° | 4.300 ms |
| moving liftoff | 260 | 0.748 cm | 0.000 cm | 0.173 cm | 2.256° | 3.835 ms |

The workspace has 86 passing Rust tests (84 core, 2 tools), strict stable
Clippy with warnings denied, valid architecture JSON, and Python bytecode
validation. The envelope unit test covers inactive, symmetric braking,
acceleration saturation, and invalid-limit cases.
