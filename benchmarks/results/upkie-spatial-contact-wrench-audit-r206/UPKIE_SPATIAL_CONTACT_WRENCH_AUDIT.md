# Bonesaw spatial contact-wrench audit · r206

> Spatial response mechanism **PASS** · strict retained/fresh coverage **FAIL** · authority **NOT ADMITTED**.

## Contract

- The plant label integrates each wheel's complete signed force impulse and spatial moment about world origin, including contact free torque, across every 1 kHz substep. The evaluator translates that moment exactly to the causal prospective point.
- Rust emits the twelve-axis spatial response and complete Delassus operator in `[moment XYZ; force XYZ]` order. A separate Rust query maps velocity error to generalized-momentum impulse `M(q) Δv`; neither completed label is online authority.
- As a conservation cross-check, Rust maps each reconstructed contact velocity jump back through `M(q_pre)` and compares it with MuJoCo's independently recorded generalized constraint impulse, reordered to Bonesaw's `[root angular; root linear; joints]` tangent.
- All variants retain identical support hypotheses and the unchanged R203 structured acceleration reserve. This isolates whether preserving distributed-contact moment explains the R205 force-point residual.

## Exact-label decomposition

| variant | n | retained coverage | fresh coverage | max miss /s | root ω p95 | root v p95 | joint p95 | momentum residual p95 | response p99 µs |
|---|---|---|---|---|---|---|---|---|---|
| support_acceleration_only | 274 | 10.526% | 62.500% | 73.278 | 0.227 | 0.069 | 1.383 | 0.0932 | 0.000 |
| prospective_point_exact_force | 274 | 83.835% | 62.500% | 2.598 | 0.227 | 0.069 | 1.383 | 0.0664 | 7.767 |
| prospective_point_exact_spatial_wrench | 274 | 81.579% | 62.500% | 5.317 | 0.227 | 0.069 | 1.383 | 0.0664 | 11.595 |

Exact spatial moment changes retained coverage **83.835%→81.579%** and fresh coverage **62.500%→62.500%**. Contact-moment magnitude is **0.001003 N·m·s p95 / 0.009842 max**.

## Contact-impulse conservation cross-check

| variant | norm p50 | norm p95 | norm p99 | norm max |
|---|---|---|---|---|
| support_acceleration_only | 0.14153 | 0.183164 | 0.380981 | 1.06537 |
| prospective_point_exact_force | 8.79073e-06 | 0.0101201 | 0.116622 | 1.03985 |
| prospective_point_exact_spatial_wrench | 1.22074e-05 | 0.0094074 | 0.116622 | 1.03986 |

The independent `MΔv` projection costs **6.076 µs p99** with zero Rust allocation. Residual here measures pre-state Jacobian/mass reconstruction against substep-integrated plant constraint impulse; it is not the broader observed-motion residual below.

Spatial response p99 is **11.595 µs** and generalized-momentum residual p99 is **5.899 µs**, both zero-allocation. The raw mixed-unit spatial Delassus condition is **1.76e+20 p95**; it is scale-dependent diagnostic evidence, not a coordinate-invariant physical condition number.

## Decision

The spatial response and momentum-residual mechanisms may be retained, but exact completed wrench is a simulator label and this decomposition is not a reachable tube. Authority still requires causal bounded spatial-wrench witnesses, strict second-morphology/contact-law holdout, conservative residual calibration, consequence non-regression, deadline evidence and hardware realization.
